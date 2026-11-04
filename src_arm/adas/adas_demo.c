/* adas_demo.c — 智能辅助驾驶演示: 实时车道线检测 + 决策状态机 + 画面叠加
 *
 * 组成（全部复用本工程已验证的部件）:
 *   1. PCIe 取帧      : 与 pcie_view/pcie_check 相同的驱动协议
 *   2. 车道线检测      : RGB565 -> 灰度 -> ROI -> 边缘 -> 分左右半区扫描
 *                        -> 直线拟合 -> 判断 实线/虚线/偏离
 *   3. 决策状态机      : adas_decision.h（赛题四条规则, 31 项单测通过）
 *   4. 画面叠加        : 在 XRGB8888 缓冲上画车道线/框/箭头/报警条
 *   5. 输出            : DRM/KMS 直接上屏 + 串口打印决策日志
 *
 * 设计: 感知与决策分离。 感知层只用几何方法(不依赖 NPU), 因此可以立即运行;
 *       后续接 rknn 模型时只需替换 adas_perceive() 的实现。
 *
 * 编译: gcc -O2 -I/usr/include/libdrm -o adas_demo adas_demo.c -ldrm -lm
 * 运行: ./adas_demo [seconds] [connector]     (seconds=0 表示一直跑)
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "adas_decision.h"

/* ==================== 驱动 ABI ==================== */
#define PCIE_DRIVER_PATH "/dev/pango_pci_driver"
#define MEM_DEVICE_PATH  "/dev/mem"
#define PCI_IOCTL_TYPE   'S'
#define PCI_READ_DATA_CMD   _IOWR(PCI_IOCTL_TYPE,  0, int)
#define PCI_MAP_ADDR_CMD    _IOWR(PCI_IOCTL_TYPE,  2, int)
#define PCI_MAP_BAR0_CMD    _IOWR(PCI_IOCTL_TYPE, 10, int)
#define PCI_SET_CONFIG      _IOWR(PCI_IOCTL_TYPE, 11, int)

typedef struct { uint64_t bar_base, bar_len; } bar_info_t;
typedef struct {
    uint32_t vendor_id, device_id, cmd_reg, status_reg;
    uint32_t revision_id, class_prog, class_device;
    bar_info_t bar[6];
    uint32_t min_gnt, max_lat, link_speed, link_width, mps, mrrs;
    uint32_t data[1024];
} pci_info_t;
typedef struct { uint8_t flag, id, addr_offset, next_offset; } cap_info_t;
typedef struct { uint8_t cap_status, cap_error; cap_info_t cap_buf[256]; } cap_list_t;
typedef struct { uint32_t num_words; uint32_t block_words[1024]; } load_data_t;
typedef struct {
    uint8_t link_status; uint32_t crc; uint8_t axi_direction, load_status;
    uint32_t total_num_words; load_data_t data_block;
} pci_load_t;
typedef struct {
    uint8_t w_r, step; uint32_t addr, data, cnt, delay;
    pci_info_t get_pci_dev_info; cap_list_t cap_info; pci_load_t load_info;
} cmd_op_t;
typedef struct { uint8_t read_buf[4096], write_buf[4096]; } dma_data_t;
typedef struct { uint32_t current_len, offset_addr, cmd; dma_data_t data; } dma_op_t;

_Static_assert(sizeof(cmd_op_t) == 9416, "COMMAND_OPERATION must be 9416");
_Static_assert(sizeof(dma_op_t) == 8204, "DMA_OPERATION must be 8204");

/* ==================== 帧参数 ==================== */
#define FRAME_W 1280
#define FRAME_H 720
#define FRAME_SIZE_RGB565 (FRAME_W * FRAME_H * 2)
#define RAW_SLOTS 4

/* ==================== 显示缓存 ==================== */
static uint32_t *g_disp = NULL;
static uint32_t  g_pitch_px = 0;
static uint32_t  g_disp_w = 0, g_disp_h = 0;

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s) { (void)s; g_stop = 1; }

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* ==================== 车道线检测 ==================== */
/*
 * 简化但有效的几何法:
 *   1. 取画面下部 45% 作为 ROI (车道线主要出现在这里)
 *   2. 转灰度, 做水平梯度(车道线与路面亮度差)
 *   3. 在 ROI 内按行扫描, 每行找左右各一个最强边缘点
 *   4. 把左右边缘点分别做最小二乘直线拟合
 *   5. 由拟合直线判断 实线/虚线(沿线的亮度占空比) 与 偏离
 */
typedef struct {
    int   valid;            /* 是否检测到 */
    float x_bottom, x_top;  /* 线在 ROI 底部/顶部的横坐标 */
    int   solid;            /* 1=实线 0=虚线 */
    float occupancy;        /* 亮暗占空比, 用于区分虚实 */
} LaneLine;

typedef struct {
    LaneLine left, right;
    float    center_offset_px;   /* 车道中心相对画面中心的偏移, +右 -左 */
    int      lane_width_px;

    /* 诊断信息: 现场光照差异大, 这些值用于判断"检不到"是画面问题还是阈值问题 */
    int      roi_mean_lum;       /* ROI 平均亮度 */
    int      roi_p90_grad;       /* ROI 水平梯度 90 分位 */
    int      edge_thr_used;      /* 本次实际使用的边缘门限 */
    int      left_points;        /* 左半区收集到的边缘点 */
    int      right_points;       /* 右半区收集到的边缘点 */
} LaneResult;

static inline int lum_of(uint16_t rgb565)
{
    int r = (rgb565 >> 11) & 0x1F, g = (rgb565 >> 5) & 0x3F, b = rgb565 & 0x1F;
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    return (299 * r + 587 * g + 114 * b) / 1000;
}

/* 在 [x0,x1) 内按行找边缘最强点, 用上一行的位置做连续性约束 */
static int scan_half(const uint16_t *src, int y0, int y1, int x0, int x1,
                     int prev_x, float *out_x, int *out_strength)
{
    int best_x = -1, best_s = 0;
    const int WIN = 60;                 /* 在上一行位置附近搜索 */
    int lo = prev_x - WIN, hi = prev_x + WIN;
    if (lo < x0) lo = x0;
    if (hi > x1) hi = x1;

    /* 梯度累加: 用上下几行平滑, 抗噪 */
    for (int x = lo + 1; x < hi - 1; x++) {
        int acc = 0;
        for (int y = y0; y < y1; y++) {
            const uint16_t *row = src + (size_t)y * FRAME_W;
            int l  = lum_of(row[x - 1]);
            int c  = lum_of(row[x]);
            int r_ = lum_of(row[x + 1]);
            acc += abs(r_ - l) - abs(c - (l + r_) / 2) / 2;
        }
        if (acc > best_s) { best_s = acc; best_x = x; }
    }
    if (best_x < 0) return 0;
    *out_x = (float)best_x;
    *out_strength = best_s;
    return 1;
}

/* ROI 的亮度与梯度统计, 用于自适应阈值 */
typedef struct {
    int mean_lum;
    int p90_grad;      /* 水平梯度的 90 分位 */
} RoiStats;

static void roi_stats(const uint16_t *src, int y_top, int y_bot, RoiStats *st)
{
    static int grad_hist[256];
    memset(grad_hist, 0, sizeof(grad_hist));
    long lsum = 0, ln = 0, gn = 0;

    for (int y = y_top; y < y_bot; y += 3) {
        const uint16_t *row = src + (size_t)y * FRAME_W;
        for (int x = 2; x < FRAME_W - 2; x += 3) {
            lsum += lum_of(row[x]); ln++;
            int g = abs(lum_of(row[x + 1]) - lum_of(row[x - 1]));
            if (g > 255) g = 255;
            grad_hist[g]++; gn++;
        }
    }
    st->mean_lum = ln ? (int)(lsum / ln) : 0;

    /* 90 分位梯度 */
    long want = (long)(gn * 0.90), acc = 0;
    int p90 = 0;
    for (int i = 0; i < 256; i++) {
        acc += grad_hist[i];
        if (acc >= want) { p90 = i; break; }
    }
    st->p90_grad = p90;
}

/* 沿线采样亮度, 估计是实线还是虚线 */
static void estimate_solid(const uint16_t *src, float x_bottom, float x_top,
                           int y_bottom, int y_top, LaneLine *ll)
{
    int on = 0, off = 0;
    int thr = 0;
    /* 先取整条线的平均亮度作为阈值参考 */
    long sum = 0; int n = 0;
    for (int y = y_top; y <= y_bottom; y += 3) {
        float t = (float)(y - y_top) / (float)(y_bottom - y_top + 1);
        int x = (int)(x_top + t * (x_bottom - x_top));
        if (x < 1 || x >= FRAME_W - 1) continue;
        sum += lum_of(src[(size_t)y * FRAME_W + x]);
        n++;
    }
    if (n < 8) { ll->solid = 0; ll->occupancy = 0.0f; return; }
    thr = (int)(sum / n) + 8;

    for (int y = y_top; y <= y_bottom; y += 2) {
        float t = (float)(y - y_top) / (float)(y_bottom - y_top + 1);
        int x = (int)(x_top + t * (x_bottom - x_top));
        if (x < 1 || x >= FRAME_W - 1) continue;
        /* 比较该点与其左右邻域的亮度, 判断该处是否有线 */
        int here = lum_of(src[(size_t)y * FRAME_W + x]);
        int side = (lum_of(src[(size_t)y * FRAME_W + (x - 6 > 0 ? x - 6 : 0)]) +
                    lum_of(src[(size_t)y * FRAME_W + (x + 6 < FRAME_W ? x + 6 : FRAME_W - 1)])) / 2;
        if (abs(here - side) > 18) on++; else off++;
    }
    ll->occupancy = (on + off) > 0 ? (float)on / (float)(on + off) : 0.0f;
    ll->solid = (ll->occupancy > 0.62f) ? 1 : 0;   /* 连续亮 => 实线 */
}

static void detect_lanes(const uint16_t *src, LaneResult *out)
{
    memset(out, 0, sizeof(*out));

    const int y_bot = (int)(FRAME_H * 0.95);      /* ROI 底 */
    const int y_top = (int)(FRAME_H * 0.55);      /* ROI 顶 */
    const int row_step = 6;
    const int cx = FRAME_W / 2;

    /* 自适应阈值: 现场光照差异很大(实测下半部平均亮度只有 21~34),
     * 固定阈值会在暗场景下什么都检不到。这里按 ROI 的实际梯度分位定标。 */
    RoiStats st;
    roi_stats(src, y_top, y_bot, &st);
    /* scan_half 累加 3 行的梯度, 所以门限与 p90 成比例 */
    int edge_thr = st.p90_grad * 3;
    if (edge_thr < 45)  edge_thr = 45;      /* 极暗场景放宽下限 */
    if (edge_thr > 420) edge_thr = 420;     /* 过亮/高对比场景收紧上限 */

    /* 收集左/右半区的边缘点 */
    float lxs[128], lys[128]; int ln = 0;
    float rxs[128], rys[128]; int rn = 0;

    int prev_l = cx - 160;    /* 左右各自上一行位置, 做连续性 */
    int prev_r = cx + 160;

    for (int y = y_bot; y > y_top; y -= row_step) {
        float xl, xr; int sl, sr;
        if (scan_half(src, y - 1, y + 2, 20, cx, prev_l, &xl, &sl) && sl > edge_thr) {
            if (ln < 128) { lxs[ln] = xl; lys[ln] = (float)y; ln++; }
            prev_l = (int)xl;
        }
        if (scan_half(src, y - 1, y + 2, cx, FRAME_W - 20, prev_r, &xr, &sr) && sr > edge_thr) {
            if (rn < 128) { rxs[rn] = xr; rys[rn] = (float)y; rn++; }
            prev_r = (int)xr;
        }
    }

    /* 记录本次使用的阈值与 ROI 统计, 便于现场调参 */
    out->roi_mean_lum = st.mean_lum;
    out->roi_p90_grad = st.p90_grad;
    out->edge_thr_used = edge_thr;
    out->left_points  = ln;
    out->right_points = rn;

    /* 最小二乘拟合 x = a*y + b */
    #define FIT(NAME, XS, YS, N, DST)                                             \
        do {                                                                      \
            if ((N) >= 12) {                                                       \
                double sy = 0, sx = 0, syy = 0, sxy = 0;                           \
                for (int i = 0; i < (N); i++) {                                    \
                    sy += (YS)[i]; sx += (XS)[i];                                  \
                    syy += (YS)[i] * (YS)[i]; sxy += (XS)[i] * (YS)[i];            \
                }                                                                  \
                double den = (N) * syy - sy * sy;                                  \
                if (fabs(den) > 1e-6) {                                            \
                    double a = ((N) * sxy - sx * sy) / den;                        \
                    double b = (sx - a * sy) / (N);                                \
                    (DST)->valid   = 1;                                            \
                    (DST)->x_bottom = (float)(a * y_bot + b);                      \
                    (DST)->x_top    = (float)(a * y_top + b);                      \
                }                                                                  \
            }                                                                      \
        } while (0)

    FIT(left,  lxs, lys, ln, &out->left);
    FIT(right, rxs, rys, rn, &out->right);

    if (out->left.valid)
        estimate_solid(src, out->left.x_bottom, out->left.x_top, y_bot, y_top, &out->left);
    if (out->right.valid)
        estimate_solid(src, out->right.x_bottom, out->right.x_top, y_bot, y_top, &out->right);

    if (out->left.valid && out->right.valid) {
        float lc = (out->left.x_bottom + out->left.x_top) * 0.5f;
        float rc = (out->right.x_bottom + out->right.x_top) * 0.5f;
        out->lane_width_px   = (int)fabsf(rc - lc);
        out->center_offset_px = ((lc + rc) * 0.5f) - (float)cx;
    } else if (out->left.valid || out->right.valid) {
        LaneLine *l = out->left.valid ? &out->left : &out->right;
        float c = (l->x_bottom + l->x_top) * 0.5f;
        /* 只看到一条线时, 用假定车宽估算中心 */
        float assumed = 380.0f;
        float center = out->left.valid ? (c + assumed * 0.5f) : (c - assumed * 0.5f);
        out->center_offset_px = center - (float)cx;
        out->lane_width_px = (int)assumed;
    }
}

/* ==================== 画面叠加 ==================== */
static inline void put_px(int x, int y, uint32_t color)
{
    if (!g_disp || x < 0 || y < 0) return;
    if ((uint32_t)x >= g_disp_w || (uint32_t)y >= g_disp_h) return;
    g_disp[(size_t)y * g_pitch_px + x] = color;
}

static void draw_line(int x0, int y0, int x1, int y1, uint32_t color, int thick)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        for (int t = -thick / 2; t <= thick / 2; t++)
            for (int u = -thick / 2; u <= thick / 2; u++)
                put_px(x0 + t, y0 + u, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_rect(int x, int y, int w, int h, uint32_t color, int thick)
{
    draw_line(x, y, x + w, y, color, thick);
    draw_line(x, y + h, x + w, y + h, color, thick);
    draw_line(x, y, x, y + h, color, thick);
    draw_line(x + w, y, x + w, y + h, color, thick);
}

static void fill_rect(int x, int y, int w, int h, uint32_t color)
{
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            put_px(i, j, color);
}

/* 7 段式画一个数字 (放大显示车速/状态用) */
static void draw_digit(int x, int y, int d, int scale, uint32_t color)
{
    static const uint8_t seg[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
    uint8_t s = seg[d & 0xF];
    int w = 4 * scale, h = 8 * scale;
    if (s & 0x01) fill_rect(x, y, w, scale, color);                 /* a */
    if (s & 0x02) fill_rect(x + w - scale, y, scale, h / 2, color); /* b */
    if (s & 0x04) fill_rect(x + w - scale, y + h / 2, scale, h / 2, color); /* c */
    if (s & 0x08) fill_rect(x, y + h - scale, w, scale, color);     /* d */
    if (s & 0x10) fill_rect(x, y + h / 2, scale, h / 2, color);     /* e */
    if (s & 0x20) fill_rect(x, y, scale, h / 2, color);             /* f */
    if (s & 0x40) fill_rect(x, y + h / 2 - scale / 2, w, scale, color); /* g */
}

static void draw_arrow(int cx, int cy, int dir, uint32_t color)
{
    /* dir: -1 左, 0 直行, +1 右 */
    const int L = 46;
    if (dir == 0) {
        draw_line(cx, cy + L, cx, cy - L, color, 6);
        draw_line(cx, cy - L, cx - 18, cy - L + 22, color, 6);
        draw_line(cx, cy - L, cx + 18, cy - L + 22, color, 6);
    } else {
        int tipx = cx + dir * L;
        draw_line(cx - dir * L / 2, cy + L / 2, tipx, cy - L / 2, color, 6);
        draw_line(tipx, cy - L / 2, tipx - dir * 22, cy - L / 2 + 20, color, 6);
        draw_line(tipx, cy - L / 2, tipx - dir * 26, cy - L / 2 - 16, color, 6);
    }
}

/* ==================== 上报/日志 ==================== */
static void log_decision(const LaneResult *lane, const AdasDecision *d, int frame_no)
{
    printf("[frame %d] 车道: L%s R%s 宽度=%d 偏移=%+.0fpx | 动作=%s | %s\n",
           frame_no,
           lane->left.valid  ? (lane->left.solid  ? "实线" : "虚线") : "--",
           lane->right.valid ? (lane->right.solid ? "实线" : "虚线") : "--",
           lane->lane_width_px, lane->center_offset_px,
           adas_action_name(d->action),
           d->reason ? d->reason : "");
}

/* 每 N 次打印一次感知诊断, 帮助现场判断画面质量 */
static void log_roi(const LaneResult *lane)
{
    printf("   [画面] ROI平均亮度=%d 梯度p90=%d 门限=%d 边缘点: 左%d 右%d%s\n",
           lane->roi_mean_lum, lane->roi_p90_grad, lane->edge_thr_used,
           lane->left_points, lane->right_points,
           (lane->roi_mean_lum < 25) ? "  <- 画面过暗, 建议增加照明" : "");
}

/* ==================== main ==================== */
int main(int argc, char **argv)
{
    int seconds = (argc > 1) ? atoi(argv[1]) : 0;
    const char *want_conn = (argc > 2) ? argv[2] : "HDMI-A-1";
    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    printf("===== 智能辅助驾驶演示 (车道线检测 + 决策状态机) =====\n");

    /* ---- PCIe ---- */
    int fd = open(PCIE_DRIVER_PATH, O_RDWR);
    if (fd < 0) { perror("open " PCIE_DRIVER_PATH); return 1; }
    cmd_op_t *op = calloc(1, sizeof(cmd_op_t));
    if (ioctl(fd, PCI_READ_DATA_CMD, op) < 0) { perror("READ_DATA"); return 1; }
    printf("PCIe: vendor=0x%04x device=0x%04x Gen%d x%d\n",
           op->get_pci_dev_info.vendor_id, op->get_pci_dev_info.device_id,
           op->get_pci_dev_info.link_speed, op->get_pci_dev_info.link_width);

    dma_op_t *dop = calloc(1, sizeof(dma_op_t));
    ioctl(fd, PCI_SET_CONFIG, dop);
    cmd_op_t *b0 = calloc(1, sizeof(cmd_op_t));
    if (ioctl(fd, PCI_MAP_BAR0_CMD, b0) < 0) { perror("MAP_BAR0"); return 1; }

    unsigned long bar0Phys = b0->get_pci_dev_info.bar[0].bar_base;
    long pagesz = sysconf(_SC_PAGESIZE);
    int memFd = open(MEM_DEVICE_PATH, O_RDWR | O_SYNC);
    unsigned long pageOff = bar0Phys & ~(unsigned long)(pagesz - 1);
    volatile uint32_t *bar0 = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
                                   MAP_SHARED, memFd, pageOff);
    close(memFd);
    if (bar0 == MAP_FAILED) { perror("mmap BAR0"); return 1; }
    bar0 = (volatile uint32_t *)((char *)bar0 + (bar0Phys - pageOff));

    dma_op_t *d = calloc(1, sizeof(dma_op_t));
    d->current_len = (FRAME_W * 2) >> 2;
    ioctl(fd, PCI_MAP_ADDR_CMD, d);

    /* ---- DRM 显示 ---- */
    int dfd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (dfd < 0) { perror("open card0"); return 1; }
    drmSetMaster(dfd);
    drmModeRes *res = drmModeGetResources(dfd);
    if (!res) { perror("getResources"); return 1; }

    uint32_t conn_id = 0, crtc_id = 0;
    drmModeModeInfo mode; memset(&mode, 0, sizeof(mode));
    for (int i = 0; i < res->count_connectors && !conn_id; i++) {
        drmModeConnector *c = drmModeGetConnector(dfd, res->connectors[i]);
        if (!c) continue;
        if (c->connector_type == DRM_MODE_CONNECTOR_HDMIA &&
            c->connection == DRM_MODE_CONNECTED && c->count_modes > 0) {
            conn_id = c->connector_id;
            mode = c->modes[0];
            if (c->encoder_id) {
                drmModeEncoder *e = drmModeGetEncoder(dfd, c->encoder_id);
                if (e) {
                    for (int k = 0; k < res->count_crtcs; k++)
                        if (e->possible_crtcs & (1 << k)) { crtc_id = res->crtcs[k]; break; }
                    drmModeFreeEncoder(e);
                }
            }
        }
        drmModeFreeConnector(c);
    }
    if (!conn_id || !crtc_id) { fprintf(stderr, "no HDMI connector\n"); return 1; }
    drmModeCrtc *orig = drmModeGetCrtc(dfd, crtc_id);

    g_disp_w = mode.hdisplay; g_disp_h = mode.vdisplay;
    struct drm_mode_create_dumb dumb; memset(&dumb, 0, sizeof(dumb));
    dumb.width = g_disp_w; dumb.height = g_disp_h; dumb.bpp = 32;
    if (ioctl(dfd, DRM_IOCTL_MODE_CREATE_DUMB, &dumb) != 0) { perror("CREATE_DUMB"); return 1; }
    uint32_t handles[4] = { dumb.handle, 0, 0, 0 };
    uint32_t pitches[4] = { dumb.pitch, 0, 0, 0 };
    uint32_t offsets[4] = { 0, 0, 0, 0 };
    uint32_t fb_id = 0;
    if (drmModeAddFB2(dfd, g_disp_w, g_disp_h, DRM_FORMAT_XRGB8888,
                      handles, pitches, offsets, &fb_id, 0) != 0) { perror("AddFB2"); return 1; }
    struct drm_mode_map_dumb mreq; memset(&mreq, 0, sizeof(mreq));
    mreq.handle = dumb.handle;
    if (ioctl(dfd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0) { perror("MAP_DUMB"); return 1; }
    g_disp = mmap(NULL, dumb.size, PROT_READ | PROT_WRITE, MAP_SHARED, dfd, mreq.offset);
    if (g_disp == MAP_FAILED) { perror("mmap dumb"); return 1; }
    g_pitch_px = dumb.pitch / 4;
    if (drmModeSetCrtc(dfd, crtc_id, fb_id, 0, 0, &conn_id, 1, &mode) != 0) { perror("SetCrtc"); return 1; }
    printf("display: %s %ux%u\n", mode.name, g_disp_w, g_disp_h);

    /* ---- 运行 ---- */
    uint8_t *raw = malloc((size_t)FRAME_SIZE_RGB565 * RAW_SLOTS);
    if (!raw) return 1;

    AdasStateMachine sm; adas_init(&sm);
    adas_set_command(&sm, CMD_GO);          /* 默认前进状态 */

    bar0[0] = 0xFFFFFFE5u;
    printf("FPGA 采集已启动\n");
    printf("按键模拟(串口键盘输入暂不支持, 用命令行参数 --no-go 停车)\n\n");

    if (argc > 3 && strcmp(argv[3], "--no-go") == 0)
        adas_set_command(&sm, CMD_STOP);

    double t0 = now_s(), tlast = t0;
    long frames = 0, skipped = 0;
    int lastSlot = -1, tick_counter = 0;

    while (!g_stop && (seconds <= 0 || (now_s() - t0) < seconds)) {
        int rd = (int)read(fd, raw, FRAME_SIZE_RGB565);
        if (rd < 0) { if (errno == EPERM) { usleep(1500); continue; } perror("read"); break; }
        int done = rd & 1, slot = (rd >> 1) & 3;
        if (!done || slot < 0 || slot >= RAW_SLOTS) { usleep(800); continue; }
        if (slot == lastSlot) { skipped++; usleep(500); continue; }
        lastSlot = slot;

        const uint16_t *src = (const uint16_t *)(raw + (size_t)slot * FRAME_SIZE_RGB565);

        /* --- 感知: 车道线 --- */
        LaneResult lane;
        detect_lanes(src, &lane);

        /* --- 决策 --- */
        AdasPerception p;
        memset(&p, 0, sizeof(p));
        p.lane_valid          = (lane.left.valid || lane.right.valid);
        p.lane_solid          = (lane.left.valid  && lane.left.solid) ||
                                (lane.right.valid && lane.right.solid);
        p.lane_dashed         = !p.lane_solid && p.lane_valid;
        p.person_distance_m   = -1.0f;
        p.obstacle_distance_m = -1.0f;
        p.frame_id            = (uint32_t)frames;
        /* 注: 红绿灯/行人/障碍需接 rknn 模型, 见 README 的接入说明 */

        AdasDecision dec = adas_step(&sm, &p);

        /* --- 渲染: 缩放 + 叠加 --- */
        for (uint32_t y = 0; y < g_disp_h; y++) {
            uint32_t sy = (uint32_t)((uint64_t)y * FRAME_H / g_disp_h);
            const uint16_t *srow = src + (size_t)sy * FRAME_W;
            uint32_t *drow = g_disp + (size_t)y * g_pitch_px;
            for (uint32_t x = 0; x < g_disp_w; x++) {
                uint32_t sx = (uint32_t)((uint64_t)x * FRAME_W / g_disp_w);
                uint16_t v = srow[sx];
                uint32_t r5 = (v >> 11) & 0x1F, g6 = (v >> 5) & 0x3F, b5 = v & 0x1F;
                uint32_t r8 = (r5 << 3) | (r5 >> 2);
                uint32_t g8 = (g6 << 2) | (g6 >> 4);
                uint32_t b8 = (b5 << 3) | (b5 >> 2);
                drow[x] = (r8 << 16) | (g8 << 8) | b8;
            }
        }

        /* 车道线叠加 (把 1280x720 坐标映射到显示分辨率) */
        const float kx = (float)g_disp_w / FRAME_W;
        const float ky = (float)g_disp_h / FRAME_H;
        if (lane.left.valid)
            draw_line((int)(lane.left.x_bottom * kx), (int)(FRAME_H * 0.95f * ky),
                      (int)(lane.left.x_top * kx),    (int)(FRAME_H * 0.55f * ky),
                      0x00FF00, 4);
        if (lane.right.valid)
            draw_line((int)(lane.right.x_bottom * kx), (int)(FRAME_H * 0.95f * ky),
                      (int)(lane.right.x_top * kx),    (int)(FRAME_H * 0.55f * ky),
                      0x00FF00, 4);

        /* 状态颜色条: 报警=红, 正常=绿 */
        uint32_t bar = dec.alarm_active ? 0xFF0000 : 0x00AA00;
        fill_rect(0, 0, g_disp_w, 8, bar);

        /* 转向/直行箭头指示 */
        if (dec.led.led_left)     draw_arrow(g_disp_w / 4,     g_disp_h - 90, -1, 0xFFFF00);
        if (dec.led.led_right)    draw_arrow(3 * g_disp_w / 4, g_disp_h - 90, +1, 0xFFFF00);
        if (dec.led.led_straight) draw_arrow(g_disp_w / 2,     g_disp_h - 90,  0, 0x00FFFF);

        /* 报警: 红色边框 + 闪烁 */
        if (dec.alarm_active && dec.led.alarm_blink)
            draw_rect(4, 12, g_disp_w - 9, g_disp_h - 26, 0xFF0000, 8);

        /* 停车指示 */
        if (dec.led.led_stop || dec.should_brake)
            fill_rect(g_disp_w / 2 - 90, g_disp_h / 2 - 40, 180, 80, 0xCC0000);

        /* 帧计数显示 */
        int tmp = (int)(frames % 1000);
        draw_digit(20, 24, (tmp / 100) % 10, 3, 0xFFFFFF);
        draw_digit(60, 24, (tmp / 10) % 10, 3, 0xFFFFFF);
        draw_digit(100, 24, tmp % 10, 3, 0xFFFFFF);

        frames++;
        if (++tick_counter >= 15) {          /* 每 ~15 帧打印一次决策, 避免刷屏 */
            tick_counter = 0;
            log_decision(&lane, &dec, (int)frames);
            if ((frames / 15) % 4 == 0) log_roi(&lane);
        }

        double now = now_s();
        if (now - tlast >= 5.0) {
            printf("-- fps=%.1f frames=%ld skipped=%ld | 报警统计: 红灯%d 实线%d 行人%d\n",
                   frames / (now - t0), frames, skipped,
                   sm.cnt_red_run, sm.cnt_solid_change, sm.cnt_ped_brake);
            tlast = now;
        }
    }

    bar0[0] = 0xFFFFFF00u;
    printf("\nFPGA 采集已停止. 总帧=%ld 丢帧=%ld\n", frames, skipped);
    printf("决策统计: 总%d 次 | 红灯闯行%d | 实线变道%d | 行人刹车%d\n",
           sm.cnt_total, sm.cnt_red_run, sm.cnt_solid_change, sm.cnt_ped_brake);

    if (orig) {
        drmModeSetCrtc(dfd, orig->crtc_id, orig->buffer_id, orig->x, orig->y,
                       &conn_id, 1, &orig->mode);
        drmModeFreeCrtc(orig);
    }
    drmModeRmFB(dfd, fb_id);
    struct drm_mode_destroy_dumb dd = { .handle = dumb.handle };
    ioctl(dfd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
    munmap(g_disp, dumb.size);
    drmDropMaster(dfd); close(dfd);
    free(raw); free(op); free(b0); free(dop); free(d);
    close(fd);
    return 0;
}
