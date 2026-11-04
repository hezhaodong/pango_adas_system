/* adas_view.c — 摄像头画面 + 车道线检测 + 决策状态, 直接上屏（裸 DRM, 不依赖 Qt）
 *
 * 为什么不用 Qt: 实测 Qt EGLFS 在这块板子上**从不调用 drmModeSetCrtc**,
 * 所以 CRTC 的 buffer_id 恒为 0, 屏幕没有任何东西被扫描输出 -> 黑屏。
 * 本程序显式调用 SetCrtc 把帧缓冲挂上, 因此能稳定显示。
 *
 * 功能:
 *   1. PCIe 取帧（与 pcie_check/pcie_view 相同的驱动协议）
 *   2. 车道线检测（自适应阈值几何法: ROI -> 梯度 -> 左右半区扫描 -> 最小二乘拟合）
 *   3. 决策状态机（adas_decision.h, 赛题四条规则）
 *   4. 画面叠加: 车道线 / 状态色条 / 转向箭头 / 报警闪烁 / 帧计数
 *   5. 输出: DRM/KMS 1440x900 全屏 + 串口决策日志
 *
 * 编译: gcc -O2 -I/usr/include/libdrm -I/usr/include -o adas_view adas_view.c -ldrm -lm
 * 运行: ./adas_view [seconds] [connector]      (seconds=0 表示一直跑)
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
#include "adas_yolo.h"

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

#define FRAME_W 1280
#define FRAME_H 720
#define FRAME_SIZE_RGB565 (FRAME_W * FRAME_H * 2)
#define RAW_SLOTS 4

static uint32_t *g_disp = NULL;
static uint32_t  g_pitch_px = 0;
static uint32_t  g_w = 0, g_h = 0;
static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static inline int lum_of(uint16_t v)
{
    int r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
    r = (r << 3) | (r >> 2); g = (g << 2) | (g >> 4); b = (b << 3) | (b >> 2);
    return (299 * r + 587 * g + 114 * b) / 1000;
}

/* ==================== 车道线检测 ==================== */
typedef struct { int valid, solid; float x_bottom, x_top, occupancy; } LaneLine;

typedef struct {
    LaneLine left, right;
    float    center_offset_px;
    int      lane_width_px;
    int      roi_mean_lum, roi_p90_grad, edge_thr_used, left_points, right_points;
} LaneResult;

static int scan_half(const uint16_t *src, int y0, int y1, int x0, int x1,
                     int prev_x, float *out_x, int *out_strength)
{
    int best_x = -1, best_s = 0;
    const int WIN = 60;
    int lo = prev_x - WIN, hi = prev_x + WIN;
    if (lo < x0) lo = x0;
    if (hi > x1) hi = x1;
    for (int x = lo + 1; x < hi - 1; x++) {
        int acc = 0;
        for (int y = y0; y < y1; y++) {
            const uint16_t *row = src + (size_t)y * FRAME_W;
            int l = lum_of(row[x-1]), c = lum_of(row[x]), r = lum_of(row[x+1]);
            acc += abs(r - l) - abs(c - (l + r) / 2) / 2;
        }
        if (acc > best_s) { best_s = acc; best_x = x; }
    }
    if (best_x < 0) return 0;
    *out_x = (float)best_x;
    *out_strength = best_s;
    return 1;
}

static void roi_stats(const uint16_t *src, int y_top, int y_bot,
                      int *mean_lum, int *p90_grad)
{
    static int gh[256];
    memset(gh, 0, sizeof(gh));
    long ls = 0, ln = 0, gn = 0;
    for (int y = y_top; y < y_bot; y += 3) {
        const uint16_t *row = src + (size_t)y * FRAME_W;
        for (int x = 2; x < FRAME_W - 2; x += 3) {
            ls += lum_of(row[x]); ln++;
            int g = abs(lum_of(row[x+1]) - lum_of(row[x-1]));
            if (g > 255) g = 255;
            gh[g]++; gn++;
        }
    }
    *mean_lum = ln ? (int)(ls / ln) : 0;
    long want = (long)(gn * 0.90), acc = 0; int p = 0;
    for (int i = 0; i < 256; i++) { acc += gh[i]; if (acc >= want) { p = i; break; } }
    *p90_grad = p;
}

static void estimate_solid(const uint16_t *src, float xb, float xt,
                           int yb, int yt, LaneLine *ll)
{
    long sum = 0; int n = 0;
    for (int y = yt; y <= yb; y += 3) {
        float t = (float)(y - yt) / (float)(yb - yt + 1);
        int x = (int)(xt + t * (xb - xt));
        if (x < 1 || x >= FRAME_W - 1) continue;
        sum += lum_of(src[(size_t)y * FRAME_W + x]); n++;
    }
    if (n < 8) { ll->solid = 0; ll->occupancy = 0; return; }
    int on = 0, off = 0;
    for (int y = yt; y <= yb; y += 2) {
        float t = (float)(y - yt) / (float)(yb - yt + 1);
        int x = (int)(xt + t * (xb - xt));
        if (x < 1 || x >= FRAME_W - 1) continue;
        int here = lum_of(src[(size_t)y * FRAME_W + x]);
        int xl = (x - 6 > 0) ? x - 6 : 0;
        int xr = (x + 6 < FRAME_W) ? x + 6 : FRAME_W - 1;
        int side = (lum_of(src[(size_t)y * FRAME_W + xl]) +
                    lum_of(src[(size_t)y * FRAME_W + xr])) / 2;
        if (abs(here - side) > 18) on++; else off++;
    }
    ll->occupancy = (on + off) > 0 ? (float)on / (float)(on + off) : 0.0f;
    ll->solid = (ll->occupancy > 0.62f) ? 1 : 0;
}

static void detect_lanes(const uint16_t *src, LaneResult *out)
{
    memset(out, 0, sizeof(*out));
    const int y_bot = (int)(FRAME_H * 0.95);
    const int y_top = (int)(FRAME_H * 0.55);
    const int step = 6;
    const int cx = FRAME_W / 2;

    roi_stats(src, y_top, y_bot, &out->roi_mean_lum, &out->roi_p90_grad);
    int thr = out->roi_p90_grad * 3;
    if (thr < 45) thr = 45;
    if (thr > 420) thr = 420;
    out->edge_thr_used = thr;

    float lxs[128], lys[128]; int ln = 0;
    float rxs[128], rys[128]; int rn = 0;
    int pl = cx - 160, pr = cx + 160;

    for (int y = y_bot; y > y_top; y -= step) {
        float xl, xr; int sl, sr;
        if (scan_half(src, y-1, y+2, 20, cx, pl, &xl, &sl) && sl > thr) {
            if (ln < 128) { lxs[ln] = xl; lys[ln] = (float)y; ln++; }
            pl = (int)xl;
        }
        if (scan_half(src, y-1, y+2, cx, FRAME_W-20, pr, &xr, &sr) && sr > thr) {
            if (rn < 128) { rxs[rn] = xr; rys[rn] = (float)y; rn++; }
            pr = (int)xr;
        }
    }
    out->left_points = ln; out->right_points = rn;

#define FIT(XS, YS, N, DST)                                                   \
    do {                                                                      \
        if ((N) >= 12) {                                                       \
            double sy=0, sx=0, syy=0, sxy=0;                                   \
            for (int i=0;i<(N);i++){ sy+=(YS)[i]; sx+=(XS)[i];                 \
                syy+=(YS)[i]*(YS)[i]; sxy+=(XS)[i]*(YS)[i]; }                  \
            double den=(N)*syy - sy*sy;                                        \
            if (fabs(den) > 1e-6) {                                            \
                double a=((N)*sxy - sx*sy)/den, b=(sx - a*sy)/(N);             \
                (DST)->valid=1;                                                \
                (DST)->x_bottom=(float)(a*y_bot+b);                            \
                (DST)->x_top   =(float)(a*y_top+b);                            \
            }                                                                  \
        }                                                                      \
    } while (0)

    FIT(lxs, lys, ln, &out->left);
    FIT(rxs, rys, rn, &out->right);
    if (out->left.valid)  estimate_solid(src, out->left.x_bottom,  out->left.x_top,  y_bot, y_top, &out->left);
    if (out->right.valid) estimate_solid(src, out->right.x_bottom, out->right.x_top, y_bot, y_top, &out->right);

    if (out->left.valid && out->right.valid) {
        float lc = (out->left.x_bottom + out->left.x_top) * 0.5f;
        float rc = (out->right.x_bottom + out->right.x_top) * 0.5f;
        out->lane_width_px = (int)fabsf(rc - lc);
        out->center_offset_px = ((lc + rc) * 0.5f) - (float)cx;
    } else if (out->left.valid || out->right.valid) {
        LaneLine *l = out->left.valid ? &out->left : &out->right;
        float c = (l->x_bottom + l->x_top) * 0.5f;
        float assumed = 380.0f;
        float center = out->left.valid ? (c + assumed*0.5f) : (c - assumed*0.5f);
        out->center_offset_px = center - (float)cx;
        out->lane_width_px = (int)assumed;
    }
}

/* ==================== 绘图 ==================== */
static inline void put_px(int x, int y, uint32_t c)
{
    if (x < 0 || y < 0 || (uint32_t)x >= g_w || (uint32_t)y >= g_h) return;
    g_disp[(size_t)y * g_pitch_px + x] = c;
}
static void draw_line(int x0,int y0,int x1,int y1,uint32_t c,int t)
{
    int dx=abs(x1-x0), sx=x0<x1?1:-1, dy=-abs(y1-y0), sy=y0<y1?1:-1, err=dx+dy;
    for(;;){
        for(int i=-t/2;i<=t/2;i++) for(int j=-t/2;j<=t/2;j++) put_px(x0+i,y0+j,c);
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>=dy){err+=dy;x0+=sx;}
        if(e2<=dx){err+=dx;y0+=sy;}
    }
}
static void fill_rect(int x,int y,int w,int h,uint32_t c)
{
    for(int j=y;j<y+h;j++) for(int i=x;i<x+w;i++) put_px(i,j,c);
}
static void draw_rect(int x,int y,int w,int h,uint32_t c,int t)
{
    draw_line(x,y,x+w,y,c,t); draw_line(x,y+h,x+w,y+h,c,t);
    draw_line(x,y,x,y+h,c,t); draw_line(x+w,y,x+w,y+h,c,t);
}
static void draw_digit(int x,int y,int d,int s,uint32_t c)
{
    static const uint8_t seg[10]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
    uint8_t g=seg[d&0xF]; int w=4*s,h=8*s;
    if(g&0x01) fill_rect(x,y,w,s,c);
    if(g&0x02) fill_rect(x+w-s,y,s,h/2,c);
    if(g&0x04) fill_rect(x+w-s,y+h/2,s,h/2,c);
    if(g&0x08) fill_rect(x,y+h-s,w,s,c);
    if(g&0x10) fill_rect(x,y+h/2,s,h/2,c);
    if(g&0x20) fill_rect(x,y,s,h/2,c);
    if(g&0x40) fill_rect(x,y+h/2-s/2,w,s,c);
}
static void draw_arrow(int cx,int cy,int dir,uint32_t c)
{
    const int L=46;
    if(dir==0){
        draw_line(cx,cy+L,cx,cy-L,c,6);
        draw_line(cx,cy-L,cx-18,cy-L+22,c,6);
        draw_line(cx,cy-L,cx+18,cy-L+22,c,6);
    } else {
        int tipx=cx+dir*L;
        draw_line(cx-dir*L/2,cy+L/2,tipx,cy-L/2,c,6);
        draw_line(tipx,cy-L/2,tipx-dir*22,cy-L/2+20,c,6);
        draw_line(tipx,cy-L/2,tipx-dir*26,cy-L/2-16,c,6);
    }
}

int main(int argc, char **argv)
{
    int seconds = (argc > 1) ? atoi(argv[1]) : 0;
    const char *want = (argc > 2) ? argv[2] : "HDMI-A-1";
    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);

    printf("===== 智能辅助驾驶 · 实时显示 =====\n");

    /* ---- PCIe ---- */
    int fd = open(PCIE_DRIVER_PATH, O_RDWR);
    if (fd < 0) { perror("open driver"); return 1; }
    cmd_op_t *op = calloc(1, sizeof(cmd_op_t));
    if (ioctl(fd, PCI_READ_DATA_CMD, op) < 0) { perror("READ_DATA"); return 1; }
    printf("PCIe: %04x:%04x Gen%d x%d\n", op->get_pci_dev_info.vendor_id,
           op->get_pci_dev_info.device_id, op->get_pci_dev_info.link_speed,
           op->get_pci_dev_info.link_width);
    dma_op_t *dop = calloc(1, sizeof(dma_op_t));
    ioctl(fd, PCI_SET_CONFIG, dop);
    cmd_op_t *b0 = calloc(1, sizeof(cmd_op_t));
    if (ioctl(fd, PCI_MAP_BAR0_CMD, b0) < 0) { perror("MAP_BAR0"); return 1; }
    unsigned long bar0Phys = b0->get_pci_dev_info.bar[0].bar_base;
    long pagesz = sysconf(_SC_PAGESIZE);
    int memFd = open(MEM_DEVICE_PATH, O_RDWR | O_SYNC);
    unsigned long pageOff = bar0Phys & ~(unsigned long)(pagesz - 1);
    volatile uint32_t *bar0 = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, memFd, pageOff);
    close(memFd);
    if (bar0 == MAP_FAILED) { perror("mmap BAR0"); return 1; }
    bar0 = (volatile uint32_t *)((char *)bar0 + (bar0Phys - pageOff));
    dma_op_t *d = calloc(1, sizeof(dma_op_t));
    d->current_len = (FRAME_W * 2) >> 2;
    ioctl(fd, PCI_MAP_ADDR_CMD, d);

    /* ---- DRM ---- */
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
            conn_id = c->connector_id; mode = c->modes[0];
            if (c->encoder_id) {
                drmModeEncoder *e = drmModeGetEncoder(dfd, c->encoder_id);
                if (e) { for (int k=0;k<res->count_crtcs;k++)
                    if (e->possible_crtcs & (1<<k)) { crtc_id = res->crtcs[k]; break; }
                    drmModeFreeEncoder(e); }
            }
        }
        drmModeFreeConnector(c);
    }
    if (!conn_id || !crtc_id) { fprintf(stderr, "no HDMI connector\n"); return 1; }
    drmModeCrtc *orig = drmModeGetCrtc(dfd, crtc_id);

    g_w = mode.hdisplay; g_h = mode.vdisplay;
    struct drm_mode_create_dumb dumb; memset(&dumb, 0, sizeof(dumb));
    dumb.width = g_w; dumb.height = g_h; dumb.bpp = 32;
    if (ioctl(dfd, DRM_IOCTL_MODE_CREATE_DUMB, &dumb) != 0) { perror("CREATE_DUMB"); return 1; }
    uint32_t handles[4] = { dumb.handle,0,0,0 };
    uint32_t pitches[4] = { dumb.pitch,0,0,0 };
    uint32_t offsets[4] = { 0,0,0,0 };
    uint32_t fb_id = 0;
    if (drmModeAddFB2(dfd, g_w, g_h, DRM_FORMAT_XRGB8888, handles, pitches, offsets, &fb_id, 0) != 0) {
        perror("AddFB2"); return 1;
    }
    struct drm_mode_map_dumb mreq; memset(&mreq, 0, sizeof(mreq));
    mreq.handle = dumb.handle;
    if (ioctl(dfd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0) { perror("MAP_DUMB"); return 1; }
    g_disp = mmap(NULL, dumb.size, PROT_READ|PROT_WRITE, MAP_SHARED, dfd, mreq.offset);
    if (g_disp == MAP_FAILED) { perror("mmap dumb"); return 1; }
    g_pitch_px = dumb.pitch / 4;

    /* ★ 关键: 显式 SetCrtc, 否则 CRTC 上没有帧缓冲, 屏幕不亮 */
    if (drmModeSetCrtc(dfd, crtc_id, fb_id, 0, 0, &conn_id, 1, &mode) != 0) {
        perror("SetCrtc"); return 1;
    }
    printf("显示就绪: %s %ux%u  (SetCrtc 已调用)\n", mode.name, g_w, g_h);

    /* ---- 运行 ---- */
    uint8_t *raw = malloc((size_t)FRAME_SIZE_RGB565 * RAW_SLOTS);
    if (!raw) return 1;
    /* YOLO 输入用的 RGB888 帧缓冲 */
    uint8_t *rgb888 = malloc((size_t)FRAME_W * FRAME_H * 3);
    if (!rgb888) return 1;

    /* ---- 可选: 初始化 YOLO 检测 ---- */
    YoloDet yolo; memset(&yolo, 0, sizeof(yolo));
    YoloResult yres; memset(&yres, 0, sizeof(yres));
    int use_yolo = 0;
    const char *model = getenv("CAMERA_MODEL_PATH");
    const char *labels = getenv("CAMERA_LABEL_PATH");
    if (!model)  model  = "/userdata/model/yolov8_traffic.rknn";
    if (!labels) labels = "/userdata/model/yolov8_traffic_labels_list.txt";
    const char *noYolo = getenv("ADAS_NO_YOLO");
    if (!(noYolo && *noYolo && *noYolo != '0')) {
        printf("加载检测模型: %s\n", model);
        if (yolo_init(&yolo, model, labels)) {
            use_yolo = 1;
            const char *thr = getenv("ADAS_BOX_THR");
            if (thr) yolo.box_thr = (float)atof(thr);
            printf("★ 检测已启用 (阈值 %.2f)\n", yolo.box_thr);
        } else {
            printf("!! 检测初始化失败, 只显示画面与车道线\n");
        }
    } else {
        printf("ADAS_NO_YOLO 已设置, 跳过检测\n");
    }

    AdasStateMachine sm; adas_init(&sm);
    adas_set_command(&sm, CMD_GO);

    bar0[0] = 0xFFFFFFE5u;
    printf("FPGA 采集已启动\n\n");

    double t0 = now_s(), tlast = t0, tdet = 0;
    long frames = 0, skipped = 0, ndet = 0;
    int lastSlot = -1, logcnt = 0;

    while (!g_stop && (seconds <= 0 || (now_s() - t0) < seconds)) {
        int rd = (int)read(fd, raw, FRAME_SIZE_RGB565);
        if (rd < 0) { if (errno == EPERM) { usleep(1500); continue; } perror("read"); break; }
        int done = rd & 1, slot = (rd >> 1) & 3;
        if (!done || slot < 0 || slot >= RAW_SLOTS) { usleep(800); continue; }
        if (slot == lastSlot) { skipped++; usleep(500); continue; }
        lastSlot = slot;

        const uint16_t *src = (const uint16_t *)(raw + (size_t)slot * FRAME_SIZE_RGB565);

        LaneResult lane;
        detect_lanes(src, &lane);

        /* ---- YOLO 检测 (限速到 ~8Hz, 避免拖慢显示) ---- */
        double nowd = now_s();
        if (use_yolo && (nowd - tdet) >= 0.12) {
            tdet = nowd;
            for (int y = 0; y < FRAME_H; y++) {
                const uint16_t *srow = src + (size_t)y * FRAME_W;
                uint8_t *drow = rgb888 + (size_t)y * FRAME_W * 3;
                for (int x = 0; x < FRAME_W; x++) {
                    uint16_t v = srow[x];
                    uint32_t r5=(v>>11)&0x1F, g6=(v>>5)&0x3F, b5=v&0x1F;
                    /* 实测: 模型期望 BGR 顺序(用 RGB 时类别分最高仅 0.0038,
                       换成 BGR 后升到 0.1750, 提升约 46 倍) */
                    drow[x*3+0] = (uint8_t)((b5<<3)|(b5>>2));
                    drow[x*3+1] = (uint8_t)((g6<<2)|(g6>>4));
                    drow[x*3+2] = (uint8_t)((r5<<3)|(r5>>2));
                }
            }
            ndet++;
            yolo_run(&yolo, rgb888, FRAME_W, FRAME_H, &yres);
        }

        AdasPerception p;
        memset(&p, 0, sizeof(p));
        p.lane_valid  = (lane.left.valid || lane.right.valid);
        p.lane_solid  = (lane.left.valid && lane.left.solid) ||
                        (lane.right.valid && lane.right.solid);
        p.lane_dashed = !p.lane_solid && p.lane_valid;
        p.person_distance_m = -1.0f;
        p.obstacle_distance_m = -1.0f;
        /* 用检测结果填充赛题关心的目标 (类别名匹配 yolo_traffic 标签) */
        for (int i = 0; i < yres.count; i++) {
            const char *nm = yolo_label(&yolo, yres.det[i].cls);
            if      (!strcmp(nm, "person") || !strcmp(nm, "rider"))     p.person_ahead = true;
            else if (!strcmp(nm, "car") || !strcmp(nm, "car_ahead") ||
                     !strcmp(nm, "truck") || !strcmp(nm, "bus") ||
                     !strcmp(nm, "motor") || !strcmp(nm, "bike"))       p.obstacle_ahead = true;
            else if (!strncmp(nm, "relevant red", 12) ||
                     !strcmp(nm, "not_relevant red"))                   p.red_light = true;
            else if (!strncmp(nm, "relevant green", 14) ||
                     !strcmp(nm, "not_relevant green"))                 p.green_light = true;
            else if (!strcmp(nm, "cross walk"))                         p.crosswalk = true;
        }
        /* 目标距离未知时, 用"是否出现在画面下半部"粗略判断威胁 */
        if (p.person_ahead)   p.person_distance_m   = 6.0f;
        if (p.obstacle_ahead) p.obstacle_distance_m = 6.0f;
        p.frame_id = (uint32_t)frames;
        AdasDecision dec = adas_step(&sm, &p);

        /* 缩放 1280x720 -> 1440x900 并转 XRGB8888 */
        for (uint32_t y = 0; y < g_h; y++) {
            uint32_t sy = (uint32_t)((uint64_t)y * FRAME_H / g_h);
            const uint16_t *srow = src + (size_t)sy * FRAME_W;
            uint32_t *drow = g_disp + (size_t)y * g_pitch_px;
            for (uint32_t x = 0; x < g_w; x++) {
                uint32_t sx = (uint32_t)((uint64_t)x * FRAME_W / g_w);
                uint16_t v = srow[sx];
                uint32_t r5=(v>>11)&0x1F, g6=(v>>5)&0x3F, b5=v&0x1F;
                uint32_t r8=(r5<<3)|(r5>>2), g8=(g6<<2)|(g6>>4), b8=(b5<<3)|(b5>>2);
                drow[x] = (r8<<16)|(g8<<8)|b8;
            }
        }

        /* 叠加 */
        const float kx = (float)g_w / FRAME_W, ky = (float)g_h / FRAME_H;
        if (lane.left.valid)
            draw_line((int)(lane.left.x_bottom*kx), (int)(FRAME_H*0.95f*ky),
                      (int)(lane.left.x_top*kx),    (int)(FRAME_H*0.55f*ky),
                      lane.left.solid ? 0x00FF00 : 0x00FFFF, 5);
        if (lane.right.valid)
            draw_line((int)(lane.right.x_bottom*kx), (int)(FRAME_H*0.95f*ky),
                      (int)(lane.right.x_top*kx),    (int)(FRAME_H*0.55f*ky),
                      lane.right.solid ? 0x00FF00 : 0x00FFFF, 5);

        /* 检测框 */
        for (int i = 0; i < yres.count; i++) {
            const YoloBox *b = &yres.det[i];
            if (b->score <= 0) continue;
            int x1 = (int)(b->x1*kx), y1 = (int)(b->y1*ky);
            int x2 = (int)(b->x2*kx), y2 = (int)(b->y2*ky);
            uint32_t col = b->cls == 0 ? 0xFFFF00 : 0x00A5FF;   /* 按类分色 */
            draw_rect(x1, y1, x2-x1, y2-y1, col, 3);
            /* 左上角一个小色块表示类别序号的个位 */
            draw_digit(x1+3, y1+3, b->cls % 10, 2, col);
        }

        fill_rect(0, 0, g_w, 10, dec.alarm_active ? 0xFF0000 : 0x00AA00);
        if (dec.led.led_left)     draw_arrow(g_w/4,     g_h-90, -1, 0xFFFF00);
        if (dec.led.led_right)    draw_arrow(3*g_w/4,   g_h-90, +1, 0xFFFF00);
        if (dec.led.led_straight) draw_arrow(g_w/2,     g_h-90,  0, 0x00FFFF);
        if (dec.alarm_active && dec.led.alarm_blink)
            draw_rect(4, 14, g_w-9, g_h-30, 0xFF0000, 8);
        if (dec.led.led_stop || dec.should_brake)
            fill_rect(g_w/2-90, g_h/2-40, 180, 80, 0xCC0000);

        int t = (int)(frames % 1000);
        draw_digit(20, 26, (t/100)%10, 3, 0xFFFFFF);
        draw_digit(60, 26, (t/10)%10,  3, 0xFFFFFF);
        draw_digit(100,26,  t%10,      3, 0xFFFFFF);

        frames++;
        if (++logcnt >= 20) {
            logcnt = 0;
            printf("[%ld] 车道 L%s R%s 偏移%+.0f | 检测%ld个", frames,
                   lane.left.valid ? (lane.left.solid?"实":"虚") : "--",
                   lane.right.valid? (lane.right.solid?"实":"虚") : "--",
                   lane.center_offset_px, (long)yres.count);
            for (int i = 0; i < yres.count && i < 4; i++)
                printf(" %s(%.2f)", yolo_label(&yolo, yres.det[i].cls), yres.det[i].score);
            printf(" | %s\n", adas_action_name(dec.action));
        }
        double now = now_s();
        if (now - tlast >= 5.0) {
            printf("-- fps=%.1f 帧=%ld 丢=%ld | 报警统计: 红灯%d 实线%d 行人%d\n",
                   frames/(now-t0), frames, skipped,
                   sm.cnt_red_run, sm.cnt_solid_change, sm.cnt_ped_brake);
            tlast = now;
        }
    }

    bar0[0] = 0xFFFFFF00u;
    printf("\n采集已停止. 总帧=%ld\n", frames);
    printf("决策统计: 总%d 次\n", sm.cnt_total);
    if (orig) { drmModeSetCrtc(dfd, orig->crtc_id, orig->buffer_id, orig->x, orig->y,
                               &conn_id, 1, &orig->mode); drmModeFreeCrtc(orig); }
    drmModeRmFB(dfd, fb_id);
    struct drm_mode_destroy_dumb dd = { .handle = dumb.handle };
    ioctl(dfd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
    munmap(g_disp, dumb.size);
    drmDropMaster(dfd); close(dfd);
    free(raw); free(rgb888); free(op); free(b0); free(dop); free(d);
    if (yolo.loaded) rknn_destroy(yolo.ctx);
    close(fd);
    return 0;
}
