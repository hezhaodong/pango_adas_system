/* =====================================================================
 *  pcie_check.c  —  RK3568 + Pango FPGA PCIe 采集自检工具 (纯 C, 无任何依赖)
 *
 *  为什么要有这个东西?
 *    Qt 版上位机要装 Qt 才能跑。这个工具只用 gcc 编译, 不需要 Qt / X11,
 *    专门用来把链路一层层查清楚:
 *        驱动能不能打开 -> PCIe 链路信息对不对 -> DMA 能不能收到帧
 *        -> 帧内容是不是在变(RGB565) -> 存成 BMP 用电脑打开看
 *
 *  编译 (在开发板上):
 *        gcc -O2 -Wall -o pcie_check pcie_check.c
 *  运行:
 *        sudo ./pcie_check                       # 抓 60 帧, 存到 ./frames
 *        sudo ./pcie_check -n 200 -o /userdata/frames
 *        sudo ./pcie_check -q                    # 只看设备信息, 不采集
 *        sudo ./pcie_check -f /dev/fb0           # 顺便直接在屏幕上显示
 *
 *  注意: 结构体必须和驱动逐字节一致! 内核里这些 ioctl 是
 *        copy_to_user(..., 9416) 的, 结构体小了会直接踩内存。
 *        下面的 static_assert 会在编译期挡住这种错误。
 * ===================================================================== */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

/* ---------------- 驱动接口 ---------------- */
#define PCIE_DRIVER_PATH  "/dev/pango_pci_driver"
#define MEM_DEVICE_PATH   "/dev/mem"

#define PCI_IOCTL_TYPE    'S'
#define PCI_READ_DATA_CMD        _IOWR(PCI_IOCTL_TYPE,  0, int)
#define PCI_WRITE_DATA_CMD       _IOWR(PCI_IOCTL_TYPE,  1, int)
#define PCI_MAP_ADDR_CMD         _IOWR(PCI_IOCTL_TYPE,  2, int)
#define PCI_WRITE_TO_KERNEL_CMD  _IOWR(PCI_IOCTL_TYPE,  3, int)
#define PCI_DMA_READ_CMD         _IOWR(PCI_IOCTL_TYPE,  4, int)
#define PCI_DMA_WRITE_CMD        _IOWR(PCI_IOCTL_TYPE,  5, int)
#define PCI_READ_FROM_KERNEL_CMD _IOWR(PCI_IOCTL_TYPE,  6, int)
#define PCI_UMAP_ADDR_CMD        _IOWR(PCI_IOCTL_TYPE,  7, int)
#define PCI_MAP_BAR0_CMD         _IOWR(PCI_IOCTL_TYPE, 10, int)
#define PCI_SET_CONFIG           _IOWR(PCI_IOCTL_TYPE, 11, int)

#define BAR0_MAP_SIZE      0x500
#define DMA_MAX_PACKET     4096

/* ---------------- 图像参数 (必须和 FPGA 一致) ---------------- */
#define FRAME_W            1280
#define FRAME_H            720
#define FRAME_SIZE_RGB565  (FRAME_W * FRAME_H * 2)      /* 1,843,200 */
#define FRAME_SIZE_RGB888  (FRAME_W * FRAME_H * 3)
#define RAW_SLOTS          4                             /* FPGA 4 槽乒乓 */

/* ---------------- 与驱动一致的结构体 ---------------- */
typedef struct { uint64_t bar_base, bar_len; } bar_info_t;

typedef struct {
    uint32_t vendor_id, device_id, cmd_reg, status_reg;
    uint32_t revision_id, class_prog, class_device;
    bar_info_t bar[6];
    uint32_t min_gnt, max_lat, link_speed, link_width, mps, mrrs;
    uint32_t data[1024];
} pci_info_t;                                            /* 4248 */

typedef struct { uint8_t flag, id, addr_offset, next_offset; } cap_info_t;   /* 4 */

typedef struct {
    uint8_t   cap_status, cap_error;
    cap_info_t cap_buf[256];
} cap_list_t;                                            /* 1026 */

typedef struct {
    uint32_t num_words;
    uint32_t block_words[1024];
} load_data_t;                                           /* 4100 */

typedef struct {
    uint8_t     link_status;
    uint32_t    crc;
    uint8_t     axi_direction, load_status;
    uint32_t    total_num_words;
    load_data_t data_block;
} pci_load_t;                                            /* 4116 */

typedef struct {
    uint8_t      w_r, step;
    uint32_t     addr, data, cnt, delay;
    pci_info_t   get_pci_dev_info;                       /* +24   */
    cap_list_t   cap_info;                               /* +4272 */
    pci_load_t   load_info;                              /* +5300 */
} command_op_t;                                          /* 9416  <- 关键 */

typedef struct {
    uint8_t read_buf[DMA_MAX_PACKET];
    uint8_t write_buf[DMA_MAX_PACKET];
} dma_data_t;                                            /* 8192 */

typedef struct {
    uint32_t   current_len, offset_addr, cmd;
    dma_data_t data;
} dma_op_t;                                              /* 8204 */

/* 编译期自检: 对不上就报错, 绝不让它带着错误布局去跑 */
#define SA_CAT_(a, b) a##b
#define SA_CAT(a, b)  SA_CAT_(a, b)
#define SA(cond)      typedef char SA_CAT(sa_, __LINE__)[(cond) ? 1 : -1]
SA(sizeof(bar_info_t)   ==   16);
SA(sizeof(pci_info_t)   == 4248);
SA(sizeof(cap_list_t)   == 1026);
SA(sizeof(pci_load_t)   == 4116);
SA(sizeof(command_op_t) == 9416);
SA(sizeof(dma_data_t)   == 8192);
SA(sizeof(dma_op_t)     == 8204);

/* ===================== 小工具 ===================== */
static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static uint32_t fnv1a(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

/* RGB565 -> RGB888 (位扩展, 和上位机 image_converter.cpp 一致) */
static void rgb565_to_rgb888(const uint8_t *src, uint8_t *dst)
{
    const uint16_t *s = (const uint16_t *)src;
    for (int i = 0; i < FRAME_W * FRAME_H; i++) {
        uint16_t px = s[i];
        uint8_t r5 = (px >> 11) & 0x1F;
        uint8_t g6 = (px >> 5)  & 0x3F;
        uint8_t b5 =  px        & 0x1F;
        dst[i*3 + 0] = (uint8_t)((r5 << 3) | (r5 >> 2));
        dst[i*3 + 1] = (uint8_t)((g6 << 2) | (g6 >> 4));
        dst[i*3 + 2] = (uint8_t)((b5 << 3) | (b5 >> 2));
    }
}

/* 写 24 位 BMP (BGR, 行自下而上, 每行 4 字节对齐) */
static int write_bmp(const char *path, const uint8_t *rgb888)
{
    const int rowRaw = FRAME_W * 3;
    const int rowPad = (rowRaw + 3) & ~3;
    const uint32_t dataSize = (uint32_t)rowPad * FRAME_H;
    const uint32_t fileSize = 54 + dataSize;

    FILE *f = fopen(path, "wb");
    if (!f) return -1;

    uint8_t hdr[54];
    memset(hdr, 0, sizeof(hdr));
    hdr[0] = 'B'; hdr[1] = 'M';
    memcpy(hdr + 2,  &fileSize, 4);
    uint32_t off = 54;      memcpy(hdr + 10, &off, 4);
    uint32_t ih  = 40;      memcpy(hdr + 14, &ih, 4);
    int32_t  w   = FRAME_W; memcpy(hdr + 18, &w, 4);
    int32_t  h   = FRAME_H; memcpy(hdr + 22, &h, 4);
    uint16_t pl  = 1;       memcpy(hdr + 26, &pl, 2);
    uint16_t bpp = 24;      memcpy(hdr + 28, &bpp, 2);
    memcpy(hdr + 34, &dataSize, 4);
    fwrite(hdr, 1, 54, f);

    uint8_t *row = (uint8_t *)malloc(rowPad);
    if (!row) { fclose(f); return -1; }
    memset(row, 0, rowPad);

    for (int y = FRAME_H - 1; y >= 0; y--) {
        const uint8_t *s = rgb888 + (size_t)y * rowRaw;
        for (int x = 0; x < FRAME_W; x++) {          /* RGB -> BGR */
            row[x*3 + 0] = s[x*3 + 2];
            row[x*3 + 1] = s[x*3 + 1];
            row[x*3 + 2] = s[x*3 + 0];
        }
        fwrite(row, 1, rowPad, f);
    }
    free(row);
    fclose(f);
    return 0;
}

/* ===================== framebuffer 显示 (可选) ===================== */
typedef struct {
    int      fd;
    uint8_t *mem;
    size_t   size;
    int      w, h, bpp, stride;
} fb_t;

static int fb_open(fb_t *fb, const char *path)
{
    memset(fb, 0, sizeof(*fb));
    fb->fd = open(path, O_RDWR);
    if (fb->fd < 0) { fprintf(stderr, "打开 %s 失败: %s\n", path, strerror(errno)); return -1; }

    char buf[64];
    int  fd;

    fd = open("/sys/class/graphics/fb0/virtual_size", O_RDONLY);
    if (fd < 0 || read(fd, buf, sizeof(buf)) <= 0) { fprintf(stderr, "读 virtual_size 失败\n"); if (fd>=0) close(fd); return -1; }
    close(fd);
    sscanf(buf, "%d,%d", &fb->w, &fb->h);

    fd = open("/sys/class/graphics/fb0/bits_per_pixel", O_RDONLY);
    if (fd >= 0 && read(fd, buf, sizeof(buf)) > 0) { fb->bpp = atoi(buf); close(fd); }
    else fb->bpp = 32;

    fb->stride = fb->w * (fb->bpp / 8);
    fb->size   = (size_t)fb->stride * fb->h;

    fb->mem = mmap(NULL, fb->size, PROT_READ | PROT_WRITE, MAP_SHARED, fb->fd, 0);
    if (fb->mem == MAP_FAILED) { fprintf(stderr, "mmap framebuffer 失败\n"); fb->mem = NULL; close(fb->fd); fb->fd = -1; return -1; }

    printf("[fb] %s  %dx%d  %dbpp  stride=%d\n", path, fb->w, fb->h, fb->bpp, fb->stride);
    return 0;
}

static void fb_close(fb_t *fb)
{
    if (fb->mem) munmap(fb->mem, fb->size);
    if (fb->fd >= 0) close(fb->fd);
}

/* 最近邻缩放贴图; 只支持 32bpp 的 framebuffer */
static void fb_show(fb_t *fb, const uint8_t *rgb888)
{
    if (!fb->mem || fb->bpp != 32) return;

    for (int y = 0; y < fb->h; y++) {
        int sy = y * FRAME_H / fb->h;
        uint32_t *dst = (uint32_t *)(fb->mem + (size_t)y * fb->stride);
        const uint8_t *srow = rgb888 + (size_t)sy * FRAME_W * 3;
        for (int x = 0; x < fb->w; x++) {
            int sx = x * FRAME_W / fb->w;
            const uint8_t *p = srow + (size_t)sx * 3;
            dst[x] = 0xFF000000u | ((uint32_t)p[0] << 16) |
                     ((uint32_t)p[1] << 8) | p[2];
        }
    }
}

/* ===================== 主流程 ===================== */
static void usage(const char *p)
{
    printf(
        "用法: %s [选项]\n"
        "  -n <帧数>      采集多少帧后退出 (默认 60)\n"
        "  -o <目录>      保存 BMP 的目录 (默认 ./frames)\n"
        "  -s <每N帧存一张> BMP 抽样间隔 (默认 20, 0=不存)\n"
        "  -f <fb设备>    顺便在 framebuffer 上显示, 例如 /dev/fb0\n"
        "  -q             只打印设备信息, 不采集\n"
        "  -h             帮助\n", p);
}

int main(int argc, char **argv)
{
    int   frames = 60, saveEvery = 20, quick = 0;
    char  outdir[256] = "./frames";
    char  fbdev[128]  = "";
    int   opt;

    while ((opt = getopt(argc, argv, "n:o:s:f:qh")) != -1) {
        switch (opt) {
        case 'n': frames    = atoi(optarg); break;
        case 'o': snprintf(outdir, sizeof(outdir), "%s", optarg); break;
        case 's': saveEvery = atoi(optarg); break;
        case 'f': snprintf(fbdev, sizeof(fbdev), "%s", optarg); break;
        case 'q': quick = 1; break;
        default : usage(argv[0]); return 0;
        }
    }

    printf("==================================================\n");
    printf(" RK3568 + Pango FPGA  PCIe 采集自检\n");
    printf("==================================================\n");

    /* ---- 1. 打开设备 ---- */
    int fd = open(PCIE_DRIVER_PATH, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[1] 打开 %s 失败: %s\n", PCIE_DRIVER_PATH, strerror(errno));
        fprintf(stderr, "    先确认驱动已加载:  sudo insmod pango_pci_driver_mul.ko ; lsmod | grep pango\n");
        return 1;
    }
    printf("[1] 设备已打开: %s (fd=%d)\n", PCIE_DRIVER_PATH, fd);

    /* ---- 2. 读 PCIe 设备信息 ---- */
    static command_op_t cmdOp;
    memset(&cmdOp, 0, sizeof(cmdOp));
    if (ioctl(fd, PCI_READ_DATA_CMD, &cmdOp) < 0)
        fprintf(stderr, "[2] PCI_READ_DATA_CMD 失败: %s\n", strerror(errno));

    const pci_info_t *info = &cmdOp.get_pci_dev_info;   /* 别名, 等价于 cmdOp 里的那个成员 */
    printf("[2] PCIe 设备信息:\n");
    printf("      Vendor ID  : 0x%04X\n", info->vendor_id);
    printf("      Device ID  : 0x%04X\n", info->device_id);
    printf("      Link Speed : Gen%d\n",  info->link_speed);
    printf("      Link Width : x%d\n",    info->link_width);
    printf("      MaxPayload : %u 字节\n", info->mps);
    printf("      BAR0 base  : 0x%llX  len=0x%llX\n",
           (unsigned long long)info->bar[0].bar_base,
           (unsigned long long)info->bar[0].bar_len);

    if (info->vendor_id == 0 || info->link_width == 0)
        printf("      !! 信息全 0, 可能是 FPGA 没先烧好 PCIe 程序就上电了\n"
               "         (RK 只在开机时枚举一次 PCIe, 顺序不能反)\n");

    if (quick) { close(fd); return 0; }

    /* ---- 3. 分配 DMA (驱动内部会申请 5 块 4MB 缓冲并写 BAR0) ---- */
    static dma_op_t dmaOp;
    memset(&dmaOp, 0, sizeof(dmaOp));
    ioctl(fd, PCI_SET_CONFIG, &dmaOp);
    printf("[3] DMA 缓冲已申请\n");

    /* ---- 4. 取 BAR0 物理地址并 mmap (PIO 用) ---- */
    static command_op_t barOp;
    memset(&barOp, 0, sizeof(barOp));
    if (ioctl(fd, PCI_MAP_BAR0_CMD, &barOp) < 0)
        fprintf(stderr, "[4] PCI_MAP_BAR0_CMD 失败: %s\n", strerror(errno));

    unsigned long long bar0Phys = barOp.get_pci_dev_info.bar[0].bar_base;
    long   pageSize = sysconf(_SC_PAGESIZE);
    off_t  pageOff  = (off_t)(bar0Phys & ~(unsigned long long)(pageSize - 1));
    size_t mapLen   = BAR0_MAP_SIZE + (size_t)(bar0Phys - pageOff);

    int memFd = open(MEM_DEVICE_PATH, O_RDWR | O_SYNC);
    if (memFd < 0) {
        fprintf(stderr, "[4] 打开 %s 失败: %s (需要 root)\n", MEM_DEVICE_PATH, strerror(errno));
        close(fd); return 1;
    }
    volatile uint32_t *bar0 = (volatile uint32_t *)mmap(
        NULL, mapLen, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, pageOff);
    close(memFd);
    if (bar0 == MAP_FAILED) {
        fprintf(stderr, "[4] mmap BAR0 失败: %s\n", strerror(errno));
        close(fd); return 1;
    }
    printf("[4] BAR0 已映射: phys=0x%llX -> %p (PIO 寄存器)\n",
           bar0Phys, (void *)bar0);

    /* ---- 5. 配置 DMA 长度 (640 DW = 2560 字节, 和 FPGA 一致) ---- */
    dmaOp.current_len = (FRAME_W * 2) >> 2;
    dmaOp.offset_addr = 0;
    if (ioctl(fd, PCI_MAP_ADDR_CMD, &dmaOp) < 0)
        fprintf(stderr, "[5] PCI_MAP_ADDR_CMD 失败: %s\n", strerror(errno));
    printf("[5] DMA 长度 = %u DW (%u 字节)\n",
           dmaOp.current_len, dmaOp.current_len * 4);

    /* ---- 6. 启动 FPGA 采集 ---- */
    bar0[0] = 0xFFFFFFE5u;          /* BAR0+0 写启动码 */
    printf("[6] 已写启动码 0xFFFFFFE5 -> BAR0[0]\n");

    /* ---- 7. 收帧 ---- */
    uint8_t *raw     = (uint8_t *)malloc((size_t)FRAME_SIZE_RGB565 * RAW_SLOTS);
    uint8_t *rgb888  = (uint8_t *)malloc(FRAME_SIZE_RGB888);
    if (!raw || !rgb888) { fprintf(stderr, "内存不足\n"); return 1; }

    fb_t fb;
    fb_t *fbp = NULL;
    if (fbdev[0] && fb_open(&fb, fbdev) == 0) fbp = &fb;
    if (saveEvery > 0) mkdir(outdir, 0755);

    printf("[7] 开始收帧 (目标 %d 帧)...\n", frames);
    printf("    说明: 驱动 read() 返回的是状态字, 不是字节数;\n"
           "          bit0=帧完成, bit1-2=槽位号; 数据直接落在 buf+槽位*帧大小。\n");

    int ok = 0, timeout = 0;
    uint32_t lastHash = 0, firstHash = 0;
    double t0 = now_s();

    while (ok < frames) {
        int rd = (int)read(fd, raw, FRAME_SIZE_RGB565);
        if (rd < 0) {
            if (errno == EPERM) {                 /* FPGA 还没产出帧 */
                if (++timeout > 2000) { fprintf(stderr, "\n一直收不到帧, 放弃\n"); break; }
                usleep(2000);
                continue;
            }
            fprintf(stderr, "\nread 出错: %s\n", strerror(errno));
            break;
        }
        timeout = 0;

        int done  = rd & 0x01;
        int slot  = (rd >> 1) & 0x03;
        if (!done) { usleep(1000); continue; }
        if (slot < 0 || slot >= RAW_SLOTS) slot = 0;

        const uint8_t *src = raw + (size_t)slot * FRAME_SIZE_RGB565;

        if (ok == 0) {
            firstHash = fnv1a(src, 4096);
        } else if (ok == frames / 2) {
            lastHash = fnv1a(src, 4096);
        }

        rgb565_to_rgb888(src, rgb888);

        if (fbp) fb_show(fbp, rgb888);

        if (saveEvery > 0 && (ok % saveEvery) == 0) {
            char path[512];
            snprintf(path, sizeof(path), "%s/frame_%04d.bmp", outdir, ok);
            if (write_bmp(path, rgb888) == 0)
                printf("    已保存 %s\n", path);
        }

        ok++;
        if ((ok % 10) == 0) {
            double dt = now_s() - t0;
            printf("\r    已收 %d/%d 帧, %.1f FPS, 槽位=%d   ",
                   ok, frames, ok / (dt > 0 ? dt : 1), slot);
            fflush(stdout);
        }
    }

    double total = now_s() - t0;

    /* ---- 8. 停采集, 释放 ---- */
    bar0[0] = 0xFFFFFF00u;
    printf("\n[8] 已写停止码 0xFFFFFF00 -> BAR0[0]\n");

    if (ok > 0) {
        printf("\n================ 结果 ==================\n");
        printf("收到帧数     : %d\n", ok);
        printf("总耗时       : %.2f 秒  (%.2f FPS)\n", total, ok / (total > 0 ? total : 1));
        printf("平均单帧     : %.2f ms\n", total * 1000.0 / ok);
        printf("首帧/中帧哈希: %08X / %08X  %s\n", firstHash, lastHash,
               (firstHash != lastHash) ? "(画面在变, 数据是活的)" : "(两帧一样, 可能画面静止或数据不动)");
        printf("BMP 目录     : %s\n", outdir);
        printf("=======================================\n");
    } else {
        printf("\n没收到任何帧。按顺序检查:\n"
               "  1) FPGA 的 PCIe 程序是不是在 RK 上电前就已经烧进 Flash 了?\n"
               "  2) lspcie -vv 能否看到 0755:0755 / Gen2 / x2 ?\n"
               "  3) 摄像头是否接好、I2C 是否配置成功 (看 cmos_init_done LED)?\n"
               "  4) 光纤/板对板连接器是否插紧?\n");
    }

    /* 反注册 DMA (注意: 这个驱动的 nr=7 释放大小和申请大小不一致, 若报错忽略即可) */
    ioctl(fd, PCI_UMAP_ADDR_CMD, &dmaOp);

    if (fbp) fb_close(&fb);
    munmap((void *)bar0, mapLen);
    free(raw);
    free(rgb888);
    close(fd);
    return (ok > 0) ? 0 : 2;
}
