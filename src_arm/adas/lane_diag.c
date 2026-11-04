/* lane_diag.c — 诊断车道线为何检不出: 打印 ROI 的亮度分布与边缘强度分布
 *
 * 目的: 车道线检测没出结果时, 需要知道是
 *   (a) 画面太暗 / 对比度太低
 *   (b) 边缘阈值定得太高
 *   (c) ROI 位置不对
 *   (d) 车道线确实不在画面里
 *
 * 编译: gcc -O2 -o lane_diag lane_diag.c
 * 运行: ./lane_diag
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

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

#define FRAME_W 1280
#define FRAME_H 720
#define FRAME_SIZE (FRAME_W * FRAME_H * 2)
#define SLOTS 4

static inline int lum_of(uint16_t v)
{
    int r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
    r = (r << 3) | (r >> 2); g = (g << 2) | (g >> 4); b = (b << 3) | (b >> 2);
    return (299 * r + 587 * g + 114 * b) / 1000;
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    int fd = open(PCIE_DRIVER_PATH, O_RDWR);
    if (fd < 0) { perror("open"); return 1; }
    cmd_op_t *op = calloc(1, sizeof(cmd_op_t));
    ioctl(fd, PCI_READ_DATA_CMD, op);
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
    if (bar0 == MAP_FAILED) { perror("mmap"); return 1; }
    bar0 = (volatile uint32_t *)((char *)bar0 + (bar0Phys - pageOff));

    dma_op_t *d = calloc(1, sizeof(dma_op_t));
    d->current_len = (FRAME_W * 2) >> 2;
    ioctl(fd, PCI_MAP_ADDR_CMD, d);

    uint8_t *raw = malloc((size_t)FRAME_SIZE * SLOTS);
    if (!raw) return 1;
    bar0[0] = 0xFFFFFFE5u;

    /* 抓 3 帧, 用最后一帧分析 */
    int got = 0;
    while (got < 3) {
        int rd = (int)read(fd, raw, FRAME_SIZE);
        if (rd < 0) { if (errno == EPERM) { usleep(1500); continue; } break; }
        if (rd & 1) got++;
    }
    int slot = ((int)read(fd, raw, FRAME_SIZE) >> 1) & 3;
    const uint16_t *src = (const uint16_t *)(raw + (size_t)slot * FRAME_SIZE);

    printf("=== 整帧亮度 ===\n");
    long hist[8] = {0}; long n = 0; long sum = 0;
    for (int y = 0; y < FRAME_H; y += 4)
        for (int x = 0; x < FRAME_W; x += 4) {
            int l = lum_of(src[(size_t)y * FRAME_W + x]);
            hist[l >> 5]++; n++; sum += l;
        }
    printf("平均亮度=%.1f\n", (double)sum / n);
    for (int i = 0; i < 8; i++)
        printf("  %3d-%3d: %5.1f%%\n", i * 32, i * 32 + 31, 100.0 * hist[i] / n);

    printf("\n=== 按行统计 (每 60 行一段): 平均亮度 / 最大水平梯度 / 强边缘数 ===\n");
    printf("  行范围      平均亮度   最大梯度   梯度>30的点数   梯度>60的点数\n");
    for (int y0 = 0; y0 < FRAME_H; y0 += 60) {
        long ls = 0, ln = 0; int gmax = 0, c30 = 0, c60 = 0;
        for (int y = y0; y < y0 + 60 && y < FRAME_H; y += 2) {
            const uint16_t *row = src + (size_t)y * FRAME_W;
            for (int x = 2; x < FRAME_W - 2; x += 2) {
                int l = lum_of(row[x]);
                ls += l; ln++;
                int g = abs(lum_of(row[x + 1]) - lum_of(row[x - 1]));
                if (g > gmax) gmax = g;
                if (g > 30) c30++;
                if (g > 60) c60++;
            }
        }
        printf("  %3d-%3d   %8.1f   %8d   %10d   %10d\n",
               y0, y0 + 59, (double)ls / ln, gmax, c30, c60);
    }

    printf("\n=== 中下部 20 行(ROI)内的水平位置分布: 梯度最强的 5 个 x ===\n");
    for (int y = 500; y <= 640; y += 35) {
        int best[5] = {0}; int bx[5] = {-1,-1,-1,-1,-1};
        const uint16_t *row = src + (size_t)y * FRAME_W;
        for (int x = 2; x < FRAME_W - 2; x++) {
            int g = abs(lum_of(row[x + 1]) - lum_of(row[x - 1]));
            for (int k = 0; k < 5; k++) {
                if (g > best[k]) {
                    for (int m = 4; m > k; m--) { best[m] = best[m-1]; bx[m] = bx[m-1]; }
                    best[k] = g; bx[k] = x;
                    break;
                }
            }
        }
        printf("  y=%3d: ", y);
        for (int k = 0; k < 5; k++) printf("x=%4d(g=%3d) ", bx[k], best[k]);
        printf("\n");
    }

    bar0[0] = 0xFFFFFF00u;
    printf("\n采集已停止\n");
    free(raw); free(op); free(b0); free(dop); free(d);
    close(fd);
    return 0;
}
