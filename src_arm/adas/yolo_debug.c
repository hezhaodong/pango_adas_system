/* yolo_debug.c — 单帧推理诊断: 打印原始类别分数与解码后的框
 *
 * 用来区分两种情况:
 *   (a) 模型确实没检出东西（分数普遍很低）-> 画质/场景问题
 *   (b) 模型检出了但解码公式写错（分数高但框不对）-> 解码 bug
 *
 * 编译: gcc -O2 -o yolo_debug yolo_debug.c -lrknnrt -ldl
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
#include <math.h>
#include "rknn_api.h"

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
#define FRAME_SIZE (FRAME_W*FRAME_H*2)
#define SLOTS 4

static const char *LABELS[32]; static int NLAB = 0;

static void load_labels(const char *p)
{
    FILE *f = fopen(p, "r");
    if (!f) return;
    static char buf[32][64];
    char line[128];
    while (fgets(line, sizeof(line), f) && NLAB < 32) {
        int n = strlen(line);
        while (n>0 && (line[n-1]=='\n'||line[n-1]=='\r')) line[--n]=0;
        if (n>0) { strncpy(buf[NLAB], line, 63); LABELS[NLAB] = buf[NLAB]; NLAB++; }
    }
    fclose(f);
}

int main(int argc, char **argv)
{
    const char *model  = (argc > 1) ? argv[1] : "/userdata/model/yolov8_traffic.rknn";
    const char *labels = (argc > 2) ? argv[2] : "/userdata/model/yolov8_traffic_labels_list.txt";
    setvbuf(stdout, NULL, _IONBF, 0);
    load_labels(labels);
    printf("标签 %d 个\n", NLAB);

    /* ---- 抓一帧 ---- */
    int fd = open(PCIE_DRIVER_PATH, O_RDWR);
    if (fd < 0) { perror("open driver"); return 1; }
    cmd_op_t *op = calloc(1, sizeof(cmd_op_t));
    ioctl(fd, PCI_READ_DATA_CMD, op);
    dma_op_t *dop = calloc(1, sizeof(dma_op_t));
    ioctl(fd, PCI_SET_CONFIG, dop);
    cmd_op_t *b0 = calloc(1, sizeof(cmd_op_t));
    if (ioctl(fd, PCI_MAP_BAR0_CMD, b0) < 0) { perror("MAP_BAR0"); return 1; }
    unsigned long bar0Phys = b0->get_pci_dev_info.bar[0].bar_base;
    long ps = sysconf(_SC_PAGESIZE);
    int mf = open(MEM_DEVICE_PATH, O_RDWR|O_SYNC);
    unsigned long po = bar0Phys & ~(unsigned long)(ps-1);
    volatile uint32_t *bar0 = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mf, po);
    close(mf);
    bar0 = (volatile uint32_t *)((char*)bar0 + (bar0Phys - po));
    dma_op_t *d = calloc(1, sizeof(dma_op_t));
    d->current_len = (FRAME_W*2)>>2;
    ioctl(fd, PCI_MAP_ADDR_CMD, d);

    uint8_t *raw = malloc((size_t)FRAME_SIZE*SLOTS);
    bar0[0] = 0xFFFFFFE5u;
    int got = 0, slot = 0;
    for (int i = 0; i < 300 && got < 6; i++) {
        int rd = (int)read(fd, raw, FRAME_SIZE);
        if (rd < 0) { if (errno==EPERM) { usleep(1500); continue; } break; }
        if (rd & 1) { got++; slot = (rd>>1)&3; }
    }
    bar0[0] = 0xFFFFFF00u;
    printf("抓到 %d 帧, 用 slot=%d\n", got, slot);
    if (!got) return 1;
    const uint16_t *src = (const uint16_t*)(raw + (size_t)slot*FRAME_SIZE);

    /* ---- RGB565 -> RGB888 + letterbox ---- */
    static uint8_t rgb[FRAME_W*FRAME_H*3];
    /* ADAS_BGR=1 时交换 R/B 通道, 用于验证模型期望的颜色顺序 */
    const int use_bgr = (getenv("ADAS_BGR") && getenv("ADAS_BGR")[0] == '1');
    printf("颜色顺序: %s\n", use_bgr ? "BGR" : "RGB");
    for (int y=0;y<FRAME_H;y++){
        const uint16_t *s = src + (size_t)y*FRAME_W;
        uint8_t *o = rgb + (size_t)y*FRAME_W*3;
        for(int x=0;x<FRAME_W;x++){ uint16_t v=s[x];
            uint32_t r5=(v>>11)&0x1F,g6=(v>>5)&0x3F,b5=v&0x1F;
            uint8_t R=(r5<<3)|(r5>>2), G=(g6<<2)|(g6>>4), B=(b5<<3)|(b5>>2);
            o[x*3+0]= use_bgr ? B : R;
            o[x*3+1]= G;
            o[x*3+2]= use_bgr ? R : B;
        }
    }
    static uint8_t lb[640*640*3];
    memset(lb,114,sizeof(lb));
    float sc = 640.0f/FRAME_W;              /* 1280x720 -> 640x360, 上下补 140 */
    int nw = 640, nh = (int)(FRAME_H*sc), px = 0, py = (640-nh)/2;
    for(int y=0;y<nh;y++){
        int sy=(int)(y/sc); if(sy>=FRAME_H) sy=FRAME_H-1;
        const uint8_t *s = rgb + (size_t)sy*FRAME_W*3;
        uint8_t *o = lb + ((size_t)(y+py)*640 + px)*3;
        for(int x=0;x<nw;x++){ int sx=(int)(x/sc); if(sx>=FRAME_W) sx=FRAME_W-1;
            o[x*3]=s[sx*3]; o[x*3+1]=s[sx*3+1]; o[x*3+2]=s[sx*3+2]; }
    }

    /* ---- 推理 ---- */
    rknn_context ctx;
    FILE *fp=fopen(model,"rb"); if(!fp){perror("model");return 1;}
    fseek(fp,0,SEEK_END); long sz=ftell(fp); fseek(fp,0,SEEK_SET);
    void *mb=malloc(sz); if(fread(mb,1,sz,fp)!=(size_t)sz){return 1;} fclose(fp);
    if (rknn_init(&ctx, mb, sz, 0, NULL) < 0) { printf("rknn_init fail\n"); return 1; }

    rknn_input_output_num io;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io));

    rknn_input in; memset(&in,0,sizeof(in));
    in.index=0; in.type=RKNN_TENSOR_UINT8; in.fmt=RKNN_TENSOR_NHWC;
    in.size=640*640*3; in.buf=lb;
    rknn_inputs_set(ctx,1,&in);
    rknn_run(ctx,NULL);

    rknn_output outs[16]; memset(outs,0,sizeof(outs));
    for (unsigned i=0;i<io.n_output;i++){ outs[i].index=i; outs[i].want_float=0; }
    rknn_outputs_get(ctx, io.n_output, outs, NULL);

    rknn_tensor_attr oa[16];
    for (unsigned i=0;i<io.n_output;i++){
        memset(&oa[i],0,sizeof(oa[i])); oa[i].index=i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &oa[i], sizeof(rknn_tensor_attr));
    }

    /* ---- 诊断: 对 80x80 尺度, 逐 anchor 找全局最高类别分 ---- */
    printf("\n=== 类别分数分布 (三个尺度, 每尺度报最高分) ===\n");
    for (unsigned i = 0; i < io.n_output; i++) {
        if (oa[i].dims[1] != (uint32_t)NLAB) continue;
        int C = oa[i].dims[1], H = oa[i].dims[2], W = oa[i].dims[3];
        const int8_t *p = (const int8_t*)outs[i].buf;
        float best = -1e9f; int bc = -1, bx = 0, by = 0;
        for (int y=0;y<H;y++) for (int x=0;x<W;x++) for (int c=0;c<C;c++) {
            float v = ((float)p[(size_t)c*H*W + y*W + x] - (float)oa[i].zp) * oa[i].scale;
            if (v > best) { best=v; bc=c; bx=x; by=y; }
        }
        printf("  尺度 %dx%d: 全局最高分 %.4f (类别=%s, 位置 %d,%d)\n",
               W, H, best, (bc>=0&&bc<NLAB)?LABELS[bc]:"?", bx, by);

        /* 统计有多少 anchor 超过 0.25 */
        int cnt = 0;
        for (int y=0;y<H;y++) for (int x=0;x<W;x++) {
            float m = -1e9f;
            for (int c=0;c<C;c++) {
                float v = ((float)p[(size_t)c*H*W + y*W + x] - (float)oa[i].zp) * oa[i].scale;
                if (v>m) m=v;
            }
            if (m > 0.25f) cnt++;
        }
        printf("          超过 0.25 的 anchor 数: %d\n", cnt);
    }

    rknn_outputs_release(ctx, io.n_output, outs);
    rknn_destroy(ctx);
    free(raw); free(op); free(b0); free(dop); free(d); free(mb);
    close(fd);
    return 0;
}
