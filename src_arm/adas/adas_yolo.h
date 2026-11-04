/* adas_yolo.h — YOLOv8 检测解码（C 版，对接 adas_view）
 *
 * 模型规格（实测 /userdata/model/yolov8_traffic.rknn）:
 *   输入  [1,640,640,3] NHWC INT8  scale=0.003922(=1/255) zp=-128
 *   输出  9 个 = 3 尺度 × 3 分支
 *         80x80 / 40x40 / 20x20
 *         每尺度: [1, 4*reg_max, H, W] 框回归(reg_max=16 → 64 通道)
 *                 [1, num_classes, H, W] 类别分数
 *                 [1, 1, H, W]           分数和(用于快速筛选)
 *   类别数 15（与 yolov8_traffic_labels_list.txt 对应）
 *
 * 解码流程（对齐 rknn_model_zoo 参考实现）:
 *   1. 反量化: f = (q - zp) * scale
 *   2. 类别分数已是 sigmoid 后的值，直接取最大值
 *   3. 框: 4 组 × reg_max 的 DFL → softmax 加权求和 → 得到 l,t,r,b
 *      (zp_box/scale_box 已把 sigmoid 吸收进量化参数，所以直接当距离用)
 *   4. 还原到 letterbox 坐标系 → 再反算回原图
 */
#ifndef ADAS_YOLO_H
#define ADAS_YOLO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "rknn_api.h"

#define YOLO_MAX_DET   64
#define YOLO_REG_MAX   16

typedef struct {
    int   cls;
    float score;
    float x1, y1, x2, y2;    /* 原图坐标 */
} YoloBox;

typedef struct {
    YoloBox det[YOLO_MAX_DET];
    int     count;
} YoloResult;

typedef struct {
    rknn_context ctx;
    int          loaded;
    int          in_w, in_h;
    int          num_classes;
    float        box_thr;      /* 类别分数阈值, 默认 0.35 */
    float        nms_thr;      /* NMS IoU 阈值, 默认 0.45 */
    /* 缓存输出属性 */
    int          n_out;
    rknn_tensor_attr out_attr[16];
    int          out_fmt_nchw[16];
    int          out_grid[16][2];   /* [w,h] */
    int          out_kind[16];      /* 0=box 1=cls 2=sum */
    char         labels[32][64];
    int          n_labels;
} YoloDet;

static inline float yolo_deq(int8_t q, int32_t zp, float s)
{
    return ((float)q - (float)zp) * s;
}

static inline void yolo_softmax(float *x, int n)
{
    float mx = x[0];
    for (int i = 1; i < n; i++) if (x[i] > mx) mx = x[i];
    float sum = 0.f;
    for (int i = 0; i < n; i++) { x[i] = expf(x[i] - mx); sum += x[i]; }
    for (int i = 0; i < n; i++) x[i] /= sum;
}

/* 读标签文件 */
static inline void yolo_load_labels(YoloDet *d, const char *path)
{
    d->n_labels = 0;
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "[yolo] 打不开标签文件 %s\n", path); return; }
    char line[128];
    while (fgets(line, sizeof(line), f) && d->n_labels < 32) {
        int n = (int)strlen(line);
        while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r' || line[n-1] == ' ')) line[--n] = 0;
        if (n > 0) { strncpy(d->labels[d->n_labels], line, 63); d->n_labels++; }
    }
    fclose(f);
    printf("[yolo] 标签 %d 个\n", d->n_labels);
}

static inline int yolo_init(YoloDet *d, const char *model, const char *labels)
{
    memset(d, 0, sizeof(*d));
    d->box_thr = 0.15f;   /* 画面偏暗, 降低阈值以能看到检测结果 */
    d->nms_thr = 0.45f;

    FILE *fp = fopen(model, "rb");
    if (!fp) { fprintf(stderr, "[yolo] 打不开模型 %s\n", model); return 0; }
    fseek(fp, 0, SEEK_END); long sz = ftell(fp); fseek(fp, 0, SEEK_SET);
    void *buf = malloc(sz);
    if (!buf || fread(buf, 1, sz, fp) != (size_t)sz) { fclose(fp); return 0; }
    fclose(fp);

    if (rknn_init(&d->ctx, buf, sz, 0, NULL) < 0) {
        fprintf(stderr, "[yolo] rknn_init 失败\n"); free(buf); return 0;
    }
    free(buf);

    if (labels) yolo_load_labels(d, labels);
    if (d->n_labels == 0) d->n_labels = 15;   /* 兜底 */

    rknn_input_output_num io;
    if (rknn_query(d->ctx, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io)) != 0) return 0;

    rknn_tensor_attr ia; memset(&ia, 0, sizeof(ia)); ia.index = 0;
    if (rknn_query(d->ctx, RKNN_QUERY_INPUT_ATTR, &ia, sizeof(ia)) != 0) return 0;
    if (ia.fmt == RKNN_TENSOR_NHWC) { d->in_h = ia.dims[1]; d->in_w = ia.dims[2]; }
    else                            { d->in_h = ia.dims[2]; d->in_w = ia.dims[3]; }
    printf("[yolo] 输入 %dx%d, 类别数=%d, 输出数=%u\n", d->in_w, d->in_h, d->n_labels, io.n_output);

    d->n_out = (io.n_output < 16) ? (int)io.n_output : 16;
    for (int i = 0; i < d->n_out; i++) {
        memset(&d->out_attr[i], 0, sizeof(d->out_attr[i]));
        d->out_attr[i].index = i;
        rknn_query(d->ctx, RKNN_QUERY_OUTPUT_ATTR, &d->out_attr[i], sizeof(rknn_tensor_attr));
        const rknn_tensor_attr *a = &d->out_attr[i];
        d->out_fmt_nchw[i] = (a->fmt == RKNN_TENSOR_NCHW);
        /* 取 H/W: NCHW 时 dims[2],dims[3]; NHWC 时 dims[1],dims[2] */
        int H = d->out_fmt_nchw[i] ? a->dims[2] : a->dims[1];
        int W = d->out_fmt_nchw[i] ? a->dims[3] : a->dims[2];
        d->out_grid[i][0] = W;
        d->out_grid[i][1] = H;
        int C = a->dims[1];
        if (C == 4 * YOLO_REG_MAX)      d->out_kind[i] = 0;
        else if (C == d->n_labels)      d->out_kind[i] = 1;
        else                            d->out_kind[i] = 2;
        printf("   out[%d] %s C=%d %dx%d kind=%s zp=%d scale=%.6f\n",
               i, d->out_fmt_nchw[i] ? "NCHW" : "NHWC", C, W, H,
               d->out_kind[i] == 0 ? "box" : (d->out_kind[i] == 1 ? "cls" : "sum"),
               a->zp, a->scale);
    }

    d->loaded = 1;
    return 1;
}

/* letterbox: 把 src(WxH, BGR/RGB 三通道紧凑) 缩放进 dst(in_w*in_h*3, uint8, RGB) */
static inline void yolo_letterbox(const uint8_t *src, int sw, int sh,
                                  uint8_t *dst, int dw, int dh,
                                  float *scale, int *pad_x, int *pad_y)
{
    float s = (float)dw / (float)(sw > sh ? sw : sh);
    if ((float)sh * s > dh) s = (float)dh / (float)sh;
    if ((float)sw * s > dw) s = (float)dw / (float)sw;
    int nw = (int)(sw * s), nh = (int)(sh * s);
    int px = (dw - nw) / 2, py = (dh - nh) / 2;

    memset(dst, 114, (size_t)dw * dh * 3);
    for (int y = 0; y < nh; y++) {
        int sy = (int)(y / s); if (sy >= sh) sy = sh - 1;
        const uint8_t *srow = src + (size_t)sy * sw * 3;
        uint8_t *drow = dst + ((size_t)(y + py) * dw + px) * 3;
        for (int x = 0; x < nw; x++) {
            int sx = (int)(x / s); if (sx >= sw) sx = sw - 1;
            drow[x*3+0] = srow[sx*3+0];
            drow[x*3+1] = srow[sx*3+1];
            drow[x*3+2] = srow[sx*3+2];
        }
    }
    *scale = s; *pad_x = px; *pad_y = py;
}

/* IoU */
static inline float yolo_iou(const YoloBox *a, const YoloBox *b)
{
    float x1 = a->x1 > b->x1 ? a->x1 : b->x1;
    float y1 = a->y1 > b->y1 ? a->y1 : b->y1;
    float x2 = a->x2 < b->x2 ? a->x2 : b->x2;
    float y2 = a->y2 < b->y2 ? a->y2 : b->y2;
    float iw = x2 - x1, ih = y2 - y1;
    if (iw <= 0 || ih <= 0) return 0.f;
    float inter = iw * ih;
    float ua = (a->x2-a->x1)*(a->y2-a->y1) + (b->x2-b->x1)*(b->y2-b->y1) - inter;
    return ua > 0 ? inter / ua : 0.f;
}

/* 跑一帧推理 */
static inline int yolo_run(YoloDet *d, const uint8_t *rgb, int sw, int sh,
                           YoloResult *res)
{
    res->count = 0;
    if (!d->loaded) return 0;

    static uint8_t lb[640*640*3];
    float scale; int px, py;
    yolo_letterbox(rgb, sw, sh, lb, d->in_w, d->in_h, &scale, &px, &py);

    rknn_input in; memset(&in, 0, sizeof(in));
    in.index = 0; in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC; in.size = d->in_w * d->in_h * 3; in.buf = lb;
    if (rknn_inputs_set(d->ctx, 1, &in) < 0) return 0;
    if (rknn_run(d->ctx, NULL) < 0) return 0;

    rknn_output outs[16]; memset(outs, 0, sizeof(outs));
    for (int i = 0; i < d->n_out; i++) {
        outs[i].index = i; outs[i].want_float = 0;   /* 要原始 int8, 自己反量化 */
    }
    if (rknn_outputs_get(d->ctx, d->n_out, outs, NULL) < 0) return 0;

    YoloBox cand[256]; int ncand = 0;

    for (int i = 0; i < d->n_out && ncand < 256; i++) {
        if (d->out_kind[i] != 1) continue;           /* 只从 cls 分支驱动 */
        int W = d->out_grid[i][0], H = d->out_grid[i][1];
        const rknn_tensor_attr *ca = &d->out_attr[i];

        /* 找同尺度的 box 分支 */
        int bi = -1;
        for (int k = 0; k < d->n_out; k++)
            if (d->out_kind[k] == 0 && d->out_grid[k][0] == W && d->out_grid[k][1] == H) { bi = k; break; }
        if (bi < 0) continue;
        const rknn_tensor_attr *ba = &d->out_attr[bi];
        const int8_t *cptr = (const int8_t *)outs[i].buf;
        const int8_t *bptr = (const int8_t *)outs[bi].buf;

        for (int y = 0; y < H && ncand < 256; y++) {
            for (int x = 0; x < W && ncand < 256; x++) {
                /* 类别分数 */
                int best = -1; float bs = 0.f;
                for (int c = 0; c < d->n_labels; c++) {
                    float v = yolo_deq(cptr[(size_t)c * H * W + y * W + x], ca->zp, ca->scale);
                    if (v > bs) { bs = v; best = c; }
                }
                if (best < 0 || bs < d->box_thr) continue;

                /* DFL 解码: 4 组 × reg_max */
                float dfl[4];
                const int RM = YOLO_REG_MAX;
                for (int g = 0; g < 4; g++) {
                    float sm[RM]; float acc = 0.f;
                    for (int k = 0; k < RM; k++) {
                        int ch = g * RM + k;
                        sm[k] = yolo_deq(bptr[(size_t)ch * H * W + y * W + x], ba->zp, ba->scale);
                    }
                    yolo_softmax(sm, RM);
                    for (int k = 0; k < RM; k++) acc += sm[k] * (float)k;
                    dfl[g] = acc;
                }
                float l = dfl[0], t = dfl[1], rr = dfl[2], b = dfl[3];
                float gx = (float)x + 0.5f, gy = (float)y + 0.5f;
                float x1 = (gx - l) * (float)d->in_w / W;
                float y1 = (gy - t) * (float)d->in_h / H;
                float x2 = (gx + rr) * (float)d->in_w / W;
                float y2 = (gy + b) * (float)d->in_h / H;

                /* 反 letterbox → 原图坐标 */
                YoloBox nb;
                nb.cls = best; nb.score = bs;
                nb.x1 = (x1 - px) / scale;
                nb.y1 = (y1 - py) / scale;
                nb.x2 = (x2 - px) / scale;
                nb.y2 = (y2 - py) / scale;
                if (nb.x1 < 0) nb.x1 = 0; if (nb.y1 < 0) nb.y1 = 0;
                if (nb.x2 > sw) nb.x2 = sw; if (nb.y2 > sh) nb.y2 = sh;
                if (nb.x2 - nb.x1 < 6 || nb.y2 - nb.y1 < 6) continue;
                cand[ncand++] = nb;
            }
        }
    }

    rknn_outputs_release(d->ctx, d->n_out, outs);

    /* 按分数降序 + 逐类 NMS */
    for (int i = 0; i < ncand; i++)
        for (int j = i + 1; j < ncand; j++)
            if (cand[j].score > cand[i].score) { YoloBox t = cand[i]; cand[i] = cand[j]; cand[j] = t; }

    for (int i = 0; i < ncand && res->count < YOLO_MAX_DET; i++) {
        int keep = 1;
        for (int k = 0; k < res->count; k++) {
            if (res->det[k].cls == cand[i].cls && yolo_iou(&res->det[k], &cand[i]) > d->nms_thr) {
                keep = 0; break;
            }
        }
        if (keep) res->det[res->count++] = cand[i];
    }
    return res->count;
}

static inline const char *yolo_label(YoloDet *d, int cls)
{
    if (cls >= 0 && cls < d->n_labels) return d->labels[cls];
    return "?";
}

#endif /* ADAS_YOLO_H */
