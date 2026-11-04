/* rknn_info.c — 加载 rknn 模型并打印输入/输出张量规格
 *
 * 目的: 写检测解码器之前, 必须先知道模型的确切输入尺寸、输出形状和量化方式,
 *       否则类别数/通道布局会写错(原版 app 就栽在这里)。
 *
 * 编译: gcc -O2 -o rknn_info rknn_info.c -lrknnrt -ldl
 * 用法: ./rknn_info /userdata/model/yolov8_traffic.rknn
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rknn_api.h"

static const char *fmt_name(rknn_tensor_format f)
{
    switch (f) {
    case RKNN_TENSOR_NCHW: return "NCHW";
    case RKNN_TENSOR_NHWC: return "NHWC";
    default: return "?";
    }
}

static const char *type_name(rknn_tensor_type t)
{
    switch (t) {
    case RKNN_TENSOR_FLOAT32: return "FP32";
    case RKNN_TENSOR_FLOAT16: return "FP16";
    case RKNN_TENSOR_INT8:    return "INT8";
    case RKNN_TENSOR_UINT8:   return "UINT8";
    case RKNN_TENSOR_INT16:   return "INT16";
    default: return "?";
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s model.rknn\n", argv[0]); return 2; }
    setvbuf(stdout, NULL, _IONBF, 0);

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) { perror("open model"); return 1; }
    fseek(fp, 0, SEEK_END); long sz = ftell(fp); fseek(fp, 0, SEEK_SET);
    void *buf = malloc(sz);
    if (fread(buf, 1, sz, fp) != (size_t)sz) { fprintf(stderr, "read fail\n"); return 1; }
    fclose(fp);
    printf("模型: %s  (%ld 字节)\n\n", argv[1], sz);

    rknn_context ctx = 0;
    int ret = rknn_init(&ctx, buf, sz, 0, NULL);
    if (ret < 0) { fprintf(stderr, "rknn_init 失败: %d\n", ret); return 1; }
    printf("rknn_init 成功\n");

    rknn_sdk_version sdk;
    if (rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &sdk, sizeof(sdk)) == 0)
        printf("SDK: api=%s drv=%s\n", sdk.api_version, sdk.drv_version);

    rknn_input_output_num io;
    if (rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io)) != 0) {
        fprintf(stderr, "查询输入输出数量失败\n"); return 1;
    }
    printf("\n输入个数=%u  输出个数=%u\n", io.n_input, io.n_output);

    for (unsigned i = 0; i < io.n_input; i++) {
        rknn_tensor_attr a; memset(&a, 0, sizeof(a)); a.index = i;
        if (rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &a, sizeof(a)) != 0) continue;
        printf("\n[输入 %u] name=%s\n", i, a.name);
        printf("   dims=[%u,%u,%u,%u]  n_dims=%u  fmt=%s  type=%s\n",
               a.dims[0], a.dims[1], a.dims[2], a.dims[3], a.n_dims,
               fmt_name(a.fmt), type_name(a.type));
        printf("   zp=%d  scale=%f  size=%u\n", a.zp, a.scale, a.size);
        /* 推断 H/W: NHWC 时 H=dims[1] W=dims[2]; NCHW 时 H=dims[2] W=dims[3] */
        if (a.fmt == RKNN_TENSOR_NHWC)
            printf("   => 输入尺寸 H=%u W=%u C=%u\n", a.dims[1], a.dims[2], a.dims[3]);
        else
            printf("   => 输入尺寸 C=%u H=%u W=%u\n", a.dims[1], a.dims[2], a.dims[3]);
    }

    for (unsigned i = 0; i < io.n_output; i++) {
        rknn_tensor_attr a; memset(&a, 0, sizeof(a)); a.index = i;
        if (rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &a, sizeof(a)) != 0) continue;
        printf("\n[输出 %u] name=%s\n", i, a.name);
        printf("   dims=[%u,%u,%u,%u]  n_dims=%u  fmt=%s  type=%s\n",
               a.dims[0], a.dims[1], a.dims[2], a.dims[3], a.n_dims,
               fmt_name(a.fmt), type_name(a.type));
        printf("   zp=%d  scale=%f  size=%u  n_elems=%u\n",
               a.zp, a.scale, a.size, a.n_elems);
    }

    rknn_destroy(ctx);
    free(buf);
    return 0;
}
