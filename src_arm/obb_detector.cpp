#include "obb_detector.h"
#include "obb_postprocess.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>

// ---- 构造/析构 ----
ObbDetector::ObbDetector()
    : m_ctx(0), m_inputAttrs(nullptr), m_outputAttrs(nullptr),
      m_loaded(false), m_isQuant(false),
      m_letterboxBuf(nullptr), m_bufSize(0) {
    memset(&m_ioNum, 0, sizeof(m_ioNum));
}

ObbDetector::~ObbDetector() {
    release();
}

// ---- 加载模型 ----
bool ObbDetector::loadModel(const char *modelPath, const char *labelPath,
                             const ObbModelParams &params) {
    m_params = params;
    m_labels.clear();
    // 1. 读取标签
    FILE *fp = fopen(labelPath, "r");
    if (!fp) {
        fprintf(stderr, "[ObbDetector] 无法打开标签文件: %s\n", labelPath);
        return false;
    }
    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        int len = (int)strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';
        if (len > 0) m_labels.push_back(std::string(line));
    }
    fclose(fp);

    // 从标签文件自动确定类别数
    m_params.numClasses = (int)m_labels.size();
    // 2. 初始化 RKNN
    int ret = rknn_init(&m_ctx, (void *)modelPath, 0, 0, nullptr);
    if (ret < 0) {
        fprintf(stderr, "[ObbDetector] rknn_init 失败: %d\n", ret);
        return false;
    }

    // 3. 查询输入输出信息
    ret = rknn_query(m_ctx, RKNN_QUERY_IN_OUT_NUM, &m_ioNum, sizeof(m_ioNum));
    if (ret < 0) {
        fprintf(stderr, "[ObbDetector] rknn_query IO num 失败: %d\n", ret);
        goto fail;
    }

    m_inputAttrs  = new rknn_tensor_attr[m_ioNum.n_input];
    m_outputAttrs = new rknn_tensor_attr[m_ioNum.n_output];

    for (int i = 0; i < (int)m_ioNum.n_input; i++) {
        m_inputAttrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_INPUT_ATTR, &m_inputAttrs[i],
                         sizeof(rknn_tensor_attr));
        if (ret < 0) { fprintf(stderr, "[ObbDetector] query input %d err\n", i); goto fail; }
    }
    for (int i = 0; i < (int)m_ioNum.n_output; i++) {
        m_outputAttrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_OUTPUT_ATTR, &m_outputAttrs[i],
                         sizeof(rknn_tensor_attr));
        if (ret < 0) { fprintf(stderr, "[ObbDetector] query output %d err\n", i); goto fail; }
    }

#ifdef ARM_FINAL_MODEL_DIAGNOSTICS
    // ================================================================
    // 诊断: 查询 NPU Native 属性，对比逻辑属性，定位精度损失根因
    // ================================================================
    // RKNN API 提供多组查询:
    //   RKNN_QUERY_INPUT_ATTR  / OUTPUT_ATTR           → 模型逻辑格式
    //   RKNN_QUERY_NATIVE_NC1HWC2_INPUT_ATTR  (=8)     → NPU 原生 NC1HWC2 格式
    //   RKNN_QUERY_NATIVE_NC1HWC2_OUTPUT_ATTR (=9)
    //   RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR      (=10)   → NPU 原生 NHWC 格式
    //   RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR     (=11)
    //
    // 如果逻辑格式 ≠ 原生格式，驱动会在每次推理做隐式转换，这是精度损失的
    // 首要嫌疑点。以下打印所有格式的完整信息用于对比诊断。

    // --- Native NC1HWC2 输入 ---
    rknn_tensor_attr nativeNc1hwc2In;
    memset(&nativeNc1hwc2In, 0, sizeof(nativeNc1hwc2In));
    nativeNc1hwc2In.index = 0;
    ret = rknn_query(m_ctx, RKNN_QUERY_NATIVE_NC1HWC2_INPUT_ATTR,
                     &nativeNc1hwc2In, sizeof(nativeNc1hwc2In));
    printf("[ObbDetector] === Native NC1HWC2 输入属性 ===\n");
    if (ret == 0) {
        printf("[ObbDetector]   fmt=%s type=%s qnt=%s n_dims=%u\n",
               get_format_string(nativeNc1hwc2In.fmt),
               get_type_string(nativeNc1hwc2In.type),
               get_qnt_type_string(nativeNc1hwc2In.qnt_type),
               nativeNc1hwc2In.n_dims);
        printf("[ObbDetector]   dims=[%u,%u,%u,%u] n_elems=%u size=%u size_with_stride=%u\n",
               nativeNc1hwc2In.dims[0], nativeNc1hwc2In.dims[1],
               nativeNc1hwc2In.dims[2], nativeNc1hwc2In.dims[3],
               nativeNc1hwc2In.n_elems, nativeNc1hwc2In.size,
               nativeNc1hwc2In.size_with_stride);
        printf("[ObbDetector]   w_stride=%u h_stride=%u pass_through=%u zp=%d scale=%f\n",
               nativeNc1hwc2In.w_stride, nativeNc1hwc2In.h_stride,
               nativeNc1hwc2In.pass_through,
               nativeNc1hwc2In.zp, nativeNc1hwc2In.scale);
    } else {
        printf("[ObbDetector]   (查询失败, ret=%d)\n", ret);
    }

    // --- Native NHWC 输入 ---
    rknn_tensor_attr nativeNhwcIn;
    memset(&nativeNhwcIn, 0, sizeof(nativeNhwcIn));
    nativeNhwcIn.index = 0;
    ret = rknn_query(m_ctx, RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR,
                     &nativeNhwcIn, sizeof(nativeNhwcIn));
    printf("[ObbDetector] === Native NHWC 输入属性 ===\n");
    if (ret == 0) {
        printf("[ObbDetector]   fmt=%s type=%s qnt=%s n_dims=%u\n",
               get_format_string(nativeNhwcIn.fmt),
               get_type_string(nativeNhwcIn.type),
               get_qnt_type_string(nativeNhwcIn.qnt_type),
               nativeNhwcIn.n_dims);
        printf("[ObbDetector]   dims=[%u,%u,%u,%u] n_elems=%u size=%u size_with_stride=%u\n",
               nativeNhwcIn.dims[0], nativeNhwcIn.dims[1],
               nativeNhwcIn.dims[2], nativeNhwcIn.dims[3],
               nativeNhwcIn.n_elems, nativeNhwcIn.size,
               nativeNhwcIn.size_with_stride);
        printf("[ObbDetector]   w_stride=%u h_stride=%u pass_through=%u zp=%d scale=%f\n",
               nativeNhwcIn.w_stride, nativeNhwcIn.h_stride,
               nativeNhwcIn.pass_through,
               nativeNhwcIn.zp, nativeNhwcIn.scale);
    } else {
        printf("[ObbDetector]   (查询失败, ret=%d)\n", ret);
    }

    // --- Native NC1HWC2 输出 (全部4个头) ---
    for (int i = 0; i < (int)m_ioNum.n_output && i < 4; i++) {
        rknn_tensor_attr nativeNc1hwc2Out;
        memset(&nativeNc1hwc2Out, 0, sizeof(nativeNc1hwc2Out));
        nativeNc1hwc2Out.index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_NATIVE_NC1HWC2_OUTPUT_ATTR,
                         &nativeNc1hwc2Out, sizeof(nativeNc1hwc2Out));
        printf("[ObbDetector] === Native NC1HWC2 输出[%d] ===\n", i);
        if (ret == 0) {
            printf("[ObbDetector]   fmt=%s type=%s qnt=%s n_dims=%u\n",
                   get_format_string(nativeNc1hwc2Out.fmt),
                   get_type_string(nativeNc1hwc2Out.type),
                   get_qnt_type_string(nativeNc1hwc2Out.qnt_type),
                   nativeNc1hwc2Out.n_dims);
            printf("[ObbDetector]   dims=[%u,%u,%u,%u] n_elems=%u size=%u size_with_stride=%u\n",
                   nativeNc1hwc2Out.dims[0], nativeNc1hwc2Out.dims[1],
                   nativeNc1hwc2Out.dims[2], nativeNc1hwc2Out.dims[3],
                   nativeNc1hwc2Out.n_elems, nativeNc1hwc2Out.size,
                   nativeNc1hwc2Out.size_with_stride);
            printf("[ObbDetector]   w_stride=%u h_stride=%u pass_through=%u zp=%d scale=%f\n",
                   nativeNc1hwc2Out.w_stride, nativeNc1hwc2Out.h_stride,
                   nativeNc1hwc2Out.pass_through,
                   nativeNc1hwc2Out.zp, nativeNc1hwc2Out.scale);

            // ---- 关键对比: 逻辑输出 vs Native 输出 ----
            printf("[ObbDetector]   [对比] 逻辑: fmt=%s type=%s dims=[%u,%u,%u,%u] zp=%d scale=%f size=%u\n",
                   get_format_string(m_outputAttrs[i].fmt),
                   get_type_string(m_outputAttrs[i].type),
                   m_outputAttrs[i].dims[0], m_outputAttrs[i].dims[1],
                   m_outputAttrs[i].dims[2], m_outputAttrs[i].dims[3],
                   m_outputAttrs[i].zp, m_outputAttrs[i].scale,
                   m_outputAttrs[i].size);
            printf("[ObbDetector]   [对比] Native: fmt=%s type=%s dims=[%u,%u,%u,%u] zp=%d scale=%f size=%u\n",
                   get_format_string(nativeNc1hwc2Out.fmt),
                   get_type_string(nativeNc1hwc2Out.type),
                   nativeNc1hwc2Out.dims[0], nativeNc1hwc2Out.dims[1],
                   nativeNc1hwc2Out.dims[2], nativeNc1hwc2Out.dims[3],
                   nativeNc1hwc2Out.zp, nativeNc1hwc2Out.scale,
                   nativeNc1hwc2Out.size);

            // 警告: 如果 zp 或 scale 不同，后处理会使用错误的量化参数
            if (m_outputAttrs[i].zp != nativeNc1hwc2Out.zp ||
                fabsf(m_outputAttrs[i].scale - nativeNc1hwc2Out.scale) > 1e-7f) {
                printf("[ObbDetector]   *** 警告: 逻辑和Native的量化参数不一致! ***\n");
                printf("[ObbDetector]   *** 当前后处理使用逻辑zp/scale, 可能引入精度偏差 ***\n");
            }
            if (m_outputAttrs[i].size != nativeNc1hwc2Out.size) {
                printf("[ObbDetector]   *** 警告: 逻辑size(%u) != Native size(%u), 存在隐式转换 ***\n",
                       m_outputAttrs[i].size, nativeNc1hwc2Out.size);
            }
        } else {
            printf("[ObbDetector]   (查询失败, ret=%d)\n", ret);
        }
    }

    // --- Native NHWC 输出 (全部4个头) ---
    for (int i = 0; i < (int)m_ioNum.n_output && i < 4; i++) {
        rknn_tensor_attr nativeNhwcOut;
        memset(&nativeNhwcOut, 0, sizeof(nativeNhwcOut));
        nativeNhwcOut.index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR,
                         &nativeNhwcOut, sizeof(nativeNhwcOut));
        printf("[ObbDetector] === Native NHWC 输出[%d] ===\n", i);
        if (ret == 0) {
            printf("[ObbDetector]   fmt=%s type=%s dims=[%u,%u,%u,%u] zp=%d scale=%f size=%u size_with_stride=%u\n",
                   get_format_string(nativeNhwcOut.fmt),
                   get_type_string(nativeNhwcOut.type),
                   nativeNhwcOut.dims[0], nativeNhwcOut.dims[1],
                   nativeNhwcOut.dims[2], nativeNhwcOut.dims[3],
                   nativeNhwcOut.zp, nativeNhwcOut.scale,
                   nativeNhwcOut.size, nativeNhwcOut.size_with_stride);
        } else {
            printf("[ObbDetector]   (查询失败, ret=%d)\n", ret);
        }
    }

    // --- 设备内存信息 ---
    {
        rknn_mem_size memInfo;
        memset(&memInfo, 0, sizeof(memInfo));
        ret = rknn_query(m_ctx, RKNN_QUERY_DEVICE_MEM_INFO, &memInfo, sizeof(memInfo));
        printf("[ObbDetector] === NPU 设备内存 ===\n");
        if (ret == 0) {
            printf("[ObbDetector]   total_weight=%u total_internal=%u dma_alloc=%llu sram_total=%u sram_free=%u\n",
                   memInfo.total_weight_size, memInfo.total_internal_size,
                   (unsigned long long)memInfo.total_dma_allocated_size,
                   memInfo.total_sram_size, memInfo.free_sram_size);
        } else {
            printf("[ObbDetector]   (查询失败, ret=%d)\n", ret);
        }
    }

    // --- 逻辑输入 vs Native 输入对比 ---
    printf("[ObbDetector] === 输入格式对比总结 ===\n");
    printf("[ObbDetector]   逻辑输入: fmt=%s type=%s dims=[%u,%u,%u,%u] size=%u\n",
           get_format_string(m_inputAttrs[0].fmt),
           get_type_string(m_inputAttrs[0].type),
           m_inputAttrs[0].dims[0], m_inputAttrs[0].dims[1],
           m_inputAttrs[0].dims[2], m_inputAttrs[0].dims[3],
           m_inputAttrs[0].size);
    printf("[ObbDetector]   Native NC1HWC2: fmt=%s type=%s dims=[%u,%u,%u,%u] size=%u w_stride=%u\n",
           get_format_string(nativeNc1hwc2In.fmt),
           get_type_string(nativeNc1hwc2In.type),
           nativeNc1hwc2In.dims[0], nativeNc1hwc2In.dims[1],
           nativeNc1hwc2In.dims[2], nativeNc1hwc2In.dims[3],
           nativeNc1hwc2In.size, nativeNc1hwc2In.w_stride);
    printf("[ObbDetector]   Native NHWC: fmt=%s type=%s dims=[%u,%u,%u,%u] size=%u w_stride=%u\n",
           get_format_string(nativeNhwcIn.fmt),
           get_type_string(nativeNhwcIn.type),
           nativeNhwcIn.dims[0], nativeNhwcIn.dims[1],
           nativeNhwcIn.dims[2], nativeNhwcIn.dims[3],
           nativeNhwcIn.size, nativeNhwcIn.w_stride);

    // 根据诊断结果给出建议
    // 注意: 不同 format 下 width 的维度索引不同:
    //   NHWC: dims[0]=N, dims[1]=H, dims[2]=W, dims[3]=C
    //   NCHW: dims[0]=N, dims[1]=C, dims[2]=H, dims[3]=W
    //   NC1HWC2: 5维, dims[3]=W
    {
        uint32_t nativeInW = 0;
        if (nativeNc1hwc2In.fmt == RKNN_TENSOR_NHWC) {
            nativeInW = nativeNc1hwc2In.dims[2];  // NHWC: dims[2]=W
        } else if (nativeNc1hwc2In.fmt == RKNN_TENSOR_NCHW ||
                   nativeNc1hwc2In.fmt == RKNN_TENSOR_NC1HWC2) {
            nativeInW = nativeNc1hwc2In.dims[3];  // NCHW/NC1HWC2: dims[3]=W
        }
        if (nativeNc1hwc2In.w_stride != 0 && nativeInW != 0 &&
            nativeNc1hwc2In.w_stride != nativeInW) {
            printf("[ObbDetector] *** 诊断: 输入宽度需要对齐 (w_stride=%u != width=%u), "
                   "NPU内部有padding, 可能引入边缘噪声 ***\n",
                   nativeNc1hwc2In.w_stride, nativeInW);
        } else {
            printf("[ObbDetector]    输入宽度对齐正常 (w_stride=%u == width=%u)\n",
                   nativeNc1hwc2In.w_stride, nativeInW);
        }
    }
    // 仅当逻辑格式与 NPU 真正原生格式(NC1HWC2)不同时告警
    if (m_inputAttrs[0].fmt == RKNN_TENSOR_NHWC &&
        nativeNc1hwc2In.fmt == RKNN_TENSOR_NHWC) {
        printf("[ObbDetector]    输入格式匹配 (均为NHWC), 无驱动层转换\n");
    } else if (m_inputAttrs[0].fmt != nativeNc1hwc2In.fmt) {
        printf("[ObbDetector] *** 诊断: 逻辑输入格式(%s) != NPU原生格式(%s), 每次推理需转换 ***\n",
               get_format_string(m_inputAttrs[0].fmt),
               get_format_string(nativeNc1hwc2In.fmt));
    }

#endif

    // 当前 OBB 后处理仅支持旧版 4 输出布局：
    // 3 个检测头 + 1 个角度头。新的单输出布局需要单独适配，
    // 否则 detect() 中会按旧布局解码并发生越界/空指针访问。
    if (m_ioNum.n_output != 4) {
        fprintf(stderr,
                "[ObbDetector] 不支持的模型输出布局: n_output=%u，当前代码仅支持 4 输出 OBB 模型。\n",
                m_ioNum.n_output);
        fprintf(stderr,
                "[ObbDetector] 当前模型与现有后处理不兼容，请改用旧版 4 输出 RKNN，或适配新的单输出后处理。\n");
        goto fail;
    }

    // 当前解码器要求前三个检测头采用 NCHW INT8，通道数必须为 64(DFL)+类别数。
    // 如果新模型导出布局不同，继续按旧布局读取会把错误通道当成类别分数，典型表现就是长期 50%。
    {
        const int expectedHeadChannels = 64 + m_params.numClasses;
        for (int i = 0; i < 3; ++i) {
            if (m_outputAttrs[i].type != RKNN_TENSOR_INT8 ||
                static_cast<int>(m_outputAttrs[i].dims[1]) != expectedHeadChannels) {
                fprintf(stderr,
                        "[ObbDetector] 输出头[%d]不兼容: type=%d dims1=%u, 期望 INT8/NCHW channels=%d\n",
                        i, m_outputAttrs[i].type, m_outputAttrs[i].dims[1],
                        expectedHeadChannels);
                goto fail;
            }
            if (m_outputAttrs[i].fmt != RKNN_TENSOR_NCHW) {
                fprintf(stderr,
                        "[ObbDetector] 输出头[%d]格式不是 NCHW: fmt=%d，当前解码结果可能错误\n",
                        i, m_outputAttrs[i].fmt);
                goto fail;
            }
        }
        if (m_outputAttrs[3].type != RKNN_TENSOR_INT8) {
            fprintf(stderr, "[ObbDetector] 角度输出不是 INT8，当前解码器不支持: type=%d\n",
                    m_outputAttrs[3].type);
            goto fail;
        }
    }

    // 4. 解析输入维度
    {
        rknn_tensor_attr &in = m_inputAttrs[0];
        // 假设 NHWC: dims[0]=1, dims[1]=H, dims[2]=W, dims[3]=C
        // 或 NCHW: dims[0]=1, dims[1]=C, dims[2]=H, dims[3]=W
        if (in.fmt == RKNN_TENSOR_NHWC) {
            m_params.modelWidth  = in.dims[2];
            m_params.modelHeight = in.dims[1];
        } else {
            m_params.modelWidth  = in.dims[3];
            m_params.modelHeight = in.dims[2];
        }
        // 类别数已经由标签文件确定。输入张量最后/第二维是 RGB 通道数 3，
        // 不能覆盖 numClasses，否则第 4 类 doubleline_yellow 永远不会被解码。
    }

    // 5. 判断量化类型
    m_isQuant = (m_outputAttrs[0].type != RKNN_TENSOR_FLOAT32 &&
                 m_outputAttrs[0].type != RKNN_TENSOR_FLOAT16);

    // 6. 分配 letterbox 缓冲区
    m_bufSize = m_params.modelWidth * m_params.modelHeight * 3;
    m_letterboxBuf = new uint8_t[m_bufSize];

    m_loaded = true;
    return true;

fail:
    rknn_destroy(m_ctx);
    m_ctx = 0;
    delete[] m_inputAttrs;  m_inputAttrs  = nullptr;
    delete[] m_outputAttrs; m_outputAttrs = nullptr;
    delete[] m_letterboxBuf; m_letterboxBuf = nullptr;
    m_bufSize = 0;
    return false;
}

void ObbDetector::release() {
    if (m_ctx) { rknn_destroy(m_ctx); m_ctx = 0; }
    delete[] m_inputAttrs;   m_inputAttrs  = nullptr;
    delete[] m_outputAttrs;  m_outputAttrs = nullptr;
    delete[] m_letterboxBuf; m_letterboxBuf = nullptr;
    m_labels.clear();
    m_loaded = false;
}

const char *ObbDetector::className(int clsId) const {
    if (clsId >= 0 && clsId < (int)m_labels.size())
        return m_labels[clsId].c_str();
    return "?";
}

// ---- Letterbox 预处理 (双线性插值, 参照Rockchip crop_and_scale_image_c) ----
void ObbDetector::letterbox(const uint8_t *src, int srcW, int srcH,
                              uint8_t *dst, float &scale, int &xPad, int &yPad) {
    int dstW = m_params.modelWidth;
    int dstH = m_params.modelHeight;

    // 计算缩放系数 (保持宽高比)
    scale = std::min((float)dstW / (float)srcW, (float)dstH / (float)srcH);
    int newW = (int)(srcW * scale);
    int newH = (int)(srcH * scale);

    // 居中偏移
    xPad = (dstW - newW) / 2;
    yPad = (dstH - newH) / 2;

    // 填充背景色 (114,114,114)
    memset(dst, 114, dstW * dstH * 3);

    // 双线性插值缩放: 1280×720 → ~416×416 是 ~3× 降采样,
    // 最近邻会丢弃 2/3 的像素信息, 双线性保留所有源像素的加权贡献,
    // 显著提升小目标和边缘检测精度
    const float xRatio = (float)srcW / (float)newW;  // >1.0 降采样
    const float yRatio = (float)srcH / (float)newH;
    const int srcStride = srcW * 3;

    for (int dy = 0; dy < newH; dy++) {
        const float srcYf = dy * yRatio;
        const int   srcYi = (int)srcYf;
        const float yDiff = srcYf - (float)srcYi;
        const int   srcYc = (srcYi >= srcH - 1) ? srcYi : srcYi + 1;  // clamp

        uint8_t *drow = dst + ((dy + yPad) * dstW + xPad) * 3;
        const uint8_t *srow0 = src + srcYi * srcStride;
        const uint8_t *srow1 = src + srcYc * srcStride;

        for (int dx = 0; dx < newW; dx++) {
            const float srcXf = dx * xRatio;
            const int   srcXi = (int)srcXf;
            const float xDiff = srcXf - (float)srcXi;
            const int   srcXc = (srcXi >= srcW - 1) ? srcXi : srcXi + 1;

            // 4邻域像素索引
            const int i00 = srcXi * 3;        // (x0, y0)
            const int i10 = srcXc * 3;        // (x1, y0)
            const int i01 = i00;              // (x0, y1) 同列
            const int i11 = i10;              // (x1, y1) 同列

            const float w00 = (1.0f - xDiff) * (1.0f - yDiff);
            const float w10 = xDiff * (1.0f - yDiff);
            const float w01 = (1.0f - xDiff) * yDiff;
            const float w11 = xDiff * yDiff;

            for (int c = 0; c < 3; c++) {
                float val = srow0[i00 + c] * w00 +
                            srow0[i10 + c] * w10 +
                            srow1[i01 + c] * w01 +
                            srow1[i11 + c] * w11;
                drow[dx * 3 + c] = (uint8_t)(val + 0.5f);
            }
        }
    }
}

// ---- 执行检测 ----
bool ObbDetector::detect(const uint8_t *srcRGB, int width, int height,
                          DetectResult &result) {
    if (!m_loaded) return false;
    result.boxes.clear();

    // 1. Letterbox
    letterbox(srcRGB, width, height, m_letterboxBuf,
              result.letterboxScale, result.letterboxXPad, result.letterboxYPad);
    result.srcWidth  = width;
    result.srcHeight = height;
    // 2. 设置输入
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index        = 0;
    inputs[0].type         = RKNN_TENSOR_UINT8;
    inputs[0].fmt          = RKNN_TENSOR_NHWC;
    inputs[0].size         = m_bufSize;
    inputs[0].buf          = m_letterboxBuf;
    inputs[0].pass_through = 0;

    int ret = rknn_inputs_set(m_ctx, m_ioNum.n_input, inputs);
    if (ret < 0) { fprintf(stderr, "[ObbDetector] inputs_set err: %d\n", ret); return false; }
    // 3. 推理
    ret = rknn_run(m_ctx, nullptr);
    if (ret < 0) { fprintf(stderr, "[ObbDetector] run err: %d\n", ret); return false; }
    // 4. 获取输出：统一让 RKNN Runtime 解量化为 FLOAT32，避免手工 zp/scale 解释错误。
    rknn_output outputs[4];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < (int)m_ioNum.n_output; i++) {
        outputs[i].index = i;
        // Match rknn_model_zoo/yolov8_obb: quantized outputs stay INT8.
        outputs[i].want_float = !m_isQuant;
    }
    ret = rknn_outputs_get(m_ctx, m_ioNum.n_output, outputs, nullptr);
    if (ret < 0) { fprintf(stderr, "[ObbDetector] outputs_get err: %d\n", ret); return false; }
    // 5. 后处理 (Runtime 解量化后的 FLOAT32 路径)
    int dimsArr[4][4] = {{0}};
    const int  *dimsPtrs[4]  = {nullptr};
    for (int i = 0; i < (int)m_ioNum.n_output && i < 4; i++) {
        memcpy(dimsArr[i], m_outputAttrs[i].dims, 4 * sizeof(int));
        dimsPtrs[i] = dimsArr[i];
    }

    if (m_isQuant) {
        const int8_t *featPtrs[4] = {nullptr};
        int32_t zeroPoints[4] = {0};
        float scales[4] = {0.0f};
        for (int i = 0; i < (int)m_ioNum.n_output && i < 4; ++i) {
            featPtrs[i] = static_cast<const int8_t *>(outputs[i].buf);
            zeroPoints[i] = m_outputAttrs[i].zp;
            scales[i] = m_outputAttrs[i].scale;
        }
        ObbPostProcess::decodeInt8(featPtrs, dimsPtrs,
                                   m_params.numClasses, zeroPoints, scales,
                                   m_params.boxThreshold, m_params.nmsThreshold,
                                   m_params.modelWidth, m_params.modelHeight,
                                   result.letterboxScale, result.letterboxXPad, result.letterboxYPad,
                                   width, height, result.boxes);
    } else {
        const float *featPtrs[4] = {nullptr};
        for (int i = 0; i < (int)m_ioNum.n_output && i < 4; ++i)
            featPtrs[i] = static_cast<const float *>(outputs[i].buf);
        ObbPostProcess::decodeFloat32(featPtrs, dimsPtrs,
                                      m_params.numClasses,
                                      m_params.boxThreshold, m_params.nmsThreshold,
                                      m_params.modelWidth, m_params.modelHeight,
                                      result.letterboxScale, result.letterboxXPad, result.letterboxYPad,
                                      width, height, result.boxes);
    }
    // 6. 释放输出
    rknn_outputs_release(m_ctx, m_ioNum.n_output, outputs);
    return true;
}
