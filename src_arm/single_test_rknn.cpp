// single_test_rknn.cpp
// Board-side single-image test for the MIMO+ stage5 FP16 RKNN model.
// Pipeline matches ARM_final1/plate_preprocess_rknn.cpp exactly:
//   input image -> resize 94x24 (INTER_LINEAR) -> resize 188x48 (INTER_LINEAR)
//   -> BGR2RGB -> uint8 RGB NHWC -> RKNN -> want_float output
//   -> clamp [0,1] -> RGB2BGR -> area-resize 94x24 -> save
// Build (on the RK3568 board):
//   g++ -O2 -o single_test_rknn single_test_rknn.cpp -I/usr/include/opencv4 //       -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lrknnrt
// Run:
//   ./single_test_rknn /userdata/model/mimo_plus_stage5_188x48_fp16_rk3568.rknn in.jpg outdir

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rknn_api.h"

static const int INPUT_W = 188;
static const int INPUT_H = 48;
static const int LOGICAL_W = 94;
static const int LOGICAL_H = 24;

static double now_ms()
{
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main(int argc, char **argv)
{
    const char *modelPath = argc > 1 ? argv[1] : "/userdata/model/mimo_plus_stage5_188x48_fp16_rk3568.rknn";
    const char *inputPath = argc > 2 ? argv[2] : "in.jpg";
    const char *outDir = argc > 3 ? argv[3] : ".";

    rknn_context ctx = 0;
    int ret = rknn_init(&ctx, const_cast<char *>(modelPath), 0, 0, nullptr);
    if (ret != RKNN_SUCC) {
        fprintf(stderr, "rknn_init failed for %s ret=%d\n", modelPath, ret);
        return 1;
    }

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC || io_num.n_input != 1 || io_num.n_output != 1) {
        fprintf(stderr, "query in/out failed ret=%d\n", ret);
        rknn_destroy(ctx);
        return 1;
    }
    rknn_tensor_attr in_attr, out_attr;
    memset(&in_attr, 0, sizeof(in_attr));
    memset(&out_attr, 0, sizeof(out_attr));
    in_attr.index = 0;
    out_attr.index = 0;
    if (rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr)) != RKNN_SUCC ||
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr)) != RKNN_SUCC) {
        fprintf(stderr, "query attrs failed\n");
        rknn_destroy(ctx);
        return 1;
    }
    const size_t expected = (size_t)INPUT_W * INPUT_H * 3;
    if (in_attr.n_elems != expected || out_attr.n_elems != expected) {
        fprintf(stderr, "shape mismatch: in=%u out=%u expected=%zu\n",
                in_attr.n_elems, out_attr.n_elems, expected);
        rknn_destroy(ctx);
        return 1;
    }
    printf("[RKNN] model loaded: %s\n", modelPath);
    printf("[RKNN] input fmt=%d type=%d dims=[%u,%u,%u,%u]\n",
           in_attr.fmt, in_attr.type, in_attr.dims[0], in_attr.dims[1],
           in_attr.dims[2], in_attr.dims[3]);

    cv::Mat bgr = cv::imread(inputPath, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        fprintf(stderr, "failed to read image: %s\n", inputPath);
        rknn_destroy(ctx);
        return 1;
    }
    printf("[RKNN] input image %dx%d\n", bgr.cols, bgr.rows);

    // --- preprocessing (matches plate_preprocess_rknn.cpp after the 94x24 fix) ---
    cv::Mat logical;
    cv::resize(bgr, logical, cv::Size(LOGICAL_W, LOGICAL_H), 0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat network;
    cv::resize(logical, network, cv::Size(INPUT_W, INPUT_H), 0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat rgb;
    cv::cvtColor(network, rgb, cv::COLOR_BGR2RGB);
    if (!rgb.isContinuous()) rgb = rgb.clone();

    // --- inference ---
    rknn_input input;
    memset(&input, 0, sizeof(input));
    input.index = 0;
    input.buf = rgb.data;
    input.size = (uint32_t)(rgb.total() * rgb.elemSize());
    input.pass_through = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;

    rknn_output output;
    memset(&output, 0, sizeof(output));
    output.index = 0;
    output.want_float = 1;
    output.is_prealloc = 0;

    const double t0 = now_ms();
    ret = rknn_inputs_set(ctx, 1, &input);
    if (ret != RKNN_SUCC) { fprintf(stderr, "rknn_inputs_set failed ret=%d\n", ret); rknn_destroy(ctx); return 1; }
    ret = rknn_run(ctx, nullptr);
    if (ret != RKNN_SUCC) { fprintf(stderr, "rknn_run failed ret=%d\n", ret); rknn_destroy(ctx); return 1; }
    ret = rknn_outputs_get(ctx, 1, &output, nullptr);
    if (ret != RKNN_SUCC) { fprintf(stderr, "rknn_outputs_get failed ret=%d\n", ret); rknn_destroy(ctx); return 1; }
    const double t1 = now_ms();
    printf("[RKNN] inference %.2f ms\n", t1 - t0);

    // --- postprocessing (matches float_output_to_bgr) ---
    const float *data = static_cast<const float *>(output.buf);
    const bool is_nhwc = out_attr.fmt == RKNN_TENSOR_NHWC;
    cv::Mat out_rgb(INPUT_H, INPUT_W, CV_8UC3);
    for (int y = 0; y < INPUT_H; ++y) {
        cv::Vec3b *row = out_rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < INPUT_W; ++x) {
            const size_t pixel = (size_t)y * INPUT_W + x;
            for (int c = 0; c < 3; ++c) {
                const size_t idx = is_nhwc ? pixel * 3 + c : (size_t)c * INPUT_W * INPUT_H + pixel;
                float v = data[idx];
                if (v < 0.0f) v = 0.0f;
                if (v > 1.0f) v = 1.0f;
                row[x][c] = (unsigned char)(v * 255.0f + 0.5f);
            }
        }
    }
    cv::Mat out_bgr;
    cv::cvtColor(out_rgb, out_bgr, cv::COLOR_RGB2BGR);
    cv::Mat restored;
    cv::resize(out_bgr, restored, cv::Size(LOGICAL_W, LOGICAL_H), 0.0, 0.0, cv::INTER_AREA);

    std::string inOut = std::string(outDir) + "/rknn_input_94x24.png";
    std::string resOut = std::string(outDir) + "/rknn_restored_94x24.png";
    cv::imwrite(inOut, logical);
    cv::imwrite(resOut, restored);
    printf("[RKNN] saved %s and %s\n", inOut.c_str(), resOut.c_str());
    rknn_outputs_release(ctx, 1, &output);
    rknn_destroy(ctx);
    printf("[RKNN] DONE\n");
    return 0;
}
