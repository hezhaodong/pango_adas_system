#include "helmet_detector.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <vector>

namespace {
constexpr int kClassCount = 3;
constexpr float kDefaultThreshold = 0.25f;
constexpr float kNoHelmetThreshold = 0.40f;
constexpr float kNmsThreshold = 0.45f;

float iou(const YoloBox &a, const YoloBox &b)
{
    const float left = std::max(a.left, b.left);
    const float top = std::max(a.top, b.top);
    const float right = std::min(a.right, b.right);
    const float bottom = std::min(a.bottom, b.bottom);
    const float intersection = std::max(0.0f, right - left) *
                               std::max(0.0f, bottom - top);
    const float areaA = std::max(0, a.right - a.left) *
                        std::max(0, a.bottom - a.top);
    const float areaB = std::max(0, b.right - b.left) *
                        std::max(0, b.bottom - b.top);
    const float total = areaA + areaB - intersection;
    return total > 0.0f ? intersection / total : 0.0f;
}
}

HelmetDetector::HelmetDetector()
    : m_ctx(0), m_inputAttrs(nullptr), m_outputAttrs(nullptr),
      m_letterboxBuf(nullptr), m_loaded(false),
      m_modelWidth(640), m_modelHeight(640)
{
    std::memset(&m_ioNum, 0, sizeof(m_ioNum));
}

HelmetDetector::~HelmetDetector() { release(); }

bool HelmetDetector::loadModel(const char *modelPath)
{
    release();
    int ret = rknn_init(&m_ctx, const_cast<char *>(modelPath), 0, 0, nullptr);
    if (ret != RKNN_SUCC) {
        std::fprintf(stderr, "[Helmet] rknn_init failed: %d\n", ret);
        return false;
    }
    ret = rknn_query(m_ctx, RKNN_QUERY_IN_OUT_NUM, &m_ioNum, sizeof(m_ioNum));
    if (ret != RKNN_SUCC || m_ioNum.n_input != 1 || m_ioNum.n_output != 1) {
        std::fprintf(stderr, "[Helmet] expected 1 input/1 output, got %u/%u\n",
                     m_ioNum.n_input, m_ioNum.n_output);
        release();
        return false;
    }

    m_inputAttrs = static_cast<rknn_tensor_attr *>(
        std::calloc(m_ioNum.n_input, sizeof(rknn_tensor_attr)));
    m_outputAttrs = static_cast<rknn_tensor_attr *>(
        std::calloc(m_ioNum.n_output, sizeof(rknn_tensor_attr)));
    m_inputAttrs[0].index = 0;
    m_outputAttrs[0].index = 0;
    if (rknn_query(m_ctx, RKNN_QUERY_INPUT_ATTR, &m_inputAttrs[0],
                   sizeof(rknn_tensor_attr)) != RKNN_SUCC ||
        rknn_query(m_ctx, RKNN_QUERY_OUTPUT_ATTR, &m_outputAttrs[0],
                   sizeof(rknn_tensor_attr)) != RKNN_SUCC) {
        release();
        return false;
    }

    if (m_inputAttrs[0].fmt == RKNN_TENSOR_NCHW) {
        m_modelHeight = m_inputAttrs[0].dims[2];
        m_modelWidth = m_inputAttrs[0].dims[3];
    } else {
        m_modelHeight = m_inputAttrs[0].dims[1];
        m_modelWidth = m_inputAttrs[0].dims[2];
    }
    m_letterboxBuf = new uint8_t[m_modelWidth * m_modelHeight * 3];
    m_loaded = true;
    return true;
}

void HelmetDetector::release()
{
    if (m_ctx) rknn_destroy(m_ctx);
    m_ctx = 0;
    std::free(m_inputAttrs); m_inputAttrs = nullptr;
    std::free(m_outputAttrs); m_outputAttrs = nullptr;
    delete[] m_letterboxBuf; m_letterboxBuf = nullptr;
    m_loaded = false;
}

const char *HelmetDetector::className(int classId) const
{
    static const char *const names[] = {"motorcycle", "helmet", "no-helmet"};
    return classId >= 0 && classId < kClassCount ? names[classId] : "?";
}

void HelmetDetector::letterbox(const uint8_t *src, int srcW, int srcH,
                               uint8_t *dst, float &scale, int &xPad, int &yPad)
{
    scale = std::min(static_cast<float>(m_modelWidth) / srcW,
                     static_cast<float>(m_modelHeight) / srcH);
    const int newW = static_cast<int>(srcW * scale);
    const int newH = static_cast<int>(srcH * scale);
    xPad = (m_modelWidth - newW) / 2;
    yPad = (m_modelHeight - newH) / 2;
    std::memset(dst, 114, m_modelWidth * m_modelHeight * 3);
    for (int y = 0; y < newH; ++y) {
        const int sy = std::min(srcH - 1, static_cast<int>(y / scale));
        for (int x = 0; x < newW; ++x) {
            const int sx = std::min(srcW - 1, static_cast<int>(x / scale));
            std::memcpy(dst + ((y + yPad) * m_modelWidth + x + xPad) * 3,
                        src + (sy * srcW + sx) * 3, 3);
        }
    }
}

bool HelmetDetector::detect(const uint8_t *srcRgb, int width, int height,
                            YoloDetectResult &result)
{
    result.boxes.clear();
    result.yieldViolationAlert = false;
    if (!m_loaded) return false;

    float scale = 1.0f;
    int xPad = 0, yPad = 0;
    letterbox(srcRgb, width, height, m_letterboxBuf, scale, xPad, yPad);

    rknn_input input{};
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.size = m_modelWidth * m_modelHeight * 3;
    input.buf = m_letterboxBuf;
    if (rknn_inputs_set(m_ctx, 1, &input) != RKNN_SUCC ||
        rknn_run(m_ctx, nullptr) != RKNN_SUCC)
        return false;

    rknn_output output{};
    output.index = 0;
    output.want_float = 1;
    if (rknn_outputs_get(m_ctx, 1, &output, nullptr) != RKNN_SUCC)
        return false;

    const float *data = static_cast<const float *>(output.buf);
    const int elements = static_cast<int>(m_outputAttrs[0].n_elems);
    const int anchors = elements / (4 + kClassCount);
    std::vector<YoloBox> candidates;
    if (data && anchors > 0 && anchors * (4 + kClassCount) == elements) {
        for (int i = 0; i < anchors; ++i) {
            int classId = 0;
            float confidence = data[4 * anchors + i];
            for (int c = 1; c < kClassCount; ++c) {
                const float score = data[(4 + c) * anchors + i];
                if (score > confidence) {
                    confidence = score;
                    classId = c;
                }
            }
            const float threshold = classId == 2 ? kNoHelmetThreshold
                                                  : kDefaultThreshold;
            if (confidence < threshold) continue;

            const float cx = data[i];
            const float cy = data[anchors + i];
            const float bw = data[2 * anchors + i];
            const float bh = data[3 * anchors + i];
            YoloBox box;
            box.left = std::clamp(static_cast<int>((cx - bw * 0.5f - xPad) / scale), 0, width - 1);
            box.top = std::clamp(static_cast<int>((cy - bh * 0.5f - yPad) / scale), 0, height - 1);
            box.right = std::clamp(static_cast<int>((cx + bw * 0.5f - xPad) / scale), 0, width - 1);
            box.bottom = std::clamp(static_cast<int>((cy + bh * 0.5f - yPad) / scale), 0, height - 1);
            box.confidence = confidence;
            box.classId = classId;
            if (box.right > box.left && box.bottom > box.top)
                candidates.push_back(box);
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const YoloBox &a, const YoloBox &b) {
                  return a.confidence > b.confidence;
              });
    for (const YoloBox &candidate : candidates) {
        bool suppressed = false;
        for (const YoloBox &kept : result.boxes) {
            if (candidate.classId == kept.classId && iou(candidate, kept) > kNmsThreshold) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed && result.boxes.size() < 128)
            result.boxes.push_back(candidate);
    }
    rknn_outputs_release(m_ctx, 1, &output);
    return true;
}
