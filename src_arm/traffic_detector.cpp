#include "traffic_detector.h"




#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>

namespace {

constexpr int kClassCount = 3;
constexpr int kValuesPerAnchor = 5 + kClassCount;
constexpr int kAnchorsPerHead = 3;

const char *const kClassNames[kClassCount] = {
    "Zebra_Cross", "R_Signal", "G_Signal"
};

const int kStrides[4] = {8, 16, 32, 64};
const float kAnchors[4][kAnchorsPerHead][2] = {
    {{19.f, 27.f}, {44.f, 40.f}, {38.f, 94.f}},
    {{96.f, 68.f}, {86.f, 152.f}, {180.f, 137.f}},
    {{140.f, 301.f}, {303.f, 264.f}, {238.f, 542.f}},
    {{436.f, 615.f}, {739.f, 380.f}, {925.f, 792.f}}
};

float sigmoid(float value)
{
    if (value >= 0.f) {
        const float z = std::exp(-value);
        return 1.f / (1.f + z);
    }
    const float z = std::exp(value);
    return z / (1.f + z);
}

float intersectionOverUnion(const YoloBox &a, const YoloBox &b)
{
    const int left = std::max(a.left, b.left);
    const int top = std::max(a.top, b.top);
    const int right = std::min(a.right, b.right);
    const int bottom = std::min(a.bottom, b.bottom);
    const float iw = static_cast<float>(std::max(0, right - left));
    const float ih = static_cast<float>(std::max(0, bottom - top));
    const float intersection = iw * ih;
    const float areaA = static_cast<float>(std::max(0, a.right - a.left) *
                                           std::max(0, a.bottom - a.top));
    const float areaB = static_cast<float>(std::max(0, b.right - b.left) *
                                           std::max(0, b.bottom - b.top));
    const float unionArea = areaA + areaB - intersection;
    return unionArea > 0.f ? intersection / unionArea : 0.f;
}

int headIndexFromGrid(int gridWidth, int inputWidth)
{
    for (int i = 0; i < 4; ++i) {
        if (gridWidth * kStrides[i] == inputWidth) return i;
    }
    return -1;
}

bool outputShape(const rknn_tensor_attr &attr, int &channels,
                 int &gridHeight, int &gridWidth, bool &nhwc)
{
    if (attr.n_dims != 4) return false;
    nhwc = attr.fmt == RKNN_TENSOR_NHWC;
    if (nhwc) {
        gridHeight = static_cast<int>(attr.dims[1]);
        gridWidth = static_cast<int>(attr.dims[2]);
        channels = static_cast<int>(attr.dims[3]);
    } else {
        channels = static_cast<int>(attr.dims[1]);
        gridHeight = static_cast<int>(attr.dims[2]);
        gridWidth = static_cast<int>(attr.dims[3]);
    }
    return channels == kAnchorsPerHead * kValuesPerAnchor &&
           gridHeight > 0 && gridWidth > 0;
}

float tensorValue(const float *data, bool nhwc, int channels,
                  int gridHeight, int gridWidth, int channel, int y, int x)
{
    if (nhwc) {
        const size_t index =
            (static_cast<size_t>(y) * gridWidth + x) * channels + channel;
        return data[index];
    }
    const size_t index =
        (static_cast<size_t>(channel) * gridHeight + y) * gridWidth + x;
    return data[index];
}

} // namespace

TrafficDetector::TrafficDetector()
    : m_ctx(0), m_inputWidth(0), m_inputHeight(0),
      m_confThreshold(0.25f), m_nmsThreshold(0.45f)
{
    std::memset(&m_ioNum, 0, sizeof(m_ioNum));
    std::memset(&m_inputAttr, 0, sizeof(m_inputAttr));
}

TrafficDetector::~TrafficDetector()
{
    release();
}

bool TrafficDetector::loadModel(const char *modelPath)
{
    release();
    std::ifstream file(modelPath, std::ios::binary | std::ios::ate);
    if (!file) {
        std::fprintf(stderr, "TrafficDetector: cannot open %s\n", modelPath);
        return false;
    }
    const std::streamsize length = file.tellg();
    if (length <= 0) return false;
    file.seekg(0, std::ios::beg);
    m_modelData.resize(static_cast<size_t>(length));
    if (!file.read(reinterpret_cast<char *>(m_modelData.data()), length)) {
        release();
        return false;
    }

    int ret = rknn_init(&m_ctx, m_modelData.data(), m_modelData.size(), 0,
                        nullptr);
    if (ret != RKNN_SUCC) {
        std::fprintf(stderr, "TrafficDetector: rknn_init failed: %d\n", ret);
        release();
        return false;
    }
    ret = rknn_query(m_ctx, RKNN_QUERY_IN_OUT_NUM, &m_ioNum, sizeof(m_ioNum));
    if (ret != RKNN_SUCC || m_ioNum.n_input != 1 || m_ioNum.n_output != 4) {
        std::fprintf(stderr,
                     "TrafficDetector: expected 1 input/4 outputs, got %u/%u\n",
                     m_ioNum.n_input, m_ioNum.n_output);
        release();
        return false;
    }

    std::memset(&m_inputAttr, 0, sizeof(m_inputAttr));
    m_inputAttr.index = 0;
    ret = rknn_query(m_ctx, RKNN_QUERY_INPUT_ATTR, &m_inputAttr,
                     sizeof(m_inputAttr));
    if (ret != RKNN_SUCC || m_inputAttr.n_dims != 4) {
        release();
        return false;
    }
    if (m_inputAttr.fmt == RKNN_TENSOR_NCHW) {
        m_inputHeight = static_cast<int>(m_inputAttr.dims[2]);
        m_inputWidth = static_cast<int>(m_inputAttr.dims[3]);
    } else {
        m_inputHeight = static_cast<int>(m_inputAttr.dims[1]);
        m_inputWidth = static_cast<int>(m_inputAttr.dims[2]);
    }
    if (m_inputWidth != 640 || m_inputHeight != 640 ||
        m_inputAttr.type != RKNN_TENSOR_INT8) {
        std::fprintf(stderr, "TrafficDetector: unexpected input %dx%d\n",
                     m_inputWidth, m_inputHeight);
        release();
        return false;
    }

    m_outputAttrs.resize(m_ioNum.n_output);
    bool foundHead[4] = {false, false, false, false};
    for (uint32_t i = 0; i < m_ioNum.n_output; ++i) {
        std::memset(&m_outputAttrs[i], 0, sizeof(rknn_tensor_attr));
        m_outputAttrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_OUTPUT_ATTR, &m_outputAttrs[i],
                         sizeof(rknn_tensor_attr));
        int channels = 0, gh = 0, gw = 0;
        bool nhwc = false;
        const int head = outputShape(m_outputAttrs[i], channels, gh, gw, nhwc)
                             ? headIndexFromGrid(gw, m_inputWidth)
                             : -1;
        if (ret != RKNN_SUCC || head < 0 || gh != gw || foundHead[head] ||
            m_outputAttrs[i].type != RKNN_TENSOR_INT8 ||
            m_outputAttrs[i].qnt_type !=
                RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC) {
            std::fprintf(stderr,
                         "TrafficDetector: unsupported output tensor %u\n", i);
            release();
            return false;
        }
        foundHead[head] = true;
    }
    for (bool found : foundHead) {
        if (!found) {
            release();
            return false;
        }
    }
    m_letterbox.resize(static_cast<size_t>(m_inputWidth) * m_inputHeight * 3);
    return true;
}

void TrafficDetector::release()
{
    if (m_ctx != 0) rknn_destroy(m_ctx);
    m_ctx = 0;
    std::memset(&m_ioNum, 0, sizeof(m_ioNum));
    std::memset(&m_inputAttr, 0, sizeof(m_inputAttr));
    m_outputAttrs.clear();
    m_modelData.clear();
    m_letterbox.clear();
    m_inputWidth = 0;
    m_inputHeight = 0;
}

const char *TrafficDetector::className(int id) const
{
    return id >= 0 && id < kClassCount ? kClassNames[id] : "?";
}

bool TrafficDetector::detect(const uint8_t *rgb, int width, int height,
                             YoloDetectResult &result)
{
    result = YoloDetectResult();
    if (!isLoaded() || !rgb || width <= 0 || height <= 0) return false;

    const float scale = std::min(static_cast<float>(m_inputWidth) / width,
                                 static_cast<float>(m_inputHeight) / height);
    const int resizedWidth = std::max(1, static_cast<int>(std::round(width * scale)));
    const int resizedHeight = std::max(1, static_cast<int>(std::round(height * scale)));
    const int padX = (m_inputWidth - resizedWidth) / 2;
    const int padY = (m_inputHeight - resizedHeight) / 2;

    std::memset(m_letterbox.data(), 114, m_letterbox.size());
    for (int y = 0; y < resizedHeight; ++y) {
        const int sourceY = std::min(height - 1,
                                     static_cast<int>(y / scale));
        for (int x = 0; x < resizedWidth; ++x) {
            const int sourceX = std::min(width - 1,
                                         static_cast<int>(x / scale));
            std::memcpy(m_letterbox.data() +
                            (static_cast<size_t>(y + padY) * m_inputWidth +
                             x + padX) * 3,
                        rgb + (static_cast<size_t>(sourceY) * width + sourceX) * 3,
                        3);
        }
    }

    rknn_input input;
    std::memset(&input, 0, sizeof(input));
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.size = static_cast<uint32_t>(m_letterbox.size());
    input.buf = m_letterbox.data();
    input.pass_through = 0;
    int ret = rknn_inputs_set(m_ctx, 1, &input);
    if (ret != RKNN_SUCC || rknn_run(m_ctx, nullptr) != RKNN_SUCC) return false;

    std::vector<rknn_output> outputs(m_ioNum.n_output);
    std::memset(outputs.data(), 0, outputs.size() * sizeof(rknn_output));
    for (uint32_t i = 0; i < m_ioNum.n_output; ++i) {
        outputs[i].index = i;
        outputs[i].want_float = 1;
        outputs[i].is_prealloc = 0;
    }
    ret = rknn_outputs_get(m_ctx, m_ioNum.n_output, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) return false;

    std::vector<YoloBox> candidates;
    candidates.reserve(256);
    const float objectLogitThreshold =
        std::log(m_confThreshold / (1.f - m_confThreshold));
    for (uint32_t outputIndex = 0; outputIndex < m_ioNum.n_output;
         ++outputIndex) {
        const rknn_tensor_attr &attr = m_outputAttrs[outputIndex];
        int channels = 0, gridHeight = 0, gridWidth = 0;
        bool nhwc = false;
        if (!outputShape(attr, channels, gridHeight, gridWidth, nhwc) ||
            !outputs[outputIndex].buf) {
            continue;
        }
        const int head = headIndexFromGrid(gridWidth, m_inputWidth);
        if (head < 0) continue;
        const float *data = static_cast<const float *>(outputs[outputIndex].buf);
        for (int anchor = 0; anchor < kAnchorsPerHead; ++anchor) {
            const int base = anchor * kValuesPerAnchor;
            for (int y = 0; y < gridHeight; ++y) {
                for (int x = 0; x < gridWidth; ++x) {
                    const float objectLogit = tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base + 4, y, x);
                    if (objectLogit < objectLogitThreshold) continue;
                    const float objectness = sigmoid(objectLogit);
                    int classId = 0;
                    float classProbability = -std::numeric_limits<float>::infinity();
                    for (int cls = 0; cls < kClassCount; ++cls) {
                        const float probability = sigmoid(tensorValue(
                            data, nhwc, channels, gridHeight, gridWidth,
                            base + 5 + cls, y, x));
                        if (probability > classProbability) {
                            classProbability = probability;
                            classId = cls;
                        }
                    }
                    const float confidence = objectness * classProbability;
                    if (confidence < m_confThreshold) continue;

                    const float cx = (sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base, y, x)) * 2.f - 0.5f + x) * kStrides[head];
                    const float cy = (sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base + 1, y, x)) * 2.f - 0.5f + y) * kStrides[head];
                    const float widthScale = sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base + 2, y, x)) * 2.f;
                    const float heightScale = sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base + 3, y, x)) * 2.f;
                    const float boxWidth = widthScale * widthScale *
                                           kAnchors[head][anchor][0];
                    const float boxHeight = heightScale * heightScale *
                                            kAnchors[head][anchor][1];
                    YoloBox box;
                    box.left = std::max(0, std::min(width - 1,
                        static_cast<int>(std::round((cx - boxWidth * 0.5f - padX) / scale))));
                    box.top = std::max(0, std::min(height - 1,
                        static_cast<int>(std::round((cy - boxHeight * 0.5f - padY) / scale))));
                    box.right = std::max(0, std::min(width - 1,
                        static_cast<int>(std::round((cx + boxWidth * 0.5f - padX) / scale))));
                    box.bottom = std::max(0, std::min(height - 1,
                        static_cast<int>(std::round((cy + boxHeight * 0.5f - padY) / scale))));
                    box.confidence = confidence;
                    box.classId = classId;
                    box.yieldViolation = false;
                    if (box.right > box.left && box.bottom > box.top)
                        candidates.push_back(box);
                }
            }
        }
    }
    rknn_outputs_release(m_ctx, m_ioNum.n_output, outputs.data());

    std::sort(candidates.begin(), candidates.end(),
              [](const YoloBox &a, const YoloBox &b) {
                  return a.confidence > b.confidence;
              });
    std::vector<bool> suppressed(candidates.size(), false);
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (suppressed[i]) continue;
        result.boxes.push_back(candidates[i]);
        if (result.boxes.size() >= 128) break;
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (!suppressed[j] &&
                candidates[i].classId == candidates[j].classId &&
                intersectionOverUnion(candidates[i], candidates[j]) >
                    m_nmsThreshold) {
                suppressed[j] = true;
            }
        }
    }
    return true;
}
