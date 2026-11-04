#include "yolov8_detector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr int kClassCount = 80;
constexpr int kValuesPerAnchor = 5 + kClassCount;
constexpr int kAnchorsPerHead = 3;

const int kStrides[3] = {8, 16, 32};
const float kAnchors[3][kAnchorsPerHead][2] = {
    {{12.f, 16.f}, {19.f, 36.f}, {40.f, 28.f}},
    {{36.f, 75.f}, {76.f, 55.f}, {72.f, 146.f}},
    {{142.f, 110.f}, {192.f, 243.f}, {459.f, 401.f}}
};

const char *const kCocoClassNames[kClassCount] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus",
    "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
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
    const float intersection = static_cast<float>(
        std::max(0, right - left) * std::max(0, bottom - top));
    const float areaA = static_cast<float>(
        std::max(0, a.right - a.left) * std::max(0, a.bottom - a.top));
    const float areaB = static_cast<float>(
        std::max(0, b.right - b.left) * std::max(0, b.bottom - b.top));
    const float unionArea = areaA + areaB - intersection;
    return unionArea > 0.f ? intersection / unionArea : 0.f;
}

bool tensorShape(const rknn_tensor_attr &attr, int &channels,
                 int &height, int &width, bool &nhwc)
{
    if (attr.n_dims != 4) return false;
    nhwc = attr.fmt == RKNN_TENSOR_NHWC;
    if (nhwc) {
        height = static_cast<int>(attr.dims[1]);
        width = static_cast<int>(attr.dims[2]);
        channels = static_cast<int>(attr.dims[3]);
    } else {
        channels = static_cast<int>(attr.dims[1]);
        height = static_cast<int>(attr.dims[2]);
        width = static_cast<int>(attr.dims[3]);
    }
    return channels > 0 && height > 0 && width > 0;
}

float tensorValue(const float *data, bool nhwc, int channels,
                  int height, int width, int channel, int y, int x)
{
    if (nhwc) {
        const size_t index =
            (static_cast<size_t>(y) * width + x) * channels + channel;
        return data[index];
    }
    const size_t index =
        (static_cast<size_t>(channel) * height + y) * width + x;
    return data[index];
}

int headFromGrid(int gridWidth, int gridHeight, int inputWidth, int inputHeight)
{
    for (int i = 0; i < 3; ++i) {
        if (gridWidth * kStrides[i] == inputWidth &&
            gridHeight * kStrides[i] == inputHeight) {
            return i;
        }
    }
    return -1;
}

void computeDfl(const float *tensor, int dflLength, float distances[4])
{
    for (int side = 0; side < 4; ++side) {
        const float *values = tensor + side * dflLength;
        float maximum = values[0];
        for (int i = 1; i < dflLength; ++i) maximum = std::max(maximum, values[i]);
        float sum = 0.f, weighted = 0.f;
        for (int i = 0; i < dflLength; ++i) {
            const float probability = std::exp(values[i] - maximum);
            sum += probability;
            weighted += probability * i;
        }
        distances[side] = sum > 0.f ? weighted / sum : 0.f;
    }
}

void loadLabels(const char *path, std::vector<std::string> &labels)
{
    labels.clear();
    if (!path) return;
    std::ifstream stream(path);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) labels.push_back(line);
    }
}

} // namespace

YoloV8Detector::YoloV8Detector()
    : m_ctx(0), m_inputAttrs(nullptr), m_outputAttrs(nullptr),
      m_loaded(false), m_isYoloPv2(false), m_isDetect3d(false), m_isQuant(false),
      m_letterboxBuf(nullptr), m_bufSize(0),
      m_detectionOutputs{-1, -1, -1}, m_drivableOutput(-1), m_laneOutput(-1)
{
    std::memset(&m_ioNum, 0, sizeof(m_ioNum));
}

YoloV8Detector::~YoloV8Detector()
{
    release();
}

bool YoloV8Detector::loadModel(const char *modelPath, const char *labelPath,
                               const YoloV8Params &params)
{
    release();
    m_params = params;

    std::ifstream file(modelPath, std::ios::binary | std::ios::ate);
    if (!file) {
        std::fprintf(stderr, "YOLOPv2: cannot open model %s\n", modelPath);
        return false;
    }
    const std::streamsize length = file.tellg();
    if (length <= 0) return false;
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> model(static_cast<size_t>(length));
    if (!file.read(reinterpret_cast<char *>(model.data()), length)) return false;

    int ret = rknn_init(&m_ctx, model.data(), model.size(), 0, nullptr);
    if (ret != RKNN_SUCC) {
        std::fprintf(stderr, "YOLOPv2: rknn_init failed: %d\n", ret);
        release();
        return false;
    }

    ret = rknn_query(m_ctx, RKNN_QUERY_IN_OUT_NUM, &m_ioNum, sizeof(m_ioNum));
    if (ret != RKNN_SUCC || m_ioNum.n_input != 1 || m_ioNum.n_output == 0) {
        std::fprintf(stderr, "Detector: invalid input/output count %u/%u\n",
                     m_ioNum.n_input, m_ioNum.n_output);
        release();
        return false;
    }

    m_inputAttrs = new rknn_tensor_attr[m_ioNum.n_input]();
    m_outputAttrs = new rknn_tensor_attr[m_ioNum.n_output]();
    m_inputAttrs[0].index = 0;
    ret = rknn_query(m_ctx, RKNN_QUERY_INPUT_ATTR,
                     &m_inputAttrs[0], sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC || m_inputAttrs[0].n_dims != 4) {
        std::fprintf(stderr, "YOLOPv2: invalid input tensor\n");
        release();
        return false;
    }

    if (m_inputAttrs[0].fmt == RKNN_TENSOR_NCHW) {
        m_params.modelHeight = static_cast<int>(m_inputAttrs[0].dims[2]);
        m_params.modelWidth = static_cast<int>(m_inputAttrs[0].dims[3]);
    } else {
        m_params.modelHeight = static_cast<int>(m_inputAttrs[0].dims[1]);
        m_params.modelWidth = static_cast<int>(m_inputAttrs[0].dims[2]);
    }
    m_isYoloPv2 = (m_ioNum.n_output == 5 &&
                   m_params.modelWidth == 640 && m_params.modelHeight == 384);
    m_isDetect3d = false;
    if (m_isYoloPv2) m_params.numClasses = kClassCount;
    if (m_isYoloPv2 &&
        (m_params.modelWidth != 640 || m_params.modelHeight != 384)) {
        std::fprintf(stderr, "YOLOPv2: expected 640x384 input, got %dx%d\n",
                     m_params.modelWidth, m_params.modelHeight);
        release();
        return false;
    }

    m_detectionOutputs[0] = m_detectionOutputs[1] = m_detectionOutputs[2] = -1;
    m_drivableOutput = -1;
    m_laneOutput = -1;
    for (uint32_t i = 0; i < m_ioNum.n_output; ++i) {
        m_outputAttrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_OUTPUT_ATTR,
                         &m_outputAttrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            release();
            return false;
        }

        if (!m_isYoloPv2) continue;
        int channels = 0, height = 0, width = 0;
        bool nhwc = false;
        if (!tensorShape(m_outputAttrs[i], channels, height, width, nhwc)) {
            release();
            return false;
        }
        if (channels == kAnchorsPerHead * kValuesPerAnchor) {
            const int head = headFromGrid(width, height,
                                          m_params.modelWidth,
                                          m_params.modelHeight);
            if (head < 0 || m_detectionOutputs[head] >= 0) {
                std::fprintf(stderr, "YOLOPv2: invalid detection output %u\n", i);
                release();
                return false;
            }
            m_detectionOutputs[head] = static_cast<int>(i);
        } else if (channels == 2 && width == m_params.modelWidth &&
                   height == m_params.modelHeight) {
            m_drivableOutput = static_cast<int>(i);
        } else if (channels == 1 && width == m_params.modelWidth &&
                   height == m_params.modelHeight) {
            m_laneOutput = static_cast<int>(i);
        } else {
            std::fprintf(stderr, "YOLOPv2: unknown output %u [%d,%d,%d]\n",
                         i, channels, height, width);
            release();
            return false;
        }
    }

    // Detect3D has one decoded end-to-end output containing 300 rows x 14
    // values. Check n_elems after querying the output attribute.
    m_isDetect3d = m_ioNum.n_output == 1 &&
                   m_params.modelWidth == 1280 &&
                   m_params.modelHeight == 416 &&
                   m_outputAttrs[0].n_elems == 300u * 14u;

    if (m_isYoloPv2 && (m_detectionOutputs[0] < 0 || m_detectionOutputs[1] < 0 ||
        m_detectionOutputs[2] < 0 || m_drivableOutput < 0 ||
        m_laneOutput < 0)) {
        std::fprintf(stderr, "YOLOPv2: incomplete output set\n");
        release();
        return false;
    }

    m_bufSize = m_params.modelWidth * m_params.modelHeight * 3;
    m_letterboxBuf = new uint8_t[m_bufSize];
    m_isQuant = m_outputAttrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
                m_outputAttrs[0].type != RKNN_TENSOR_FLOAT16;
    if (m_isDetect3d) {
        m_params.numClasses = 3;
        static const char *const names[] = {"Car", "Pedestrian", "Cyclist"};
        m_labels.assign(names, names + 3);
    } else if (m_isYoloPv2)
        m_labels.assign(kCocoClassNames, kCocoClassNames + kClassCount);
    else
        loadLabels(labelPath, m_labels);
    if (!m_isYoloPv2 && !m_labels.empty())
        m_params.numClasses = static_cast<int>(m_labels.size());
    m_loaded = true;
    std::printf("%s loaded: %dx%d, outputs=%u\n",
                m_isDetect3d ? "Detect3D" : (m_isYoloPv2 ? "YOLOPv2" : "YOLOv8"),
                m_params.modelWidth, m_params.modelHeight, m_ioNum.n_output);
    return true;
}

void YoloV8Detector::release()
{
    if (m_ctx != 0) rknn_destroy(m_ctx);
    m_ctx = 0;
    delete[] m_inputAttrs;
    delete[] m_outputAttrs;
    delete[] m_letterboxBuf;
    m_inputAttrs = nullptr;
    m_outputAttrs = nullptr;
    m_letterboxBuf = nullptr;
    m_bufSize = 0;
    m_loaded = false;
    m_isYoloPv2 = false;
    m_isDetect3d = false;
    m_isQuant = false;
    std::memset(&m_ioNum, 0, sizeof(m_ioNum));
    m_detectionOutputs[0] = m_detectionOutputs[1] = m_detectionOutputs[2] = -1;
    m_drivableOutput = -1;
    m_laneOutput = -1;
    m_labels.clear();
}

const char *YoloV8Detector::className(int clsId) const
{
    return clsId >= 0 && clsId < static_cast<int>(m_labels.size())
        ? m_labels[clsId].c_str() : "?";
}

void YoloV8Detector::letterbox(const uint8_t *src, int srcW, int srcH,
                               uint8_t *dst, float &scale,
                               int &xPad, int &yPad,
                               int &resizedWidth, int &resizedHeight)
{
    scale = std::min(static_cast<float>(m_params.modelWidth) / srcW,
                     static_cast<float>(m_params.modelHeight) / srcH);
    resizedWidth = std::max(1, static_cast<int>(std::round(srcW * scale)));
    resizedHeight = std::max(1, static_cast<int>(std::round(srcH * scale)));
    xPad = (m_params.modelWidth - resizedWidth) / 2;
    yPad = (m_params.modelHeight - resizedHeight) / 2;

    cv::Mat source(srcH, srcW, CV_8UC3, const_cast<uint8_t *>(src));
    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resizedWidth, resizedHeight),
               0.0, 0.0, cv::INTER_LINEAR);
    std::memset(dst, 114, static_cast<size_t>(m_bufSize));
    cv::Mat target(m_params.modelHeight, m_params.modelWidth, CV_8UC3, dst);
    resized.copyTo(target(cv::Rect(xPad, yPad, resizedWidth, resizedHeight)));
}

bool YoloV8Detector::detect(const uint8_t *srcRGB, int width, int height,
                            YoloDetectResult &result)
{
    result = YoloDetectResult();
    if (!m_loaded || !srcRGB || width <= 0 || height <= 0) return false;
    if (m_isDetect3d)
        return detect3d(srcRGB, width, height, result);
    if (!m_isYoloPv2)
        return detectLegacyYoloV8(srcRGB, width, height, result);

    float scale = 1.f;
    int xPad = 0, yPad = 0, resizedWidth = 0, resizedHeight = 0;
    letterbox(srcRGB, width, height, m_letterboxBuf,
              scale, xPad, yPad, resizedWidth, resizedHeight);

    rknn_input input{};
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.size = static_cast<uint32_t>(m_bufSize);
    input.buf = m_letterboxBuf;
    input.pass_through = 0;
    int ret = rknn_inputs_set(m_ctx, 1, &input);
    if (ret != RKNN_SUCC) return false;
    ret = rknn_run(m_ctx, nullptr);
    if (ret != RKNN_SUCC) return false;

    std::vector<rknn_output> outputs(m_ioNum.n_output);
    std::memset(outputs.data(), 0, outputs.size() * sizeof(rknn_output));
    for (uint32_t i = 0; i < m_ioNum.n_output; ++i) {
        outputs[i].index = i;
        outputs[i].want_float = 1;
        outputs[i].is_prealloc = 0;
    }
    ret = rknn_outputs_get(m_ctx, m_ioNum.n_output, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) return false;

    bool ok = true;
    std::vector<YoloBox> candidates;
    candidates.reserve(512);
    const float objectLogitThreshold =
        std::log(m_params.boxThreshold / (1.f - m_params.boxThreshold));

    for (int head = 0; head < 3 && ok; ++head) {
        const int outputIndex = m_detectionOutputs[head];
        const rknn_tensor_attr &attr = m_outputAttrs[outputIndex];
        const float *data = static_cast<const float *>(outputs[outputIndex].buf);
        int channels = 0, gridHeight = 0, gridWidth = 0;
        bool nhwc = false;
        ok = data != nullptr &&
             tensorShape(attr, channels, gridHeight, gridWidth, nhwc);
        if (!ok) break;

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
                    if (confidence < m_params.boxThreshold) continue;
                    // YOLOPv2/COCO class 3 is motorcycle. Keep the model and
                    // tensor layout unchanged, but suppress this class before
                    // box decoding, NMS, display, WiFi output and yield logic.
                    if (classId == 3) continue;

                    const float sx = sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base, y, x));
                    const float sy = sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base + 1, y, x));
                    const float sw = sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base + 2, y, x)) * 2.f;
                    const float sh = sigmoid(tensorValue(
                        data, nhwc, channels, gridHeight, gridWidth,
                        base + 3, y, x)) * 2.f;
                    const float centerX = (sx * 2.f - 0.5f + x) * kStrides[head];
                    const float centerY = (sy * 2.f - 0.5f + y) * kStrides[head];
                    const float boxWidth = sw * sw * kAnchors[head][anchor][0];
                    const float boxHeight = sh * sh * kAnchors[head][anchor][1];

                    YoloBox box;
                    box.left = std::clamp(static_cast<int>(std::round(
                        (centerX - boxWidth * 0.5f - xPad) / scale)), 0, width - 1);
                    box.top = std::clamp(static_cast<int>(std::round(
                        (centerY - boxHeight * 0.5f - yPad) / scale)), 0, height - 1);
                    box.right = std::clamp(static_cast<int>(std::round(
                        (centerX + boxWidth * 0.5f - xPad) / scale)), 0, width - 1);
                    box.bottom = std::clamp(static_cast<int>(std::round(
                        (centerY + boxHeight * 0.5f - yPad) / scale)), 0, height - 1);
                    box.confidence = confidence;
                    box.classId = classId;
                    if (box.right > box.left && box.bottom > box.top)
                        candidates.push_back(box);
                }
            }
        }
    }

    if (ok) {
        std::sort(candidates.begin(), candidates.end(),
                  [](const YoloBox &a, const YoloBox &b) {
                      return a.confidence > b.confidence;
                  });
        std::vector<bool> suppressed(candidates.size(), false);
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (suppressed[i]) continue;
            result.boxes.push_back(candidates[i]);
            if (result.boxes.size() >= 300) break;
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                if (!suppressed[j] &&
                    candidates[i].classId == candidates[j].classId &&
                    intersectionOverUnion(candidates[i], candidates[j]) >
                        m_params.nmsThreshold) {
                    suppressed[j] = true;
                }
            }
        }

        const int driveIndex = m_drivableOutput;
        const int laneIndex = m_laneOutput;
        const float *driveData = static_cast<const float *>(outputs[driveIndex].buf);
        const float *laneData = static_cast<const float *>(outputs[laneIndex].buf);
        int driveChannels = 0, driveHeight = 0, driveWidth = 0;
        int laneChannels = 0, laneHeight = 0, laneWidth = 0;
        bool driveNhwc = false, laneNhwc = false;
        ok = driveData != nullptr && laneData != nullptr &&
             tensorShape(m_outputAttrs[driveIndex], driveChannels,
                         driveHeight, driveWidth, driveNhwc) &&
             tensorShape(m_outputAttrs[laneIndex], laneChannels,
                         laneHeight, laneWidth, laneNhwc) &&
             driveChannels == 2 && laneChannels == 1;

        if (ok) {
            cv::Mat driveModel(driveHeight, driveWidth, CV_8UC1);
            cv::Mat laneModel(laneHeight, laneWidth, CV_8UC1);
            for (int y = 0; y < driveHeight; ++y) {
                uint8_t *driveRow = driveModel.ptr<uint8_t>(y);
                uint8_t *laneRow = laneModel.ptr<uint8_t>(y);
                for (int x = 0; x < driveWidth; ++x) {
                    const float background = tensorValue(
                        driveData, driveNhwc, driveChannels,
                        driveHeight, driveWidth, 0, y, x);
                    const float foreground = tensorValue(
                        driveData, driveNhwc, driveChannels,
                        driveHeight, driveWidth, 1, y, x);
                    driveRow[x] = foreground > background ? 255 : 0;
                    laneRow[x] = tensorValue(
                        laneData, laneNhwc, laneChannels,
                        laneHeight, laneWidth, 0, y, x) >= 0.5f ? 255 : 0;
                }
            }

            const cv::Rect content(xPad, yPad, resizedWidth, resizedHeight);
            cv::Mat driveRestored, laneRestored;
            cv::resize(driveModel(content), driveRestored, cv::Size(width, height),
                       0.0, 0.0, cv::INTER_NEAREST);
            cv::resize(laneModel(content), laneRestored, cv::Size(width, height),
                       0.0, 0.0, cv::INTER_NEAREST);

            result.segmentationWidth = width;
            result.segmentationHeight = height;
            result.drivableMask.assign(driveRestored.data,
                                       driveRestored.data + driveRestored.total());
            result.laneMask.assign(laneRestored.data,
                                   laneRestored.data + laneRestored.total());
        }
    }

    rknn_outputs_release(m_ctx, m_ioNum.n_output, outputs.data());
    if (!ok) result = YoloDetectResult();
    return ok;
}

bool YoloV8Detector::detect3d(const uint8_t *srcRGB, int width, int height,
                              YoloDetectResult &result)
{
    float scale = 1.f;
    int xPad = 0, yPad = 0, resizedWidth = 0, resizedHeight = 0;
    letterbox(srcRGB, width, height, m_letterboxBuf, scale, xPad, yPad,
              resizedWidth, resizedHeight);

    rknn_input input{};
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.size = static_cast<uint32_t>(m_bufSize);
    input.buf = m_letterboxBuf;
    if (rknn_inputs_set(m_ctx, 1, &input) != RKNN_SUCC ||
        rknn_run(m_ctx, nullptr) != RKNN_SUCC) return false;

    rknn_output output{};
    output.index = 0;
    output.want_float = 1;
    if (rknn_outputs_get(m_ctx, 1, &output, nullptr) != RKNN_SUCC) return false;
    const float *rows = static_cast<const float *>(output.buf);
    bool ok = rows != nullptr;

    // Scalable average calibration: square pixels and a 70-degree horizontal
    // field of view. Environment variables allow replacement without rebuild.
    const float pi = 3.14159265358979323846f;
    float hfov = 70.f;
    if (const char *value = std::getenv("HS_CAMERA_HFOV_DEG")) {
        const float parsed = std::strtof(value, nullptr);
        if (parsed >= 30.f && parsed <= 140.f) hfov = parsed;
    }
    float fx = width / (2.f * std::tan(hfov * pi / 360.f));
    float fy = fx;
    float cx = width * 0.5f;
    float cy = height * 0.5f;
    if (const char *value = std::getenv("HS_CAMERA_FX")) fx = std::strtof(value, nullptr);
    if (const char *value = std::getenv("HS_CAMERA_FY")) fy = std::strtof(value, nullptr);
    if (const char *value = std::getenv("HS_CAMERA_CX")) cx = std::strtof(value, nullptr);
    if (const char *value = std::getenv("HS_CAMERA_CY")) cy = std::strtof(value, nullptr);
    ok = ok && fx > 1.f && fy > 1.f;

    for (int i = 0; i < 300 && ok; ++i) {
        const float *p = rows + i * 14;
        const float confidence = p[4];
        const int classId = static_cast<int>(std::round(p[5]));
        if (!std::isfinite(confidence) || confidence < m_params.boxThreshold ||
            classId < 0 || classId >= 3) continue;

        YoloBox box;
        box.left = std::clamp(static_cast<int>(std::round((p[0] - xPad) / scale)), 0, width - 1);
        box.top = std::clamp(static_cast<int>(std::round((p[1] - yPad) / scale)), 0, height - 1);
        box.right = std::clamp(static_cast<int>(std::round((p[2] - xPad) / scale)), 0, width - 1);
        box.bottom = std::clamp(static_cast<int>(std::round((p[3] - yPad) / scale)), 0, height - 1);
        box.confidence = confidence;
        box.classId = classId;

        const float centerU = (p[6] - xPad) / scale;
        const float centerV = (p[7] - yPad) / scale;
        const float h3 = p[11], w3 = p[12], l3 = p[13];
        if (h3 <= 0.f || w3 <= 0.f || l3 <= 0.f ||
            !std::isfinite(h3 + w3 + l3)) continue;

        // Estimate absolute distance from the 2D box instead of using the
        // KITTI-camera-specific monocular depth p[8]. This keeps the distance
        // label and cuboid projection on the same camera-dependent scale.
        const float pixelBoxWidth = std::max(1.0f,
            static_cast<float>(box.right - box.left));
        const float pixelBoxHeight = std::max(1.0f,
            static_cast<float>(box.bottom - box.top));
        static const float averageObjectHeight[3] = {
            1.50f, // Car
            1.70f, // Pedestrian
            1.70f  // Cyclist including rider
        };
        const float depthFromHeight =
            fy * averageObjectHeight[classId] / pixelBoxHeight;
        float estimatedDepth = depthFromHeight;
        if (classId == 0) {
            constexpr float averageCarWidth = 1.80f;
            const float depthFromWidth = fx * averageCarWidth / pixelBoxWidth;
            estimatedDepth = depthFromHeight * 0.70f + depthFromWidth * 0.30f;
        }
        if (!std::isfinite(estimatedDepth)) continue;
        const float z = std::clamp(estimatedDepth, 1.0f, 150.0f);

        // Reject implausible geometry before perspective division. A single
        // bad depth/dimension residual can otherwise place one corner almost
        // on the camera plane and create a very long or flattened cuboid.
        static const float dimensionLimits[3][6] = {
            // min/max height, min/max width, min/max length (metres)
            {1.0f, 3.2f, 1.1f, 3.2f, 2.0f, 8.5f}, // Car
            {1.0f, 2.6f, 0.2f, 1.5f, 0.2f, 2.0f}, // Pedestrian
            {1.0f, 2.6f, 0.2f, 1.5f, 0.5f, 3.8f}  // Cyclist
        };
        const float *limits = dimensionLimits[classId];
        // Do not drop an otherwise valid 2D detection when a monocular size
        // residual crosses the class range. Clamp the physical dimensions and
        // retain the original eight-corner ordering; the image-space envelope
        // is constrained again below to the 2D box safety region.
        const float safeH3 = std::clamp(h3, limits[0], limits[1]);
        const float safeW3 = std::clamp(w3, limits[2], limits[3]);
        const float safeL3 = std::clamp(l3, limits[4], limits[5]);

        const float x3 = (centerU - cx) * z / fx;
        const float y3 = (centerV - cy) * z / fy;
        const float alpha = std::atan2(p[9], p[10]);
        const float rotationY = alpha + std::atan2(x3, z);
        const float c = std::cos(rotationY), s = std::sin(rotationY);
        const float localX[8] = {-safeW3/2, safeW3/2, safeW3/2, -safeW3/2,
                                 -safeW3/2, safeW3/2, safeW3/2, -safeW3/2};
        const float localY[8] = {-safeH3/2, -safeH3/2, -safeH3/2, -safeH3/2,
                                  safeH3/2,  safeH3/2,  safeH3/2,  safeH3/2};
        const float localZ[8] = {-safeL3/2, -safeL3/2, safeL3/2, safeL3/2,
                                 -safeL3/2, -safeL3/2, safeL3/2, safeL3/2};
        bool projectionValid = true;
        float projectedMinX = std::numeric_limits<float>::infinity();
        float projectedMinY = std::numeric_limits<float>::infinity();
        float projectedMaxX = -std::numeric_limits<float>::infinity();
        float projectedMaxY = -std::numeric_limits<float>::infinity();
        const float minimumCornerDepth = std::max(0.5f, z * 0.15f);
        for (int corner = 0; corner < 8; ++corner) {
            if (!projectionValid) break;
            const float cameraX = x3 + c * localX[corner] + s * localZ[corner];
            const float cameraY = y3 + localY[corner];
            const float cameraZ = z - s * localX[corner] + c * localZ[corner];
            if (cameraZ < minimumCornerDepth || cameraZ > z * 3.0f ||
                !std::isfinite(cameraX + cameraY + cameraZ)) {
                projectionValid = false;
                break;
            }
            const float projectedX = fx * cameraX / cameraZ + cx;
            const float projectedY = fy * cameraY / cameraZ + cy;
            // Large but finite coordinates can be recovered below by the 2D
            // envelope constraint. Only reject numerical explosions here.
            if (!std::isfinite(projectedX + projectedY) ||
                std::fabs(projectedX) > 100000.f ||
                std::fabs(projectedY) > 100000.f) {
                projectionValid = false;
                break;
            }
            box.projectedCorners[corner * 2] = projectedX;
            box.projectedCorners[corner * 2 + 1] = projectedY;
            projectedMinX = std::min(projectedMinX, projectedX);
            projectedMinY = std::min(projectedMinY, projectedY);
            projectedMaxX = std::max(projectedMaxX, projectedX);
            projectedMaxY = std::max(projectedMaxY, projectedY);
        }

        if (projectionValid) {
            const float boxWidth2d = static_cast<float>(box.right - box.left);
            const float boxHeight2d = static_cast<float>(box.bottom - box.top);
            const float projectionWidth = projectedMaxX - projectedMinX;
            const float projectionHeight = projectedMaxY - projectedMinY;
            const float widthRatio = projectionWidth / std::max(1.0f, boxWidth2d);
            const float heightRatio = projectionHeight / std::max(1.0f, boxHeight2d);
            const float projectedCenterX = (projectedMinX + projectedMaxX) * 0.5f;
            const float projectedCenterY = (projectedMinY + projectedMaxY) * 0.5f;
            const float boxCenterX = (box.left + box.right) * 0.5f;
            const float boxCenterY = (box.top + box.bottom) * 0.5f;
            const float centerErrorX = std::fabs(projectedCenterX - boxCenterX) /
                                       std::max(1.0f, boxWidth2d);
            const float centerErrorY = std::fabs(projectedCenterY - boxCenterY) /
                                       std::max(1.0f, boxHeight2d);

            // Keep geometrically consistent predictions untouched. If the
            // monocular geometry disagrees strongly with its own 2D box,
            // immediately fit the eight projected points to the 2D vehicle
            // envelope. This preserves point topology and perspective ordering
            // while preventing stretched, flattened or remote cuboids.
            const bool needs2dConstraint =
                widthRatio < 0.60f || widthRatio > 1.80f ||
                heightRatio < 0.60f || heightRatio > 1.80f ||
                centerErrorX > 0.35f || centerErrorY > 0.35f;
            if (needs2dConstraint && projectionWidth > 1e-3f &&
                projectionHeight > 1e-3f) {
                const float targetWidth = boxWidth2d;
                const float targetHeight = boxHeight2d;
                const float scaleX = targetWidth / projectionWidth;
                const float scaleY = targetHeight / projectionHeight;
                for (int corner = 0; corner < 8; ++corner) {
                    const int offset = corner * 2;
                    box.projectedCorners[offset] = boxCenterX +
                        (box.projectedCorners[offset] - projectedCenterX) * scaleX;
                    box.projectedCorners[offset + 1] = boxCenterY +
                        (box.projectedCorners[offset + 1] - projectedCenterY) * scaleY;
                }
            }

            // Final coherent safety fit. Transform the complete cuboid as one
            // unit instead of clipping individual points; this keeps every
            // edge connected while moving the envelope inside the 2D box.
            const float safeLeft = box.left;
            const float safeRight = box.right;
            const float safeTop = box.top;
            const float safeBottom = box.bottom;
            float fittedMinX = std::numeric_limits<float>::infinity();
            float fittedMinY = std::numeric_limits<float>::infinity();
            float fittedMaxX = -std::numeric_limits<float>::infinity();
            float fittedMaxY = -std::numeric_limits<float>::infinity();
            for (int corner = 0; corner < 8; ++corner) {
                const int offset = corner * 2;
                fittedMinX = std::min(fittedMinX, box.projectedCorners[offset]);
                fittedMaxX = std::max(fittedMaxX, box.projectedCorners[offset]);
                fittedMinY = std::min(fittedMinY, box.projectedCorners[offset + 1]);
                fittedMaxY = std::max(fittedMaxY, box.projectedCorners[offset + 1]);
            }
            const float fittedWidth = std::max(1e-3f, fittedMaxX - fittedMinX);
            const float fittedHeight = std::max(1e-3f, fittedMaxY - fittedMinY);
            const float coherentScale = std::min(1.0f, std::min(
                boxWidth2d / fittedWidth, boxHeight2d / fittedHeight));
            const float fittedCenterX = (fittedMinX + fittedMaxX) * 0.5f;
            const float fittedCenterY = (fittedMinY + fittedMaxY) * 0.5f;
            const float scaledMinX = fittedCenterX +
                (fittedMinX - fittedCenterX) * coherentScale;
            const float scaledMaxX = fittedCenterX +
                (fittedMaxX - fittedCenterX) * coherentScale;
            const float scaledMinY = fittedCenterY +
                (fittedMinY - fittedCenterY) * coherentScale;
            const float scaledMaxY = fittedCenterY +
                (fittedMaxY - fittedCenterY) * coherentScale;
            float shiftX = 0.0f;
            float shiftY = 0.0f;
            if (scaledMinX < safeLeft) shiftX = safeLeft - scaledMinX;
            else if (scaledMaxX > safeRight) shiftX = safeRight - scaledMaxX;
            if (scaledMinY < safeTop) shiftY = safeTop - scaledMinY;
            else if (scaledMaxY > safeBottom) shiftY = safeBottom - scaledMaxY;
            for (int corner = 0; corner < 8; ++corner) {
                const int offset = corner * 2;
                box.projectedCorners[offset] = fittedCenterX +
                    (box.projectedCorners[offset] - fittedCenterX) *
                    coherentScale + shiftX;
                box.projectedCorners[offset + 1] = fittedCenterY +
                    (box.projectedCorners[offset + 1] - fittedCenterY) *
                    coherentScale + shiftY;
            }
        }
        box.has3dProjection = projectionValid;
        box.depthMeters = z;
        box.dimensions3d[0] = safeH3;
        box.dimensions3d[1] = safeW3;
        box.dimensions3d[2] = safeL3;
        if (box.right > box.left && box.bottom > box.top)
            result.boxes.push_back(box);
    }
    rknn_outputs_release(m_ctx, 1, &output);
    if (!ok) result = YoloDetectResult();
    return ok;
}

bool YoloV8Detector::detectLegacyYoloV8(const uint8_t *srcRGB, int width,
                                        int height, YoloDetectResult &result)
{
    float scale = 1.f;
    int xPad = 0, yPad = 0, resizedWidth = 0, resizedHeight = 0;
    letterbox(srcRGB, width, height, m_letterboxBuf, scale, xPad, yPad,
              resizedWidth, resizedHeight);

    rknn_input input{};
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.size = static_cast<uint32_t>(m_bufSize);
    input.buf = m_letterboxBuf;
    input.pass_through = 0;
    if (rknn_inputs_set(m_ctx, 1, &input) != RKNN_SUCC ||
        rknn_run(m_ctx, nullptr) != RKNN_SUCC) return false;

    std::vector<rknn_output> outputs(m_ioNum.n_output);
    std::memset(outputs.data(), 0, outputs.size() * sizeof(rknn_output));
    for (uint32_t i = 0; i < m_ioNum.n_output; ++i) {
        outputs[i].index = i;
        // Float conversion keeps this compatibility path correct for both the
        // FP16 helmet model and quantized YOLOv8 exports.
        outputs[i].want_float = 1;
    }
    if (rknn_outputs_get(m_ctx, m_ioNum.n_output, outputs.data(), nullptr) !=
        RKNN_SUCC) return false;

    bool ok = m_ioNum.n_output >= 6 && m_ioNum.n_output % 3 == 0;
    std::vector<YoloBox> candidates;
    const int outputsPerBranch = static_cast<int>(m_ioNum.n_output / 3);
    for (int branch = 0; branch < 3 && ok; ++branch) {
        const int boxIndex = branch * outputsPerBranch;
        const int scoreIndex = boxIndex + 1;
        const rknn_tensor_attr &boxAttr = m_outputAttrs[boxIndex];
        const rknn_tensor_attr &scoreAttr = m_outputAttrs[scoreIndex];
        int boxChannels = 0, gridHeight = 0, gridWidth = 0;
        int scoreChannels = 0, scoreHeight = 0, scoreWidth = 0;
        bool boxNhwc = false, scoreNhwc = false;
        ok = tensorShape(boxAttr, boxChannels, gridHeight, gridWidth, boxNhwc) &&
             tensorShape(scoreAttr, scoreChannels, scoreHeight, scoreWidth,
                         scoreNhwc) &&
             gridHeight == scoreHeight && gridWidth == scoreWidth &&
             boxChannels % 4 == 0;
        if (!ok) break;
        const int dflLength = boxChannels / 4;
        const int classCount = std::min(m_params.numClasses, scoreChannels);
        const int stride = m_params.modelHeight / gridHeight;
        const float *boxData = static_cast<const float *>(outputs[boxIndex].buf);
        const float *scoreData = static_cast<const float *>(outputs[scoreIndex].buf);
        if (!boxData || !scoreData || dflLength <= 0 || classCount <= 0) {
            ok = false;
            break;
        }

        std::vector<float> dfl(static_cast<size_t>(boxChannels));
        for (int y = 0; y < gridHeight; ++y) {
            for (int x = 0; x < gridWidth; ++x) {
                int classId = -1;
                float confidence = m_params.boxThreshold;
                for (int cls = 0; cls < classCount; ++cls) {
                    const float score = tensorValue(scoreData, scoreNhwc,
                        scoreChannels, gridHeight, gridWidth, cls, y, x);
                    if (score > confidence) {
                        confidence = score;
                        classId = cls;
                    }
                }
                if (classId < 0) continue;
                for (int channel = 0; channel < boxChannels; ++channel)
                    dfl[channel] = tensorValue(boxData, boxNhwc, boxChannels,
                        gridHeight, gridWidth, channel, y, x);
                float distance[4];
                computeDfl(dfl.data(), dflLength, distance);
                const float centerX = (x + 0.5f) * stride;
                const float centerY = (y + 0.5f) * stride;
                YoloBox box;
                box.left = std::clamp(static_cast<int>((centerX - distance[0] * stride - xPad) / scale), 0, width - 1);
                box.top = std::clamp(static_cast<int>((centerY - distance[1] * stride - yPad) / scale), 0, height - 1);
                box.right = std::clamp(static_cast<int>((centerX + distance[2] * stride - xPad) / scale), 0, width - 1);
                box.bottom = std::clamp(static_cast<int>((centerY + distance[3] * stride - yPad) / scale), 0, height - 1);
                box.confidence = confidence;
                box.classId = classId;
                if (box.right > box.left && box.bottom > box.top)
                    candidates.push_back(box);
            }
        }
    }

    if (ok) {
        std::sort(candidates.begin(), candidates.end(),
                  [](const YoloBox &a, const YoloBox &b) {
                      return a.confidence > b.confidence;
                  });
        std::vector<bool> suppressed(candidates.size(), false);
        for (size_t i = 0; i < candidates.size() && result.boxes.size() < 128; ++i) {
            if (suppressed[i]) continue;
            result.boxes.push_back(candidates[i]);
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                if (!suppressed[j] && candidates[i].classId == candidates[j].classId &&
                    intersectionOverUnion(candidates[i], candidates[j]) >
                        m_params.nmsThreshold) suppressed[j] = true;
            }
        }
    }
    rknn_outputs_release(m_ctx, m_ioNum.n_output, outputs.data());
    if (!ok) result = YoloDetectResult();
    return ok;
}
