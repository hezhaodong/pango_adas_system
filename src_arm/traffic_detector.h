#ifndef TRAFFIC_DETECTOR_H
#define TRAFFIC_DETECTOR_H

#include "yolov8_detector.h"

#include <rknn_api.h>
#include <cstdint>
#include <vector>

class TrafficDetector
{
public:
    TrafficDetector();
    ~TrafficDetector();

    bool loadModel(const char *modelPath);
    void release();
    bool isLoaded() const { return m_ctx != 0; }
    bool detect(const uint8_t *rgb, int width, int height,
                YoloDetectResult &result);
    const char *className(int id) const;

private:
    TrafficDetector(const TrafficDetector &) = delete;
    TrafficDetector &operator=(const TrafficDetector &) = delete;

    rknn_context m_ctx;
    rknn_input_output_num m_ioNum;
    rknn_tensor_attr m_inputAttr;
    std::vector<rknn_tensor_attr> m_outputAttrs;
    std::vector<uint8_t> m_modelData;
    std::vector<uint8_t> m_letterbox;
    int m_inputWidth;
    int m_inputHeight;
    float m_confThreshold;
    float m_nmsThreshold;
};

#endif // TRAFFIC_DETECTOR_H
