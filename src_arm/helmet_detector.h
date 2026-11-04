#ifndef HELMET_DETECTOR_H
#define HELMET_DETECTOR_H

#include "yolov8_detector.h"
#include "rknn_api.h"

#include <cstdint>

// YOLO11 helmet model: one decoded output [1, 7, 8400].
// Classes: 0 motorcycle, 1 helmet, 2 no-helmet.
class HelmetDetector
{
public:
    HelmetDetector();
    ~HelmetDetector();

    bool loadModel(const char *modelPath);
    void release();
    bool isLoaded() const { return m_loaded; }
    bool detect(const uint8_t *srcRgb, int width, int height,
                YoloDetectResult &result);
    const char *className(int classId) const;

private:
    void letterbox(const uint8_t *src, int srcW, int srcH,
                   uint8_t *dst, float &scale, int &xPad, int &yPad);

    rknn_context m_ctx;
    rknn_input_output_num m_ioNum;
    rknn_tensor_attr *m_inputAttrs;
    rknn_tensor_attr *m_outputAttrs;
    uint8_t *m_letterboxBuf;
    bool m_loaded;
    int m_modelWidth;
    int m_modelHeight;
};

#endif
