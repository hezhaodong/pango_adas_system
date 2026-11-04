#ifndef YOLOV8_DETECTOR_H
#define YOLOV8_DETECTOR_H

#include <cstdint>
#include <string>
#include <vector>

#include "rknn_api.h"

// Shared axis-aligned detection result used by the existing UI and the other
// small detectors.  The class name is kept for source compatibility while the
// main traffic backend is now YOLOPv2.
struct YoloBox {
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
    float confidence = 0.0f;
    int classId = -1;
    bool yieldViolation = false;
    // Monocular Detect3D result. projectedCorners stores eight (x,y) pairs
    // in source-frame pixels. Invalid/behind-camera corners disable drawing.
    bool has3dProjection = false;
    float depthMeters = 0.0f;
    float dimensions3d[3] = {0.0f, 0.0f, 0.0f}; // height, width, length
    float projectedCorners[16] = {0.0f};
};

struct YoloDetectResult {
    std::vector<YoloBox> boxes;
    bool yieldViolationAlert = false;
    // Dedicated feature alerts, rendered as red text in the upper-right.
    bool helmetNoHelmetAlert = false;
    bool redLightCrosswalkPedestrianAlert = false;

    // YOLOPv2 semantic outputs, mapped to the source-frame coordinates.
    // Empty for detectors that only produce boxes.
    int segmentationWidth = 0;
    int segmentationHeight = 0;
    std::vector<uint8_t> drivableMask;
    std::vector<uint8_t> laneMask;

    // Debug/visualization center tracks extracted from the thick YOLOPv2
    // lane mask. Each pair of vectors stores one center sample per horizontal
    // scan band for the current lane boundary nearest the camera center.
    std::vector<int> laneLeftCenterXs;
    std::vector<int> laneLeftCenterYs;
    std::vector<int> laneRightCenterXs;
    std::vector<int> laneRightCenterYs;

    // Forward-camera lane geometry derived from laneMask. Coordinates are in
    // source-frame pixels. laneOffsetRatio is positive when the camera/vehicle
    // is to the right of the fitted lane center and negative on the left.
    bool laneGeometryValid = false;
    int laneGeometryTopY = 0;
    int laneGeometryBottomY = 0;
    int laneLeftTopX = 0;
    int laneLeftBottomX = 0;
    int laneRightTopX = 0;
    int laneRightBottomX = 0;
    int laneCenterX = 0;
    int cameraCenterX = 0;
    float laneOffsetRatio = 0.0f;
    float laneGeometryConfidence = 0.0f;
    int laneLeftSamples = 0;
    int laneRightSamples = 0;
    int laneFitStatus = 0; // 0 success, 1 left fit, 2 right fit, 3 geometry
    int laneDepartureDirection = 0; // -1 left, +1 right, 0 none
    bool laneDepartureAlert = false;
    int roadDirection = 0; // -1 left curve, 0 straight, +1 right curve
    // Road direction is derived from lane-mask evidence retained inside the
    // forward wedge formed by the two yellow near-field fits.
    // roadDirectionRatio is the signed left/right source balance:
    // negative = more right-boundary evidence => left bend; positive => right.
    float roadDirectionRatio = 0.0f;
    float roadDirectionConfidence = 0.0f;
    // Legacy field names kept to minimize interface churn. In the triangle-ROI
    // trial these carry compact source-evidence counts rather than curvature.
    float roadLeftCurvature = 0.0f;
    float roadRightCurvature = 0.0f;
    bool roadLeftCurveValid = false;
    bool roadRightCurveValid = false;
    int roadCurveTopY = 0;
    float roadLeftCurveConfidence = 0.0f;
    float roadRightCurveConfidence = 0.0f;

    // Triangle-ROI lane evidence. The two existing yellow lane fits define a
    // forward wedge. Only lane-mask centers inside that wedge are retained.
    // Left-source points come from the left half/boundary; right-source points
    // come from the right half/boundary.  Direction uses both centre-source
    // counts and the amount of red mask retained from each source in the far
    // wedge; right-source dominance votes LEFT and left-source dominance RIGHT.
    int roadTriangleLeftSourcePoints = 0;
    int roadTriangleRightSourcePoints = 0;
    int roadLeftVotePoints = 0;
    int roadRightVotePoints = 0;
    int roadStraightVotePoints = 0;

    bool hasSegmentation() const
    {
        const size_t expected = static_cast<size_t>(segmentationWidth) *
                                static_cast<size_t>(segmentationHeight);
        return segmentationWidth > 0 && segmentationHeight > 0 &&
               drivableMask.size() == expected && laneMask.size() == expected;
    }
};

struct YoloV8Params {
    int numClasses = 80;
    float boxThreshold = 0.30f;
    float nmsThreshold = 0.45f;
    int modelWidth = 640;
    int modelHeight = 384;
    bool laneDepartureEnabled = true;
    float cameraCenterRatio = 0.50f;
    float laneDepartureThreshold = 0.15f;
    int laneDepartureConfirmFrames = 3;
};

// Compatibility-facing class name.  Its implementation is YOLOPv2:
// 3 detection heads + drivable-area segmentation + lane segmentation.
class YoloV8Detector
{
public:
    YoloV8Detector();
    ~YoloV8Detector();

    bool loadModel(const char *modelPath, const char *labelPath,
                   const YoloV8Params &params = YoloV8Params());
    void release();
    bool isLoaded() const { return m_loaded; }

    bool detect(const uint8_t *srcRGB, int width, int height,
                YoloDetectResult &result);

    const char *className(int clsId) const;
    int numClasses() const { return m_params.numClasses; }
    int modelWidth() const { return m_params.modelWidth; }
    int modelHeight() const { return m_params.modelHeight; }
    bool isDetect3d() const { return m_isDetect3d; }

private:
    void letterbox(const uint8_t *src, int srcW, int srcH,
                   uint8_t *dst, float &scale, int &xPad, int &yPad,
                   int &resizedWidth, int &resizedHeight);
    bool detectLegacyYoloV8(const uint8_t *srcRGB, int width, int height,
                            YoloDetectResult &result);
    bool detect3d(const uint8_t *srcRGB, int width, int height,
                  YoloDetectResult &result);

    rknn_context m_ctx;
    rknn_input_output_num m_ioNum;
    rknn_tensor_attr *m_inputAttrs;
    rknn_tensor_attr *m_outputAttrs;

    YoloV8Params m_params;
    bool m_loaded;
    bool m_isYoloPv2;
    bool m_isDetect3d;
    bool m_isQuant;
    uint8_t *m_letterboxBuf;
    int m_bufSize;

    int m_detectionOutputs[3];
    int m_drivableOutput;
    int m_laneOutput;
    std::vector<std::string> m_labels;
};

#endif // YOLOV8_DETECTOR_H
