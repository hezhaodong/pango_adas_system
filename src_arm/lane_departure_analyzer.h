#ifndef LANE_DEPARTURE_ANALYZER_H
#define LANE_DEPARTURE_ANALYZER_H

#include "yolov8_detector.h"

class LaneDepartureAnalyzer
{
public:
    struct Sample { float y; float x; };
    struct Line { float a = 0.0f; float b = 0.0f; int inliers = 0; };
    void configure(const YoloV8Params &params);
    void reset();
    void update(YoloDetectResult &result);

private:
    bool fitRobust(const std::vector<Sample> &points, int width,
                   Line &line) const;
    bool fitAnchoredRoadCurve(const std::vector<Sample> &points,
                              int width, int curveTopY, int anchorY,
                              float anchorX, float yellowSlope,
                              float &farOffsetPx, float &confidence) const;
    static float lineX(const Line &line, float y)
    { return line.a * y + line.b; }

    bool m_enabled = true;
    float m_cameraCenterRatio = 0.50f;
    float m_warningThreshold = 0.15f;
    float m_clearThreshold = 0.10f;
    int m_confirmFrames = 3;
    bool m_hasSmoothGeometry = false;
    float m_leftTop = 0.0f;
    float m_leftBottom = 0.0f;
    float m_rightTop = 0.0f;
    float m_rightBottom = 0.0f;
    int m_candidateDirection = 0;
    int m_candidateFrames = 0;
    int m_alertDirection = 0;
    int m_clearFrames = 0;
    int m_missingFrames = 0;
    int m_roadCandidateDirection = 0;
    int m_roadCandidateFrames = 0;
    int m_roadDirection = 0;
};

#endif
