#ifndef YOLOV8_WORKER_H
#define YOLOV8_WORKER_H

#include <QObject>
#include <QMutex>
#include <QElapsedTimer>
#include <utility>
#include <cstdint>
#include "frame_buffer.h"
#include "yolov8_detector.h"
#include "lane_departure_analyzer.h"

// ============================================================================
// Shared by YOLOPv2, legacy YOLOv8 and Detect3D. Multiple instances may share
// DetectionWorker's QThread so Qt serializes their NPU calls.
// ============================================================================
class YoloV8Worker : public QObject
{
    Q_OBJECT

public:
    explicit YoloV8Worker(QObject *parent = nullptr);
    ~YoloV8Worker();

    void setFrameBuffer(FrameBuffer *fb) { m_frameBuffer = fb; }
    void setYieldDetectionEnabled(bool enabled)
    {
        m_yieldDetectionEnabled = enabled;
    }

    bool loadModel(const char *modelPath, const char *labelPath,
                   const YoloV8Params &params = YoloV8Params());
    bool isLoaded() const { return m_detector.isLoaded(); }
    int  numClasses() const { return m_detector.numClasses(); }
    const char *className(int id) const { return m_detector.className(id); }

    // 原始检测结果
    bool getLatestResult(YoloDetectResult &outResult, int &sourceFrameSeq)
    {
        QMutexLocker lock(&m_resultMutex);
        if (!m_hasResult) return false;
        outResult = std::move(m_stableResult);
        sourceFrameSeq = m_stableFrameSeq;
        m_hasResult = false;
        return true;
    }

signals:
    void inferenceDone(int frameSeq);
    void statusMessage(const QString &msg);

public slots:
    void startInference();
    void stopInference();
    void resetSceneState();

private:
    void processNextFrame();
    void detectYieldViolation(YoloDetectResult &result);

    FrameBuffer    *m_frameBuffer;
    uint8_t        *m_frameBuf565;
    uint8_t        *m_rgb888Buf;

    YoloV8Detector  m_detector;
    YoloV8Params    m_params;
    LaneDepartureAnalyzer m_laneDepartureAnalyzer;

    bool m_running;
    bool m_yieldDetectionEnabled;
    int  m_lastProcessedSeq;
    QElapsedTimer m_violationAlertTimer;
    QElapsedTimer m_crosswalkGateTimer;

    mutable QMutex m_resultMutex;
    YoloDetectResult m_stableResult;
    int              m_stableFrameSeq;
    bool             m_hasResult;

};

#endif // YOLOV8_WORKER_H
