#ifndef HELMET_WORKER_H
#define HELMET_WORKER_H

#include "frame_buffer.h"
#include "yolov8_detector.h"

#include <QMutex>
#include <QObject>

class HelmetWorker : public QObject
{
    Q_OBJECT
public:
    explicit HelmetWorker(QObject *parent = nullptr);
    ~HelmetWorker();
    void setFrameBuffer(FrameBuffer *buffer) { m_frameBuffer = buffer; }
    bool isLoaded() const { return m_detector.isLoaded(); }
    bool loadModel(const char *modelPath, const char *labelPath,
                   const YoloV8Params &params)
    { return m_detector.loadModel(modelPath, labelPath, params); }
    const char *className(int id) const { return m_detector.className(id); }
    bool getLatestResult(YoloDetectResult &result, int &frameSeq) const;

signals:
    void inferenceDone(int frameSeq);
    void statusMessage(const QString &message);

public slots:
    void startInference();
    void stopInference();
    void resetSceneState();

private:
    void processNextFrame();
    FrameBuffer *m_frameBuffer;
    uint8_t *m_rgb565;
    uint8_t *m_rgb888;
    YoloV8Detector m_detector;
    bool m_running;
    int m_lastProcessedSeq;
    mutable QMutex m_resultMutex;
    YoloDetectResult m_result;
    int m_resultFrameSeq;
    bool m_hasResult;
};

#endif
