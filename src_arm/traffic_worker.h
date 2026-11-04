#ifndef TRAFFIC_WORKER_H
#define TRAFFIC_WORKER_H

#include "frame_buffer.h"
#include "traffic_detector.h"

#include <QMutex>
#include <QObject>

class TrafficWorker : public QObject
{
    Q_OBJECT
public:
    explicit TrafficWorker(QObject *parent = nullptr);
    ~TrafficWorker();
    void setFrameBuffer(FrameBuffer *buffer) { m_frameBuffer = buffer; }
    bool isLoaded() const { return m_detector.isLoaded(); }
    bool loadModel(const char *modelPath) { return m_detector.loadModel(modelPath); }
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
    TrafficDetector m_detector;
    bool m_running;
    int m_lastProcessedSeq;
    mutable QMutex m_resultMutex;
    YoloDetectResult m_result;
    int m_resultFrameSeq;
    bool m_hasResult;
};

#endif // TRAFFIC_WORKER_H
