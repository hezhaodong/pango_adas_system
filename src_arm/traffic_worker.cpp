#include "traffic_worker.h"
#include "image_converter.h"

#include <QMutexLocker>
#include <QTimer>

TrafficWorker::TrafficWorker(QObject *parent)
    : QObject(parent), m_frameBuffer(nullptr), m_rgb565(nullptr),
      m_rgb888(nullptr), m_running(false), m_lastProcessedSeq(-1),
      m_resultFrameSeq(-1), m_hasResult(false)
{
}

TrafficWorker::~TrafficWorker()
{
    stopInference();
}

bool TrafficWorker::getLatestResult(YoloDetectResult &result, int &frameSeq) const
{
    QMutexLocker lock(&m_resultMutex);
    if (!m_hasResult) return false;
    result = m_result;
    frameSeq = m_resultFrameSeq;
    return true;
}

void TrafficWorker::startInference()
{
    if (m_running || !m_detector.isLoaded() || !m_frameBuffer) return;
    if (!m_rgb565) m_rgb565 = new uint8_t[FRAME_SIZE_RGB565];
    if (!m_rgb888) m_rgb888 = new uint8_t[FRAME_WIDTH * FRAME_HEIGHT * 3];
    resetSceneState();
    m_running = true;
    emit statusMessage(QStringLiteral("Traffic detection ON [5]"));
    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}

void TrafficWorker::stopInference()
{
    m_running = false;
    m_detector.release();
    delete[] m_rgb565; m_rgb565 = nullptr;
    delete[] m_rgb888; m_rgb888 = nullptr;
    resetSceneState();
    emit statusMessage(QStringLiteral("Traffic detection OFF [5]"));
}

void TrafficWorker::resetSceneState()
{
    m_lastProcessedSeq = -1;
    QMutexLocker lock(&m_resultMutex);
    m_result = YoloDetectResult();
    m_resultFrameSeq = -1;
    m_hasResult = false;
}

void TrafficWorker::processNextFrame()
{
    if (!m_running || !m_detector.isLoaded() || !m_frameBuffer) return;
    int frameSeq = -1;
    if (!m_frameBuffer->copyLatestFrame(m_rgb565, frameSeq) ||
        frameSeq == m_lastProcessedSeq) {
        QTimer::singleShot(2, this, [this]() { processNextFrame(); });
        return;
    }
    m_lastProcessedSeq = frameSeq;
    ImageConverter::rgb565ToRGB888Fast(m_rgb565, m_rgb888,
                                       FRAME_WIDTH, FRAME_HEIGHT);
    YoloDetectResult result;
    m_detector.detect(m_rgb888, FRAME_WIDTH, FRAME_HEIGHT, result);
    {
        QMutexLocker lock(&m_resultMutex);
        m_result = result;
        m_resultFrameSeq = frameSeq;
        m_hasResult = true;
    }
    emit inferenceDone(frameSeq);
    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}
