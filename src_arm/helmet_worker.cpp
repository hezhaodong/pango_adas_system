#include "helmet_worker.h"
#include "image_converter.h"

#include <QMutexLocker>
#include <QTimer>
#include <algorithm>

HelmetWorker::HelmetWorker(QObject *parent)
    : QObject(parent), m_frameBuffer(nullptr), m_rgb565(nullptr),
      m_rgb888(nullptr), m_running(false), m_lastProcessedSeq(-1),
      m_resultFrameSeq(-1), m_hasResult(false)
{
}

HelmetWorker::~HelmetWorker()
{
    stopInference();
}

bool HelmetWorker::getLatestResult(YoloDetectResult &result, int &frameSeq) const
{
    QMutexLocker lock(&m_resultMutex);
    if (!m_hasResult) return false;
    result = m_result;
    frameSeq = m_resultFrameSeq;
    return true;
}

void HelmetWorker::startInference()
{
    if (m_running || !m_detector.isLoaded() || !m_frameBuffer) return;
    if (!m_rgb565) m_rgb565 = new uint8_t[FRAME_SIZE_RGB565];
    if (!m_rgb888) m_rgb888 = new uint8_t[FRAME_WIDTH * FRAME_HEIGHT * 3];
    resetSceneState();
    m_running = true;
    emit statusMessage(QStringLiteral("Helmet detection ON [4]"));
    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}

void HelmetWorker::stopInference()
{
    m_running = false;
    m_detector.release();
    delete[] m_rgb565; m_rgb565 = nullptr;
    delete[] m_rgb888; m_rgb888 = nullptr;
    resetSceneState();
    emit statusMessage(QStringLiteral("Helmet detection OFF [4]"));
}

void HelmetWorker::resetSceneState()
{
    m_lastProcessedSeq = -1;
    QMutexLocker lock(&m_resultMutex);
    m_result = YoloDetectResult();
    m_resultFrameSeq = -1;
    m_hasResult = false;
}

void HelmetWorker::processNextFrame()
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
    result.boxes.erase(
        std::remove_if(result.boxes.begin(), result.boxes.end(),
                       [](const YoloBox &box) {
                           return box.classId == 2 && box.confidence < 0.40f;
                       }),
        result.boxes.end());
    {
        QMutexLocker lock(&m_resultMutex);
        m_result = result;
        m_resultFrameSeq = frameSeq;
        m_hasResult = true;
    }
    emit inferenceDone(frameSeq);
    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}
