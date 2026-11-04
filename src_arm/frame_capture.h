#ifndef FRAME_CAPTURE_H
#define FRAME_CAPTURE_H

#include <QThread>
#include <QElapsedTimer>
#include <atomic>

#include "pcie_device.h"
#include "frame_buffer.h"

// FPGA 使用 4 缓冲乒乓机制，驱动会按索引写入 buffer+idx*size
#define RAW_BUF_COUNT   4
#define RAW_BUF_SIZE    (FRAME_SIZE_RGB565 * RAW_BUF_COUNT)   // 4帧 = 7,372,800 字节

class FrameCapture : public QThread
{
    Q_OBJECT

public:
    explicit FrameCapture(PcieDevice *dev, FrameBuffer *fb, QObject *parent = nullptr);
    ~FrameCapture();

    void    stopCapture();
    bool    isRunning() const { return m_running.load(std::memory_order_acquire); }
    double       inputFPS() const { return m_inputFPS; }
    double       processFPS() const { return m_processFPS; }
    unsigned int totalFrames() const { return m_totalFrames; }

signals:
    // Monotonic timestamp lets the GUI distinguish queued-signal backlog
    // from FPGA/capture and DRM/display stalls.
    void frameReady(int frameSeq, qint64 captureReadyUs);
    void captureError(const QString &msg);
    void frameTimeout();
    void frameResumed();
    void statsUpdated(unsigned int inputFrames,
                      unsigned int processedFrames,
                      unsigned int dropped,
                      double inputFps,
                      double processFps);

protected:
    void run() override;

private:
    PcieDevice  *m_device;
    FrameBuffer *m_buffer;

    // 4 帧原始缓冲区 (匹配 FPGA 4 槽乒乓)
    uint8_t     *m_rawBuf4;

    std::atomic_bool m_running;
    std::atomic_bool m_stopRequested;

    unsigned int m_totalFrames;
    unsigned int m_droppedFrames;
    unsigned int m_frameSeq;
    int          m_lastIndex;
    bool         m_timeout;

    QElapsedTimer m_fpsTimer;
    unsigned int  m_inputFrameCount;
    unsigned int  m_processFrameCount;
    double        m_inputFPS;
    double        m_processFPS;
};

#endif // FRAME_CAPTURE_H
