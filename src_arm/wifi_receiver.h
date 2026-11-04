#ifndef WIFI_RECEIVER_H
#define WIFI_RECEIVER_H

#include <QThread>
#include <QString>
#include <QMutex>
#include <QElapsedTimer>
#include <atomic>
#include <cstdint>

#include "frame_buffer.h"

// 最大 JPEG 帧缓冲: 720p JPEG 通常 < 200KB, 留 512KB 安全余量
#define MAX_JPEG_SIZE (512 * 1024)

// ============================================================================
// WifiReceiver — TCP MJPEG 接收线程
// 与 FrameCapture 平行的 QThread，从 Windows 发送器接收 MJPEG 帧
// 解码后写入共享的 FrameBuffer
//
// 帧协议: | 4 bytes: jpeg_size (uint32 LE) | N bytes: JPEG data |
// ============================================================================
class WifiReceiver : public QThread
{
    Q_OBJECT

public:
    explicit WifiReceiver(FrameBuffer *fb, QObject *parent = nullptr);
    ~WifiReceiver() override;

    // 连接控制
    void connectToHost(const QString &host, quint16 port);
    void stop();

    // 状态
    bool isConnected()   const { return m_connected.load(std::memory_order_acquire); }
    bool isRunning()     const { return m_running.load(std::memory_order_acquire); }
    QString sourceName() const { return QStringLiteral("WiFi"); }

    // 统计 (与 FrameCapture 接口对齐)
    double       inputFPS()     const { return m_fps; }
    unsigned int totalFrames()  const { return m_totalFrames; }

signals:
    void frameReady(int frameSeq);
    void connected();
    void disconnected();
    void connectionError(const QString &msg);
    void statsUpdated(unsigned int frames, double fps, double kbps);

protected:
    void run() override;

private:
    // 从 socket 精确读取 n 字节 (阻塞, 超时返回 false)
    bool readExact(int sockFd, uint8_t *buf, size_t len, int timeoutMs = 3000);

    // 处理一帧: JPEG 解码 → RGB565 → FrameBuffer
    bool processJpegFrame(const uint8_t *jpegData, size_t jpegSize);

    // BGR → RGB565 转换 (小端序)
    static void bgrToRgb565(const uint8_t *src, uint8_t *dst,
                            int width, int height);

    FrameBuffer  *m_buffer;

    // 连接参数
    QString       m_host;
    quint16       m_port;

    // 线程控制
    std::atomic_bool m_running;
    std::atomic_bool m_stopRequested;
    std::atomic_bool m_connected;

    // 帧统计
    unsigned int  m_totalFrames;
    unsigned int  m_frameSeq;
    double        m_fps;
    QElapsedTimer m_fpsTimer;
    unsigned int  m_fpsFrameCount;
    unsigned int  m_bytesReceived;
    double        m_kbps;

    // 接收缓冲
    uint8_t      *m_jpegBuf;
    uint8_t      *m_rgb888Buf;   // cv::imdecode 输出

    // 线程安全
    QMutex        m_mutex;       // 保护 m_host / m_port
};

#endif // WIFI_RECEIVER_H
