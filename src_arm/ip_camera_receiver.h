#ifndef IP_CAMERA_RECEIVER_H
#define IP_CAMERA_RECEIVER_H

#include <QElapsedTimer>
#include <QMutex>
#include <QThread>
#include <QUrl>

#include <atomic>

#include "frame_buffer.h"

// Pulls an HTTP MJPEG stream produced by a phone IP-camera application.
// JPEG frames are decoded to the same RGB565 FrameBuffer used by PCIe/WiFi.
class IpCameraReceiver : public QThread
{
    Q_OBJECT

public:
    explicit IpCameraReceiver(FrameBuffer *buffer, QObject *parent = nullptr);
    ~IpCameraReceiver() override;

    bool setUrl(const QString &url, QString *errorMessage = nullptr);
    QString url() const;
    bool hasUrl() const;
    bool isConnected() const { return m_connected.load(std::memory_order_acquire); }
    void stop();

signals:
    void frameReady(int frameSeq);
    void connected();
    void disconnected();
    void connectionError(const QString &message);
    void statsUpdated(unsigned int frames, double fps, double kbps);

protected:
    void run() override;

private:
    int connectHttp(const QUrl &url, QByteArray &initialBody, QString &errorMessage);
    bool processJpeg(const char *data, int size);
    static void bgrToRgb565(const uint8_t *src, uint8_t *dst, int pixelCount);

    FrameBuffer *m_buffer;
    mutable QMutex m_configMutex;
    QUrl m_url;
    std::atomic_bool m_stopRequested;
    std::atomic_bool m_connected;
    std::atomic_int m_socketFd;
    unsigned int m_totalFrames;
    int m_frameSeq;
};

#endif // IP_CAMERA_RECEIVER_H
