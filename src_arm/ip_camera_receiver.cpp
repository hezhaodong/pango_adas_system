#include "ip_camera_receiver.h"

#include <QByteArray>
#include <QMutexLocker>
#include <QDebug>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
constexpr int kConnectTimeoutMs = 3000;
constexpr int kReadTimeoutMs = 3000;
constexpr int kMaxHeaderSize = 64 * 1024;
constexpr int kMaxJpegSize = 4 * 1024 * 1024;

QString socketError(const char *prefix)
{
    return QStringLiteral("%1: %2").arg(QString::fromUtf8(prefix),
                                        QString::fromLocal8Bit(std::strerror(errno)));
}
}

IpCameraReceiver::IpCameraReceiver(FrameBuffer *buffer, QObject *parent)
    : QThread(parent)
    , m_buffer(buffer)
    , m_stopRequested(false)
    , m_connected(false)
    , m_socketFd(-1)
    , m_totalFrames(0)
    , m_frameSeq(0)
{
}

IpCameraReceiver::~IpCameraReceiver()
{
    stop();
}

bool IpCameraReceiver::setUrl(const QString &text, QString *errorMessage)
{
    const QUrl parsed = QUrl::fromUserInput(text.trimmed());
    QString error;
    if (!parsed.isValid() || parsed.host().isEmpty())
        error = QStringLiteral("无效的 IP 摄像头 URL");
    else if (parsed.scheme().compare(QStringLiteral("http"), Qt::CaseInsensitive) != 0)
        error = QStringLiteral("初版仅支持 http:// MJPEG 地址");

    if (!error.isEmpty()) {
        if (errorMessage) *errorMessage = error;
        return false;
    }

    QMutexLocker lock(&m_configMutex);
    m_url = parsed;
    return true;
}

QString IpCameraReceiver::url() const
{
    QMutexLocker lock(&m_configMutex);
    return m_url.toString(QUrl::RemovePassword);
}

bool IpCameraReceiver::hasUrl() const
{
    QMutexLocker lock(&m_configMutex);
    return m_url.isValid() && !m_url.host().isEmpty();
}

void IpCameraReceiver::stop()
{
    m_stopRequested.store(true, std::memory_order_release);
    const int fd = m_socketFd.load(std::memory_order_acquire);
    if (fd >= 0) ::shutdown(fd, SHUT_RDWR);
    if (QThread::isRunning()) wait(3500);
}

int IpCameraReceiver::connectHttp(const QUrl &url, QByteArray &initialBody,
                                  QString &errorMessage)
{
    const QByteArray host = url.host().toUtf8();
    const int port = url.port(80);
    QByteArray portText = QByteArray::number(port);

    addrinfo hints {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo *addresses = nullptr;
    const int gai = ::getaddrinfo(host.constData(), portText.constData(), &hints, &addresses);
    if (gai != 0) {
        errorMessage = QStringLiteral("域名/IP解析失败: %1")
                           .arg(QString::fromLocal8Bit(gai_strerror(gai)));
        return -1;
    }

    int fd = -1;
    for (addrinfo *address = addresses; address && fd < 0; address = address->ai_next) {
        const int candidate = ::socket(address->ai_family, address->ai_socktype,
                                       address->ai_protocol);
        if (candidate < 0) continue;
        const int oldFlags = ::fcntl(candidate, F_GETFL, 0);
        ::fcntl(candidate, F_SETFL, oldFlags | O_NONBLOCK);
        int result = ::connect(candidate, address->ai_addr, address->ai_addrlen);
        if (result < 0 && errno == EINPROGRESS) {
            fd_set writes;
            FD_ZERO(&writes);
            FD_SET(candidate, &writes);
            timeval timeout { kConnectTimeoutMs / 1000,
                              (kConnectTimeoutMs % 1000) * 1000 };
            result = ::select(candidate + 1, nullptr, &writes, nullptr, &timeout);
            if (result > 0) {
                int connectError = 0;
                socklen_t length = sizeof(connectError);
                ::getsockopt(candidate, SOL_SOCKET, SO_ERROR, &connectError, &length);
                if (connectError != 0) {
                    errno = connectError;
                    result = -1;
                } else {
                    result = 0;
                }
            } else {
                if (result == 0) errno = ETIMEDOUT;
                result = -1;
            }
        }
        if (result == 0) {
            ::fcntl(candidate, F_SETFL, oldFlags);
            fd = candidate;
        } else {
            ::close(candidate);
        }
    }
    ::freeaddrinfo(addresses);
    if (fd < 0) {
        errorMessage = socketError("连接失败");
        return -1;
    }

    m_socketFd.store(fd, std::memory_order_release);
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    timeval readTimeout { kReadTimeoutMs / 1000, (kReadTimeoutMs % 1000) * 1000 };
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &readTimeout, sizeof(readTimeout));

    QByteArray path = url.path(QUrl::FullyEncoded).toUtf8();
    if (path.isEmpty()) path = "/";
    if (url.hasQuery()) {
        path.append('?');
        path.append(url.query(QUrl::FullyEncoded).toUtf8());
    }
    QByteArray request = "GET " + path + " HTTP/1.1\r\nHost: " + host;
    if (port != 80) {
        request.append(':');
        request.append(portText);
    }
    request += "\r\nAccept: multipart/x-mixed-replace,image/jpeg,*/*\r\n"
               "Connection: keep-alive\r\nUser-Agent: SAI-RK3568/1.0\r\n";
    if (!url.userName().isEmpty()) {
        const QByteArray credentials =
            (url.userName(QUrl::FullyDecoded) + ':' + url.password(QUrl::FullyDecoded)).toUtf8();
        request += "Authorization: Basic " + credentials.toBase64() + "\r\n";
    }
    request += "\r\n";

    int sent = 0;
    while (sent < request.size()) {
        const ssize_t count = ::send(fd, request.constData() + sent,
                                     size_t(request.size() - sent), MSG_NOSIGNAL);
        if (count <= 0) {
            errorMessage = socketError("HTTP请求发送失败");
            ::close(fd);
            m_socketFd.store(-1, std::memory_order_release);
            return -1;
        }
        sent += int(count);
    }

    QByteArray response;
    char chunk[8192];
    int headerEnd = -1;
    while (response.size() < kMaxHeaderSize && !m_stopRequested.load()) {
        const ssize_t count = ::recv(fd, chunk, sizeof(chunk), 0);
        if (count <= 0) break;
        response.append(chunk, int(count));
        headerEnd = response.indexOf("\r\n\r\n");
        if (headerEnd >= 0) break;
    }
    if (headerEnd < 0) {
        errorMessage = QStringLiteral("未收到有效的 HTTP 响应头");
        ::close(fd);
        m_socketFd.store(-1, std::memory_order_release);
        return -1;
    }

    const QByteArray header = response.left(headerEnd);
    const int firstLineEnd = header.indexOf("\r\n");
    const QByteArray statusLine = firstLineEnd >= 0 ? header.left(firstLineEnd) : header;
    if (!statusLine.contains(" 200 ")) {
        errorMessage = QStringLiteral("IP摄像头返回: %1")
                           .arg(QString::fromLatin1(statusLine));
        ::close(fd);
        m_socketFd.store(-1, std::memory_order_release);
        return -1;
    }
    initialBody = response.mid(headerEnd + 4);
    return fd;
}

bool IpCameraReceiver::processJpeg(const char *data, int size)
{
    cv::Mat encoded(1, size, CV_8UC1, const_cast<char *>(data));
    cv::Mat frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (frame.empty()) return false;
    if (frame.cols != FRAME_WIDTH || frame.rows != FRAME_HEIGHT) {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT),
                   0, 0, cv::INTER_LINEAR);
        frame = resized;
    }
    uint8_t *destination = m_buffer->lockWriteBuffer();
    bgrToRgb565(frame.data, destination, FRAME_WIDTH * FRAME_HEIGHT);
    m_buffer->commitFrame(m_frameSeq);
    return true;
}

void IpCameraReceiver::bgrToRgb565(const uint8_t *source, uint8_t *destination,
                                   int pixelCount)
{
    for (int i = 0; i < pixelCount; ++i) {
        const uint8_t b = source[0];
        const uint8_t g = source[1];
        const uint8_t r = source[2];
        source += 3;
        const uint16_t value = uint16_t((r >> 3) << 11)
                             | uint16_t((g >> 2) << 5)
                             | uint16_t(b >> 3);
        destination[0] = uint8_t(value & 0xff);
        destination[1] = uint8_t(value >> 8);
        destination += 2;
    }
}

void IpCameraReceiver::run()
{
    m_stopRequested.store(false, std::memory_order_release);
    m_totalFrames = 0;
    m_frameSeq = 0;

    while (!m_stopRequested.load(std::memory_order_acquire)) {
        QUrl currentUrl;
        {
            QMutexLocker lock(&m_configMutex);
            currentUrl = m_url;
        }
        if (!currentUrl.isValid() || currentUrl.host().isEmpty()) {
            emit connectionError(QStringLiteral("未设置 IP 摄像头 URL"));
            break;
        }

        QByteArray stream;
        QString error;
        const int fd = connectHttp(currentUrl, stream, error);
        if (fd < 0) {
            if (!m_stopRequested.load()) emit connectionError(error);
            for (int i = 0; i < 10 && !m_stopRequested.load(); ++i) msleep(100);
            continue;
        }

        m_connected.store(true, std::memory_order_release);
        emit connected();
        QElapsedTimer statsTimer;
        statsTimer.start();
        unsigned int intervalFrames = 0;
        unsigned int intervalBytes = 0;
        char receiveBuffer[64 * 1024];

        while (!m_stopRequested.load(std::memory_order_acquire)) {
            int start = stream.indexOf(QByteArray("\xff\xd8", 2));
            int end = start >= 0 ? stream.indexOf(QByteArray("\xff\xd9", 2), start + 2) : -1;
            if (start >= 0 && end >= 0) {
                const int jpegSize = end + 2 - start;
                if (jpegSize <= kMaxJpegSize && processJpeg(stream.constData() + start, jpegSize)) {
                    ++m_totalFrames;
                    ++intervalFrames;
                    emit frameReady(m_frameSeq++);
                }
                stream.remove(0, end + 2);
                continue;
            }
            if (start > 0) stream.remove(0, start);
            else if (start < 0 && stream.size() > 1) stream = stream.right(1);
            else if (stream.size() > kMaxJpegSize) stream.clear();

            const ssize_t count = ::recv(fd, receiveBuffer, sizeof(receiveBuffer), 0);
            if (count <= 0) break;
            stream.append(receiveBuffer, int(count));
            intervalBytes += unsigned(count);

            if (statsTimer.elapsed() >= 1000) {
                const double elapsed = statsTimer.elapsed();
                emit statsUpdated(m_totalFrames,
                                  intervalFrames * 1000.0 / elapsed,
                                  intervalBytes * 8.0 / elapsed);
                intervalFrames = 0;
                intervalBytes = 0;
                statsTimer.restart();
            }
        }

        ::close(fd);
        m_socketFd.store(-1, std::memory_order_release);
        const bool wasConnected = m_connected.exchange(false, std::memory_order_acq_rel);
        if (wasConnected) emit disconnected();
        for (int i = 0; i < 10 && !m_stopRequested.load(); ++i) msleep(100);
    }

    m_connected.store(false, std::memory_order_release);
    m_socketFd.store(-1, std::memory_order_release);
}
