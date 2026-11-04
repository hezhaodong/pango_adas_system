#include "wifi_receiver.h"

#include <cstdio>
#include <cstring>

#include <QMutex>
#include <QMutexLocker>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <QDebug>

// ============================================================================
// 构造 / 析构
// ============================================================================

WifiReceiver::WifiReceiver(FrameBuffer *fb, QObject *parent)
    : QThread(parent)
    , m_buffer(fb)
    , m_port(0)
    , m_running(false)
    , m_stopRequested(false)
    , m_connected(false)
    , m_totalFrames(0)
    , m_frameSeq(0)
    , m_fps(0.0)
    , m_fpsFrameCount(0)
    , m_bytesReceived(0)
    , m_kbps(0.0)
    , m_jpegBuf(nullptr)
    , m_rgb888Buf(nullptr)
{
    m_jpegBuf   = new uint8_t[MAX_JPEG_SIZE];
    m_rgb888Buf = new uint8_t[FRAME_WIDTH * FRAME_HEIGHT * 3];
}

WifiReceiver::~WifiReceiver()
{
    stop();
    delete[] m_jpegBuf;
    delete[] m_rgb888Buf;
}

// ============================================================================
// 公共接口
// ============================================================================

void WifiReceiver::connectToHost(const QString &host, quint16 port)
{
    QMutexLocker lock(&m_mutex);
    m_host = host;
    m_port = port;
}

void WifiReceiver::stop()
{
    m_stopRequested.store(true, std::memory_order_release);
    // 等待线程退出 (最多等 3 秒)
    if (isRunning()) {
        wait(3000);
    }
}

// ============================================================================
// run() — 主循环 (在工作线程中执行)
// ============================================================================

void WifiReceiver::run()
{
    m_running.store(true, std::memory_order_release);
    m_stopRequested.store(false, std::memory_order_release);
    m_connected.store(false, std::memory_order_release);
    m_fps = 0.0;
    m_kbps = 0.0;
    m_totalFrames = 0;
    m_frameSeq = 0;
    m_fpsFrameCount = 0;
    m_bytesReceived = 0;

    m_fpsTimer.start();

    while (!m_stopRequested.load(std::memory_order_acquire)) {

        // ---- 1. 等待发送器地址 (线程安全读取) ----
        QString host;
        quint16 port;
        {
            QMutexLocker lock(&m_mutex);
            host = m_host;
            port = m_port;
        }
        if (host.isEmpty() || port == 0) {
            msleep(500);
            continue;
        }

        int sockFd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sockFd < 0) {
            qWarning() << "[WifiReceiver] socket 创建失败:" << strerror(errno);
            msleep(1000);
            continue;
        }

        // TCP_NODELAY 减少延迟
        int opt = 1;
        ::setsockopt(sockFd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

        // 设置接收超时 3 秒
        struct timeval tv;
        tv.tv_sec = 3;
        tv.tv_usec = 0;
        ::setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        if (::inet_pton(AF_INET, host.toStdString().c_str(),
                        &addr.sin_addr) != 1) {
            qWarning() << "[WifiReceiver] 地址解析失败:" << host;
            ::close(sockFd);
            msleep(1000);
            continue;
        }

        if (::connect(sockFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            qWarning() << "[WifiReceiver] connect 失败:" << strerror(errno);
            ::close(sockFd);
            emit connectionError(QString("连接失败: %1").arg(strerror(errno)));
            msleep(1000);
            continue;
        }

        m_connected.store(true, std::memory_order_release);
        emit connected();

        // ---- 2. 帧接收循环 ----
        while (!m_stopRequested.load(std::memory_order_acquire)
               && m_connected.load(std::memory_order_acquire)) {
            // a. 读取 4 字节长度头 (uint32 LE)
            uint8_t header[4];
            if (!readExact(sockFd, header, 4, 3000)) {
                break;  // 断开, 进入重连
            }
            uint32_t jpegSize = header[0]
                              | (uint32_t(header[1]) << 8)
                              | (uint32_t(header[2]) << 16)
                              | (uint32_t(header[3]) << 24);

            if (jpegSize == 0 || jpegSize > MAX_JPEG_SIZE) {
                qWarning() << "[WifiReceiver] 非法帧大小:" << jpegSize;
                continue;
            }

            // b. 读取 JPEG 数据
            if (!readExact(sockFd, m_jpegBuf, jpegSize, 3000)) {
                break;  // 断开, 进入重连
            }

            m_bytesReceived += jpegSize + 4;

            // c. 解码并写入 FrameBuffer
            if (processJpegFrame(m_jpegBuf, jpegSize)) {
                ++m_totalFrames;
                ++m_fpsFrameCount;
                emit frameReady(m_frameSeq++);
            }

            // d. 每秒统计
            if (m_fpsTimer.elapsed() >= 1000) {
                m_fps = m_fpsFrameCount * 1000.0 / m_fpsTimer.elapsed();
                m_kbps = m_bytesReceived * 8.0 / m_fpsTimer.elapsed();
                emit statsUpdated(m_totalFrames, m_fps, m_kbps);
                m_fpsFrameCount = 0;
                m_bytesReceived = 0;
                m_fpsTimer.restart();
            }
        }

        // ---- 3. 断开处理 ----
        ::close(sockFd);
        m_connected.store(false, std::memory_order_release);
        emit disconnected();

        if (!m_stopRequested.load(std::memory_order_acquire)) {
            msleep(1000);  // 重连间隔
        }
    }

    m_running.store(false, std::memory_order_release);
}

// ============================================================================
// 精确读取 n 字节
// ============================================================================

bool WifiReceiver::readExact(int sockFd, uint8_t *buf, size_t len, int timeoutMs)
{
    size_t received = 0;
    while (received < len) {
        ssize_t n = ::recv(sockFd, buf + received, len - received, 0);
        if (n <= 0) {
            if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
                return false;  // 连接关闭或错误
            }
            // EAGAIN/EINTR: 超时或信号中断, 继续尝试
            if (m_stopRequested.load(std::memory_order_acquire)) return false;
            continue;
        }
        received += n;
    }
    return true;
}

// ============================================================================
// 处理一帧: JPEG 解码 → RGB565 → FrameBuffer
// ============================================================================

bool WifiReceiver::processJpegFrame(const uint8_t *jpegData, size_t jpegSize)
{
    // 1. OpenCV JPEG → BGR
    cv::Mat jpegMat(1, jpegSize, CV_8UC1, const_cast<uint8_t *>(jpegData));
    cv::Mat srcMat = cv::imdecode(jpegMat, cv::IMREAD_COLOR);
    if (srcMat.empty())
        return false;

    // 2. 缩放 (如需要)
    if (srcMat.cols != FRAME_WIDTH || srcMat.rows != FRAME_HEIGHT) {
        cv::Mat tmp;
        cv::resize(srcMat, tmp, cv::Size(FRAME_WIDTH, FRAME_HEIGHT),
                   0, 0, cv::INTER_LINEAR);
        srcMat = tmp;
    }

    // 3. BGR 直转 RGB565 (省掉 BGR→RGB 这一步)
    uint8_t *dst = m_buffer->lockWriteBuffer();
    bgrToRgb565(srcMat.data, dst, FRAME_WIDTH, FRAME_HEIGHT);
    m_buffer->commitFrame(m_frameSeq);

    return true;
}

// ============================================================================
// BGR → RGB565 快速转换 (查表, 比逐像素位运算快 3-5 倍)
// 输入: B,G,R,B,G,R,... (OpenCV cv::imdecode 原生格式)
// 输出: uint16 LE RGB565
// ============================================================================

static uint16_t s_hiTable[256][256];  // [R5][G6] → RGB565 高字节部分

static void initLookupTable()
{
    static bool ready = false;
    if (ready) return;
    for (int r5 = 0; r5 < 32; ++r5) {
        for (int g6 = 0; g6 < 64; ++g6) {
            uint16_t val = (r5 << 11) | (g6 << 5);
            s_hiTable[r5][g6] = val;
        }
    }
    ready = true;
}

void WifiReceiver::bgrToRgb565(const uint8_t *src, uint8_t *dst,
                                int width, int height)
{
    initLookupTable();

    const int pixelCount = width * height;
    const uint8_t *s = src;
    uint8_t *d = dst;

    for (int i = 0; i < pixelCount; ++i) {
        uint8_t b = s[0];  // BGR 顺序
        uint8_t g = s[1];
        uint8_t r = s[2];
        s += 3;

        uint16_t val = s_hiTable[r >> 3][g >> 2] | (b >> 3);

        d[0] = val & 0xFF;
        d[1] = (val >> 8) & 0xFF;
        d += 2;
    }
}
