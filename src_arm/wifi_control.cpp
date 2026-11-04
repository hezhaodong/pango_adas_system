#include "wifi_control.h"

#include <QNetworkDatagram>
#include <QDebug>
#include <QStringList>
#include <QDataStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTcpSocket>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <vector>

WifiControl::WifiControl(QObject *parent)
    : QObject(parent)
    , m_socket(nullptr)
    , m_resultPort(0)
    , m_imagePort(0)
    , m_imageSenderRunning(false)
{
}

WifiControl::~WifiControl()
{
    stop();
}

bool WifiControl::start()
{
    m_socket = new QUdpSocket(this);
    if (!m_socket->bind(QHostAddress::AnyIPv4, CONTROL_PORT,
                        QAbstractSocket::ShareAddress)) {
        qWarning() << "[WifiCtrl] 绑定 UDP:" << CONTROL_PORT
                   << "失败:" << m_socket->errorString();
        delete m_socket;
        m_socket = nullptr;
        return false;
    }

    connect(m_socket, &QUdpSocket::readyRead,
            this, &WifiControl::onReadyRead);

    m_imageSenderRunning = true;
    m_imageSenderThread = std::thread(&WifiControl::imageSenderLoop, this);
    return true;
}

void WifiControl::stop()
{
    m_imageSenderRunning = false;
    m_imageCondition.notify_all();
    if (m_imageSenderThread.joinable())
        m_imageSenderThread.join();
    {
        std::lock_guard<std::mutex> lock(m_imageMutex);
        m_imageQueue.clear();
    }

    if (m_socket) {
        m_socket->close();
        delete m_socket;
        m_socket = nullptr;
    }
}

bool WifiControl::enqueuePlateImage(const cv::Mat &plateBgr, int frameSeq,
                                    int trackId, const QString &plateText)
{
    if (plateBgr.empty() || !m_imageSenderRunning)
        return false;

    std::lock_guard<std::mutex> lock(m_imageMutex);
    if (m_imageHost.isEmpty() || m_imagePort == 0)
        return false;
    if (m_imageQueue.size() >= MAX_IMAGE_QUEUE)
        m_imageQueue.pop_front();

    PlateImageTask task;
    task.image = plateBgr.clone();
    task.frameSeq = frameSeq;
    task.trackId = trackId;
    task.plateText = plateText;
    m_imageQueue.push_back(std::move(task));
    m_imageCondition.notify_one();
    return true;
}

void WifiControl::sendRecognitionResult(const QByteArray &jsonPayload)
{
    if (!m_socket || m_resultHost.isNull() || m_resultPort == 0 || jsonPayload.isEmpty())
        return;

    QByteArray packet("SAI_RESULT ");
    packet.append(jsonPayload);
    const qint64 sent = m_socket->writeDatagram(packet, m_resultHost, m_resultPort);
    if (sent < 0)
        qWarning() << "[WifiCtrl] result send failed:" << m_socket->errorString();
}

void WifiControl::imageSenderLoop()
{
    QTcpSocket socket;
    QString connectedHost;
    quint16 connectedPort = 0;

    while (m_imageSenderRunning) {
        PlateImageTask task;
        QString targetHost;
        quint16 targetPort = 0;
        {
            std::unique_lock<std::mutex> lock(m_imageMutex);
            m_imageCondition.wait(lock, [this]() {
                return !m_imageSenderRunning ||
                       (!m_imageQueue.empty() && !m_imageHost.isEmpty() && m_imagePort != 0);
            });
            if (!m_imageSenderRunning)
                break;
            task = std::move(m_imageQueue.front());
            m_imageQueue.pop_front();
            targetHost = m_imageHost;
            targetPort = m_imagePort;
        }

        cv::Mat output = task.image;
        if (output.cols > 320) {
            const double scale = 320.0 / static_cast<double>(output.cols);
            cv::resize(output, output,
                       cv::Size(320, std::max(1, static_cast<int>(output.rows * scale))),
                       0.0, 0.0, cv::INTER_AREA);
        }

        std::vector<uchar> encoded;
        const std::vector<int> encodeParams = {cv::IMWRITE_JPEG_QUALITY, 78};
        if (!cv::imencode(".jpg", output, encoded, encodeParams)) {
            qWarning() << "[WifiCtrl] plate JPEG encode failed";
            continue;
        }

        QJsonObject metadata;
        metadata.insert(QStringLiteral("type"), QStringLiteral("plate_image"));
        metadata.insert(QStringLiteral("frame_seq"), task.frameSeq);
        metadata.insert(QStringLiteral("track_id"), task.trackId);
        metadata.insert(QStringLiteral("plate"), task.plateText);
        metadata.insert(QStringLiteral("width"), output.cols);
        metadata.insert(QStringLiteral("height"), output.rows);
        const QByteArray metadataBytes =
            QJsonDocument(metadata).toJson(QJsonDocument::Compact);
        const QByteArray jpegBytes(reinterpret_cast<const char *>(encoded.data()),
                                   static_cast<int>(encoded.size()));

        QByteArray packet;
        {
            QDataStream stream(&packet, QIODevice::WriteOnly);
            stream.setByteOrder(QDataStream::LittleEndian);
            stream.writeRawData("SPIM", 4);
            stream << static_cast<quint32>(metadataBytes.size());
            stream << static_cast<quint32>(jpegBytes.size());
        }
        packet.append(metadataBytes);
        packet.append(jpegBytes);

        bool sent = false;
        for (int attempt = 0; attempt < 3 && m_imageSenderRunning; ++attempt) {
            if (socket.state() != QAbstractSocket::ConnectedState ||
                connectedHost != targetHost || connectedPort != targetPort) {
                socket.abort();
                socket.connectToHost(targetHost, targetPort);
                if (!socket.waitForConnected(700))
                    continue;
                connectedHost = targetHost;
                connectedPort = targetPort;
            }

            const qint64 queued = socket.write(packet);
            if (queued == packet.size() &&
                (socket.bytesToWrite() == 0 || socket.waitForBytesWritten(1000))) {
                sent = true;
                break;
            }
            socket.abort();
            connectedHost.clear();
            connectedPort = 0;
        }

        if (!sent) {
            qWarning() << "[WifiCtrl] plate image send failed:"
                       << targetHost << targetPort << task.trackId << task.plateText;
        }
    }

    socket.abort();
}

void WifiControl::onReadyRead()
{
    while (m_socket && m_socket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_socket->receiveDatagram();
        QString msg = QString::fromUtf8(datagram.data().trimmed());

        // Control discovery is independent from the active video source.
        // WifiControl is started with the application and remains available
        // while PCIe/FMC/HDMI/WiFi video is displayed.
        if (msg.startsWith("SAI_CTRL_DISCOVER")) {
            const QStringList parts = msg.split(' ', Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                bool ok = false;
                const uint portValue = parts[1].toUInt(&ok);
                if (ok && portValue > 0 && portValue <= 65535) {
                    m_resultHost = datagram.senderAddress();
                    m_resultPort = static_cast<quint16>(portValue);
                }
            }
            if (parts.size() >= 3) {
                bool ok = false;
                const uint portValue = parts[2].toUInt(&ok);
                if (ok && portValue > 0 && portValue <= 65535) {
                    std::lock_guard<std::mutex> lock(m_imageMutex);
                    m_imageHost = datagram.senderAddress().toString();
                    if (m_imageHost.startsWith(QStringLiteral("::ffff:")))
                        m_imageHost = m_imageHost.mid(7);
                    m_imagePort = static_cast<quint16>(portValue);
                    m_imageCondition.notify_one();
                }
            }
            m_socket->writeDatagram(QByteArrayLiteral("SAI_CTRL_HERE"),
                                    datagram.senderAddress(),
                                    datagram.senderPort());
            continue;
        }

        if (!msg.startsWith("SAI_CTRL")) continue;

        // Acknowledge receipt before executing the command. Some operations
        // (model loading, source switching) can block the UI thread briefly.
        const QString command = msg.mid(QStringLiteral("SAI_CTRL").size()).trimmed();
        const QByteArray ack = QStringLiteral("SAI_CTRL_ACK %1")
                                   .arg(command).toUtf8();
        m_socket->writeDatagram(ack, datagram.senderAddress(), datagram.senderPort());

        if (msg == "SAI_CTRL DEVICE_OPEN") {
            emit deviceOpen();
        } else if (msg == "SAI_CTRL DEVICE_CLOSE") {
            emit deviceClose();
        } else if (msg == "SAI_CTRL CAPTURE_START") {
            emit captureStart();
        } else if (msg == "SAI_CTRL CAPTURE_STOP") {
            emit captureStop();
        } else if (msg == "SAI_CTRL AI_ON") {
            emit aiOn();
        } else if (msg == "SAI_CTRL AI_OFF") {
            emit aiOff();
        } else if (msg == "SAI_CTRL YOLO_ON") {
            emit yoloOn();
        } else if (msg == "SAI_CTRL YOLO_OFF") {
            emit yoloOff();
        } else if (msg == "SAI_CTRL SRC_PCIE") {
            emit switchToPcie();
        } else if (msg == "SAI_CTRL SRC_WIFI") {
            emit switchToWifi();
        } else if (msg == "SAI_CTRL SRC_IPCAM") {
            emit switchToIpCamera();
        } else if (command.startsWith(QStringLiteral("IPCAM_URL "))) {
            emit ipCameraUrlChanged(command.mid(QStringLiteral("IPCAM_URL ").size()).trimmed());
        } else if (msg == "SAI_CTRL FPGA_FMC") {
            emit fpgaFmc();
        } else if (msg == "SAI_CTRL FPGA_HDMI") {
            emit fpgaHdmi();
        } else if (msg == "SAI_CTRL FPGA_FIBER") {
            emit fpgaFiber();
        } else if (msg == "SAI_CTRL BRIGHTNESS_AUTO_ON") {
            emit autoBrightnessOn();
        } else if (msg == "SAI_CTRL BRIGHTNESS_AUTO_OFF") {
            emit autoBrightnessOff();
        } else if (msg == "SAI_CTRL SAVE_FRAME") {
            emit saveFrame();
        } else if (msg == "SAI_CTRL KEY_Q") {
            emit toggleInputSource();
        } else if (msg == "SAI_CTRL KEY_H") {
            emit toggleFpgaInputSource();
        } else if (msg == "SAI_CTRL KEY_R") {
            emit recoverSystemPipeline();
        } else if (msg == "SAI_CTRL KEY_A") {
            emit toggleAi();
        } else if (msg == "SAI_CTRL KEY_E") {
            emit toggleEnhancedLpr();
        } else if (msg == "SAI_CTRL KEY_1") {
            emit togglePreprocessStage(1);
        } else if (msg == "SAI_CTRL KEY_2") {
            emit togglePreprocessStage(2);
        } else if (msg == "SAI_CTRL KEY_3") {
            emit togglePreprocessStage(3);
        } else if (msg == "SAI_CTRL KEY_4") {
            emit toggleHelmet();
        } else if (msg == "SAI_CTRL KEY_5") {
            emit toggleTraffic();
        } else if (msg == "SAI_CTRL KEY_6") {
            emit toggleAdas();
        } else if (msg == "SAI_CTRL KEY_7") {
            emit toggleYieldDetection();
        } else if (msg == "SAI_CTRL KEY_ESC") {
            emit quitApplication();
        } else {
            qWarning() << "[WifiCtrl] unknown command:" << msg;
        }
    }
}
