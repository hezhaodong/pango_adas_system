#include "wifi_discovery.h"

#include <QNetworkDatagram>
#include <QDebug>

WifiDiscovery::WifiDiscovery(QObject *parent)
    : QObject(parent)
    , m_socket(nullptr)
    , m_timeoutTimer(nullptr)
    , m_listening(false)
    , m_timeoutSeconds(0)
{
}

WifiDiscovery::~WifiDiscovery()
{
    stop();
}

void WifiDiscovery::start()
{
    if (m_listening) return;

    m_socket = new QUdpSocket(this);
    if (!m_socket->bind(QHostAddress::AnyIPv4, DISCOVERY_PORT,
                        QAbstractSocket::ShareAddress)) {
        qWarning() << "[WifiDiscovery] 绑定 UDP:" << DISCOVERY_PORT
                   << "失败:" << m_socket->errorString();
        delete m_socket;
        m_socket = nullptr;
        return;
    }

    connect(m_socket, &QUdpSocket::readyRead,
            this, &WifiDiscovery::onReadyRead);

    m_timeoutTimer = new QTimer(this);
    m_timeoutTimer->setInterval(1000);
    connect(m_timeoutTimer, &QTimer::timeout,
            this, &WifiDiscovery::onTimeout);

    m_timeoutTimer->start();
    m_timeoutSeconds = 0;
    m_listening = true;

}

void WifiDiscovery::stop()
{
    if (!m_listening) return;

    if (m_timeoutTimer) {
        m_timeoutTimer->stop();
        delete m_timeoutTimer;
        m_timeoutTimer = nullptr;
    }

    if (m_socket) {
        m_socket->close();
        delete m_socket;
        m_socket = nullptr;
    }

    m_listening = false;
}

void WifiDiscovery::onReadyRead()
{
    while (m_socket && m_socket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_socket->receiveDatagram();
        QByteArray data = datagram.data().trimmed();
        QString msg = QString::fromUtf8(data);

        // 格式: "SAI_DISCOVER <tcp_port> <width> <height>"
        if (!msg.startsWith("SAI_DISCOVER")) continue;

        QStringList parts = msg.split(' ', Qt::SkipEmptyParts);
        if (parts.size() < 3) continue;

        bool ok = false;
        quint16 port = static_cast<quint16>(parts[1].toUInt(&ok));
        if (!ok || port == 0) continue;

        int width  = parts.size() >= 4 ? parts[3].toInt() : 1280;
        int height = parts.size() >= 5 ? parts[4].toInt() : 720;

        QString host = datagram.senderAddress().toString();
        // IPv6 映射的 IPv4 地址去掉前缀
        if (host.startsWith("::ffff:"))
            host = host.mid(7);

        emit senderFound(host, port, width, height);
        stop();  // 找到后停止监听
        return;
    }
}

void WifiDiscovery::onTimeout()
{
    ++m_timeoutSeconds;
    if (m_timeoutSeconds >= DISCOVERY_TIMEOUT_S) {
        stop();
    }
}
