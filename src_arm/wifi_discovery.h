#ifndef WIFI_DISCOVERY_H
#define WIFI_DISCOVERY_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

// ============================================================================
// WifiDiscovery — UDP 广播监听器
// 监听 Windows 发送器的自动发现广播，解析其 IP 和 TCP 端口
//
// 协议: Windows 每 1 秒发送 UDP 广播到 255.255.255.255:12345
//       数据格式: "SAI_DISCOVER <tcp_port> <width> <height>"
// ============================================================================
class WifiDiscovery : public QObject
{
    Q_OBJECT

public:
    explicit WifiDiscovery(QObject *parent = nullptr);
    ~WifiDiscovery() override;

    void start();
    void stop();
    bool isListening() const { return m_listening; }

signals:
    // 发现 Windows 发送器
    void senderFound(const QString &host, quint16 port,
                     int width, int height);

private slots:
    void onReadyRead();
    void onTimeout();

private:
    QUdpSocket *m_socket;
    QTimer     *m_timeoutTimer;
    bool        m_listening;
    int         m_timeoutSeconds;

    static constexpr quint16 DISCOVERY_PORT = 12345;
    static constexpr int    DISCOVERY_TIMEOUT_S = 30;
};

#endif // WIFI_DISCOVERY_H
