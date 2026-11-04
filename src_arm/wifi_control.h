#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include <QObject>
#include <QUdpSocket>
#include <opencv2/core.hpp>
#include <atomic>
#include <cstddef>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

// ============================================================================
// WifiControl — UDP 远程控制接收器
// 监听 Windows 发送的控制命令，覆盖 RK3568 主界面的全部操作。
//
// 命令格式 (UDP 端口 12346):
//   SAI_CTRL_DISCOVER <result_port> <image_port>
//       查询控制端并登记 JSON/车牌图片回传端口
//   SAI_CTRL AI_ON      开启 OBB AI
//   SAI_CTRL AI_OFF     关闭 OBB AI
//   SAI_CTRL YOLO_ON    开启 YOLO
//   SAI_CTRL YOLO_OFF   关闭 YOLO
//   SAI_CTRL SRC_PCIE   切换到 PCIe 源
//   SAI_CTRL SRC_WIFI   切换到 WiFi 源
//   SAI_CTRL SRC_IPCAM  切换到手机 HTTP MJPEG 源
//   SAI_CTRL IPCAM_URL <url>  设置手机 HTTP MJPEG 地址
//   SAI_CTRL DEVICE_OPEN / DEVICE_CLOSE
//   SAI_CTRL CAPTURE_START / CAPTURE_STOP
//   SAI_CTRL FPGA_FMC / FPGA_HDMI / FPGA_FIBER
//   SAI_CTRL BRIGHTNESS_AUTO_ON / BRIGHTNESS_AUTO_OFF
//   SAI_CTRL SAVE_FRAME
//   SAI_CTRL KEY_Q / KEY_H / KEY_R / KEY_A / KEY_E
//   SAI_CTRL KEY_1 ... KEY_7 / KEY_ESC
//       与 RK3568 EGLFS 窗口的同名键完全等价
// ============================================================================
class WifiControl : public QObject
{
    Q_OBJECT

public:
    explicit WifiControl(QObject *parent = nullptr);
    ~WifiControl() override;

    bool start();
    void stop();
    void sendRecognitionResult(const QByteArray &jsonPayload);
    bool enqueuePlateImage(const cv::Mat &plateBgr, int frameSeq,
                           int trackId, const QString &plateText);

signals:
    void deviceOpen();
    void deviceClose();
    void captureStart();
    void captureStop();
    void aiOn();
    void aiOff();
    void yoloOn();
    void yoloOff();
    void switchToPcie();
    void switchToWifi();
    void switchToIpCamera();
    void ipCameraUrlChanged(const QString &url);
    void fpgaFmc();
    void fpgaHdmi();
    void fpgaFiber();
    void autoBrightnessOn();
    void autoBrightnessOff();
    void saveFrame();
    void toggleInputSource();
    void toggleFpgaInputSource();
    void recoverSystemPipeline();
    void toggleAi();
    void toggleEnhancedLpr();
    void togglePreprocessStage(int stage);
    void toggleHelmet();
    void toggleTraffic();
    void toggleAdas();
    void toggleYieldDetection();
    void quitApplication();

private slots:
    void onReadyRead();

private:
    struct PlateImageTask {
        cv::Mat image;
        int frameSeq = -1;
        int trackId = -1;
        QString plateText;
    };

    void imageSenderLoop();

    QUdpSocket *m_socket;
    QHostAddress m_resultHost;
    quint16      m_resultPort;
    QString      m_imageHost;
    quint16      m_imagePort;
    std::atomic<bool> m_imageSenderRunning;
    std::thread       m_imageSenderThread;
    std::mutex        m_imageMutex;
    std::condition_variable m_imageCondition;
    std::deque<PlateImageTask> m_imageQueue;
    static constexpr std::size_t MAX_IMAGE_QUEUE = 4;
    static constexpr quint16 CONTROL_PORT = 12346;
};

#endif // WIFI_CONTROL_H
