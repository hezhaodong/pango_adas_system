#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QTimer>
#include <QCheckBox>
#include <QElapsedTimer>
#include <QImage>
#include <QThread>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QListWidget>

#include <string>
#include <vector>

#include "pcie_device.h"
#include "frame_buffer.h"
#include "frame_capture.h"
#include "detection_worker.h"
#include "eglfs_video_window.h"
#include "obb_detector.h"
#include "yolov8_worker.h"
#include "helmet_worker.h"
#include "traffic_worker.h"
#include "video_frame_widget.h"
#include "wifi_receiver.h"
#include "ip_camera_receiver.h"
#include "wifi_discovery.h"
#include "wifi_control.h"
struct PlateHistoryEntry {
    std::string lastValidPlate;
    std::string lastVehicleLabel;
    int         trackId = -1;
    int         classId;
    int         lastSeenFrameSeq;
};

enum class InputSource { PCIE, WIFI, IP_CAMERA, NONE };

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void prepareForShutdown();
    bool usesDedicatedEglfsDisplay() const;
    void setWifiHost(const QString &host, quint16 port);
    bool setIpCameraUrl(const QString &url, QString *errorMessage = nullptr);

private slots:
    // 设备控制
    void onOpenDevice();
    void onCloseDevice();
    void onStartCapture();
    void onStopCapture();

    // 帧处理
    void onFrameReady(int frameSeq, qint64 captureReadyUs);
    void onDisplayUpdate();

    // 统计
    void onStatsUpdated(unsigned int inputFrames,
                        unsigned int processedFrames,
                        unsigned int dropped,
                        double inputFps,
                        double processFps);

    // 错误处理
    void onCaptureError(const QString &msg);
    void onFrameTimeout();
    void onFrameResumed();

    // 保存图像
    void onSaveImage();
    void onRemoteSaveImage();

    // AI 检测 (OBB + 车牌)
    void onAiDetectClick();
    void onToggleDetection(bool enabled);
    void onPreprocessClick();   // Master toggle for the three-stage cascade.
    void onPreprocessStateChanged(int enabledMask, const QString &message);
    void onFastDetectionReady(const DetectResult &result, int frameSeq);
    void onAsyncInferenceDone(int frameSeq);

    // YOLOPv2 traffic perception (legacy slot/control names retained)
    void onYoloDetectClick();
    void onToggleYoloDetection(bool enabled);
    void onYoloInferenceDone(int frameSeq);
    void onYieldYoloDetectClick();
    void onToggleYieldYoloDetection(bool enabled);
    void onYieldYoloInferenceDone(int frameSeq);
    void onDetect3dClick();
    void onToggleDetect3d(bool enabled);
    void onDetect3dInferenceDone(int frameSeq);
    void onHelmetDetectClick();
    void onToggleHelmetDetection(bool enabled);
    void onHelmetInferenceDone(int frameSeq);
    void onTrafficDetectClick();
    void onToggleTrafficDetection(bool enabled);
    void onTrafficInferenceDone(int frameSeq);

    // 状态消息
    void onInferenceStatus(const QString &msg);
    void onYoloStatus(const QString &msg);
    void onHelmetStatus(const QString &msg);
    void onTrafficStatus(const QString &msg);

    // WiFi 接收
    void onWifiFrameReady(int frameSeq);
    void onWifiConnected();
    void onWifiDisconnected();
    void onWifiConnectionError(const QString &msg);
    void onWifiStatsUpdated(unsigned int frames, double fps, double kbps);

    // 手机 HTTP MJPEG 摄像头
    void onIpCameraConnected();
    void onIpCameraDisconnected();
    void onIpCameraError(const QString &msg);
    void onIpCameraStatsUpdated(unsigned int frames, double fps, double kbps);

    // 源切换
    void onSenderDiscovered(const QString &host, quint16 port,
                            int width, int height);

    // 自动亮度
    void onAutoBrightnessToggled(bool enabled);

private:
    void setupUI();
    void updateDeviceInfo();
    void setUIEnabled(bool enabled);
    void scheduleDedicatedEglfsStartup();
    void startDefaultPcieAfterDisplayReady(int retriesRemaining);
    void showDisplayPlaceholder(const QString &text);
    void setDisplayObbOverlay(const DetectResult &result,
                              const char *const *classNames, int numClasses);
    void clearDisplayObbOverlay();
    void setDisplayYoloOverlay(const YoloDetectResult &result,
                               const char *const *classNames, int numClasses);
    void clearDisplayYoloOverlay();
    void setDisplayYieldYoloOverlay(const YoloDetectResult &result,
                                    const char *const *classNames,
                                    int numClasses);
    void clearDisplayYieldYoloOverlay();
    void setDisplayDetect3dOverlay(const YoloDetectResult &result,
                                   const char *const *classNames, int numClasses);
    void clearDisplayDetect3dOverlay();
    void setDisplayHelmetOverlay(const YoloDetectResult &result,
                                 const char *const *classNames, int numClasses);
    void clearDisplayHelmetOverlay();
    void setDisplayTrafficOverlay(const YoloDetectResult &result,
                                  const char *const *classNames, int numClasses);
    void clearDisplayTrafficOverlay();
    void updateSharedYieldYoloBackend();
    void updateTrafficCrossingAlert();
    void presentDisplayFrame(const QImage &frame);
    void reportCriticalError(const QString &title, const QString &text);
    void refreshDedicatedDisplayHud();
    void refreshPreprocessPreview();
    void onPreprocessStageClick(int stage);
    void onEnhancedLprClick();
    void requestPreprocessMask(int enabledMask);
    void updatePreprocessControls();
    int  estimateSceneLuma() const;
    void updateAutoBrightnessControl();
    DetectResult makeDisplayDetectResult(int currentFrameSeq) const;
    void applyPlateHistory(DetectResult &result, int frameSeq);
    void resetPlateHistory();
    void updatePlateList(const DetectResult &result);
    void sendObbRecognitionResult(const DetectResult &result, int frameSeq);
    void sendYoloRecognitionResult(const YoloDetectResult &result, int frameSeq);
    void toggleInputSource();
    void toggleFpgaInputSource();
    void selectFpgaInputSource(FpgaInputSource source);
    void restartRkPciePipeline();
    void switchToSource(InputSource newSource);

    // ---- PCIe 设备 ----
    PcieDevice   *m_device;

    // ---- 帧缓冲 ----
    FrameBuffer  *m_frameBuffer;
    uint8_t      *m_displayBuffer;

    // ---- 采集线程 ----
    FrameCapture *m_capture;

    // ---- AI 推理线程 (OBB + LPRNet) ----
    DetectionWorker *m_detectionWorker;
    QThread         *m_inferenceThread;       // 两个 Worker 共用
    QPushButton     *m_btnAiDetect;
    bool             m_detectionEnabled;

    // ---- YOLOv8 推理 (标准轴对齐框) ----
    YoloV8Worker   *m_yoloWorker;            // 与 DetectionWorker 同线程
    QPushButton     *m_btnYoloDetect;
    bool             m_yoloEnabled;
    YoloV8Worker   *m_yieldYoloWorker;       // 斑马线/车辆/行人 YOLOv8
    bool             m_yieldYoloEnabled;
    bool             m_yieldFeatureEnabled;
    bool             m_crossingFeatureEnabled;
    YoloV8Worker   *m_detect3dWorker;
    QPushButton     *m_btnDetect3d;
    bool             m_detect3dEnabled;
    bool             m_adasEnabled;

    HelmetWorker    *m_helmetWorker;
    QThread         *m_helmetThread;
    bool             m_helmetEnabled;

    TrafficWorker   *m_trafficWorker;
    QThread         *m_trafficThread;
    bool             m_trafficEnabled;

    // ---- Display-only SCI + FFDNet + MIMO+ comparison ----
    QPushButton     *m_btnPreprocess;
    bool             m_preprocessEnabled;
    int              m_preprocessMask;
    int              m_preprocessRequestedMask;
    bool             m_preprocessRequestPending;
    int              m_lastDisplayedPlatePairSeq;

    // ---- Enhanced-cascade LPR result row (separate from raw LPR) ----
    bool             m_enhancedLprEnabled;
    int              m_lastEnhancedPlateSeq;
    QString          m_enhancedLprDisplayText;

    // ---- 检测结果显示 (UI 线程读取) ----
    DetectResult     m_stableDetectResult;
    int              m_stableDetectFrameSeq;
    QElapsedTimer    m_detectResultTimer;
    std::vector<PlateHistoryEntry> m_plateHistory;

    // ---- WiFi 采集 ----
    WifiReceiver  *m_wifiReceiver;
    IpCameraReceiver *m_ipCameraReceiver;
    WifiDiscovery *m_wifiDiscovery;
    WifiControl   *m_wifiControl;
    InputSource    m_currentSource;
    FpgaInputSource m_fpgaInputSource;
    QString        m_sourceName;

    // ---- 类别名称缓存 ----
    std::vector<const char *> m_plateClassNames;
    std::vector<const char *> m_yoloClassNames;
    std::vector<const char *> m_yieldYoloClassNames;
    std::vector<const char *> m_detect3dClassNames;
    std::vector<const char *> m_helmetClassNames;
    std::vector<const char *> m_trafficClassNames;

    // Key 5 combines the traffic-light/crosswalk worker with the shared
    // YOLOv8 pedestrian result. Keep the latest source-frame results so the
    // alert is emitted only when both conditions occur in the same scene.
    YoloDetectResult m_latestYieldYoloForCrossing;
    int              m_latestYieldYoloSourceSeq = -1;
    YoloDetectResult m_latestTrafficForCrossing;
    int              m_latestTrafficSourceSeq = -1;

    // ---- 定时器 ----
    QTimer       *m_displayTimer;
    QTimer       *m_statsTimer;

    // ---- 统计 ----
    double        m_inputFPS;
    double        m_displayFPS;
    unsigned int  m_displayFrameCount;
    QElapsedTimer m_displayFpsTimer;
    const uint8_t *m_lastDisplaySrc;
    QImage        m_cachedDisplayImage;
    bool          m_hasCachedDisplayImage;
    int           m_cachedDisplayFrameSeq;
    QString       m_plateListCacheKey;

    // ---- display performance instrumentation (1 Hz aggregate, qWarning) ----
    QElapsedTimer m_dispProfTimer;
    qint64        m_dispProfWindowUs = 0;
    qint64        m_dispPollTotalUs = 0;
    qint64        m_dispPollMaxUs = 0;
    int           m_dispPollCount = 0;
    qint64        m_dispWorkTotalUs = 0;
    qint64        m_dispWorkMaxUs = 0;
    int           m_dispWorkCount = 0;
    qint64        m_captureSignalQueueTotalUs = 0;
    qint64        m_captureSignalQueueMaxUs = 0;
    int           m_captureSignalQueueCount = 0;
    int           m_lastCaptureSignalSeq = -1;
    quint64       m_captureSignalGapFrames = 0;
    qint64        m_lastDispUpdateUs = 0;
    int           m_lastFrameSeq;
    int           m_autoBrightnessFrameCounter;
    int           m_currentBrightnessLevel;
    int           m_lastSceneLuma;
    bool          m_useDedicatedEglfsDisplay;
    bool          m_shuttingDown;

    // ---- UI 组件 ----
    VideoFrameWidget *m_videoWidget;
    EglfsVideoWindow *m_eglfsVideoWindow;
    QLabel       *m_statusLabel;

    QLineEdit    *m_vendorIdEdit;
    QLineEdit    *m_linkSpeedEdit;
    QLineEdit    *m_linkWidthEdit;
    QLineEdit    *m_maxPayloadEdit;

    QLabel       *m_inputFpsLabel;
    QLabel       *m_displayFpsLabel;
    QLabel       *m_plateCountLabel = nullptr;
    QListWidget  *m_plateListWidget = nullptr;

    QPushButton  *m_btnOpenDevice;
    QPushButton  *m_btnStartCapture;
    QPushButton  *m_btnFpgaSource;
    QPushButton  *m_btnSaveImage;
    QCheckBox    *m_autoBrightnessCheckBox;

    QLabel       *m_linkStatusLabel;
};

#endif // MAINWINDOW_H
