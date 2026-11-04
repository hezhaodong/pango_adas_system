// 高速PCIe摄像头采集 + AI检测 (方案一: 自驱动异步推理)
// 推理线程直接从 FrameBuffer 读取, UI 线程专注显示
#include "mainwindow.h"
#include "overlay_renderer.h"
#include "image_converter.h"

#include <QApplication>
#include <QGuiApplication>
#include <QDebug>
#include <QStatusBar>
#include <QShortcut>
#include <unistd.h>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QFont>
#include <QFrame>
#include <QDir>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QStringList>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <set>

namespace {

constexpr unsigned int kPioBrightnessLevelOffset = 0x070;
constexpr int kAutoBrightnessUpdateInterval = 5;
constexpr int kDisplayResultMaxAgeFrames = 30;
constexpr int kDisplayResultMaxAgeMs = 500;
constexpr int kPlateHistoryMaxAgeFrames = 24;
constexpr int kPlateClassCount = 4;

QString fpgaSourceDisplayName(FpgaInputSource source)
{
    switch (source) {
    case FpgaInputSource::FMC:   return QStringLiteral("PCIe/摄像头");
    case FpgaInputSource::HDMI:  return QStringLiteral("PCIe/HDMI");
    case FpgaInputSource::Fiber: return QStringLiteral("PCIe/光纤");
    }
    return QStringLiteral("PCIe/未知");
}

QString fpgaSourceShortName(FpgaInputSource source)
{
    switch (source) {
    case FpgaInputSource::FMC:   return QStringLiteral("摄像头");
    case FpgaInputSource::HDMI:  return QStringLiteral("HDMI");
    case FpgaInputSource::Fiber: return QStringLiteral("光纤");
    }
    return QStringLiteral("未知");
}

FpgaInputSource nextFpgaInputSource(FpgaInputSource source)
{
    // Preserve the existing first click from HDMI to optical input while
    // retaining the FMC camera as the third selectable physical source.
    switch (source) {
    case FpgaInputSource::HDMI:  return FpgaInputSource::Fiber;
    case FpgaInputSource::Fiber: return FpgaInputSource::FMC;
    case FpgaInputSource::FMC:   return FpgaInputSource::HDMI;
    }
    return FpgaInputSource::HDMI;
}

QString plateDisplayText(const ObbBox &box)
{
    if (box.plateName.empty()) return QStringLiteral("识别中…");
    QString text = QString::fromStdString(box.plateName);
    if (!box.vehicleLabel.empty()) {
        text += QStringLiteral(" ") + QString::fromStdString(box.vehicleLabel);
    }
    return text;
}

QString resolveFirstExistingPath(const QStringList &candidates)
{
    for (const QString &path : candidates) {
        if (QFileInfo::exists(path)) return path;
    }
    return candidates.isEmpty() ? QString() : candidates.front();
}

bool boxesOverlap(const YoloBox &a, const YoloBox &b)
{
    return std::max(a.left, b.left) < std::min(a.right, b.right) &&
           std::max(a.top, b.top) < std::min(a.bottom, b.bottom);
}

bool hasNoHelmetDetection(const YoloDetectResult &result)
{
    return std::any_of(result.boxes.begin(), result.boxes.end(),
                       [](const YoloBox &box) {
                           return box.classId == 2; // no-helmet
                       });
}

bool isPedestrianBox(const YoloBox &box,
                     const std::vector<const char *> &classNames)
{
    if (box.classId < 0 || box.classId >= static_cast<int>(classNames.size()) ||
        !classNames[box.classId]) {
        // COCO-style pedestrian models conventionally use class 0 for person.
        return box.classId == 0;
    }
    const QString name = QString::fromUtf8(classNames[box.classId]).toLower();
    return name.contains(QStringLiteral("person")) ||
           name.contains(QStringLiteral("pedestrian"));
}

bool hasRedLightCrosswalkPedestrian(const YoloDetectResult &traffic,
                                    const YoloDetectResult &pedestrians,
                                    const std::vector<const char *> &classNames)
{
    const bool redLightPresent = std::any_of(
        traffic.boxes.begin(), traffic.boxes.end(),
        [](const YoloBox &box) { return box.classId == 1; });
    if (!redLightPresent) return false;

    for (const YoloBox &crosswalk : traffic.boxes) {
        if (crosswalk.classId != 0) continue;
        for (const YoloBox &person : pedestrians.boxes) {
            if (isPedestrianBox(person, classNames) &&
                boxesOverlap(person, crosswalk)) {
                return true;
            }
        }
    }
    return false;
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_device(nullptr)
    , m_frameBuffer(nullptr)
    , m_displayBuffer(nullptr)
    , m_capture(nullptr)
    , m_detectionWorker(nullptr)
    , m_inferenceThread(nullptr)
    , m_detectionEnabled(false)
    , m_yoloWorker(nullptr)
    , m_yoloEnabled(false)
    , m_yieldYoloWorker(nullptr)
    , m_yieldYoloEnabled(false)
    , m_yieldFeatureEnabled(false)
    , m_crossingFeatureEnabled(false)
    , m_detect3dWorker(nullptr)
    , m_btnDetect3d(nullptr)
    , m_detect3dEnabled(false)
    , m_adasEnabled(false)
    , m_helmetWorker(nullptr)
    , m_helmetThread(nullptr)
    , m_helmetEnabled(false)
    , m_trafficWorker(nullptr)
    , m_trafficThread(nullptr)
    , m_trafficEnabled(false)
    , m_preprocessEnabled(false)
    , m_preprocessMask(PlatePreprocessNone)
    , m_preprocessRequestedMask(PlatePreprocessNone)
    , m_preprocessRequestPending(false)
    , m_lastDisplayedPlatePairSeq(-1)
    , m_enhancedLprEnabled(false)
    , m_lastEnhancedPlateSeq(-1)
    , m_enhancedLprDisplayText()
    , m_stableDetectFrameSeq(-1)
    , m_wifiReceiver(nullptr)
    , m_ipCameraReceiver(nullptr)
    , m_wifiDiscovery(nullptr)
    , m_currentSource(InputSource::NONE)
    // Application startup policy: present HDMI first. The FPGA source control
    // cycles HDMI -> fiber -> FMC camera -> HDMI.
    , m_fpgaInputSource(FpgaInputSource::HDMI)
    , m_displayTimer(nullptr)
    , m_statsTimer(nullptr)
    , m_inputFPS(0.0)
    , m_displayFPS(0.0)
    , m_displayFrameCount(0)
    , m_lastDisplaySrc(nullptr)
    , m_hasCachedDisplayImage(false)
    , m_cachedDisplayFrameSeq(-1)
    , m_lastFrameSeq(-1)
    , m_autoBrightnessFrameCounter(0)
    , m_currentBrightnessLevel(0)
    , m_lastSceneLuma(0)
    , m_useDedicatedEglfsDisplay(false)
    , m_shuttingDown(false)
    , m_eglfsVideoWindow(nullptr)
{
    m_displayBuffer     = new uint8_t[FRAME_SIZE_RGB565];

    m_dispProfTimer.start();

    m_device      = new PcieDevice(this);
    m_frameBuffer = new FrameBuffer();
    m_capture     = new FrameCapture(m_device, m_frameBuffer, this);

    // ---- WiFi 采集 ----
    m_wifiReceiver = new WifiReceiver(m_frameBuffer, this);
    m_ipCameraReceiver = new IpCameraReceiver(m_frameBuffer, this);
    m_wifiDiscovery = new WifiDiscovery(this);
    m_wifiControl   = new WifiControl(this);
    m_wifiControl->start();

    // WiFi 远程控制信号
    connect(m_wifiControl, &WifiControl::deviceOpen, this, [this]() {
        if (!m_device->isOpened()) onOpenDevice();
    });
    connect(m_wifiControl, &WifiControl::deviceClose, this, [this]() {
        if (m_device->isOpened()) onCloseDevice();
    });
    connect(m_wifiControl, &WifiControl::captureStart, this, [this]() {
        if (!m_device->isOpened()) return;
        if (m_currentSource != InputSource::PCIE) {
            switchToSource(InputSource::PCIE);
        } else if (!m_capture->isRunning()) {
            onStartCapture();
        }
    });
    connect(m_wifiControl, &WifiControl::captureStop, this, [this]() {
        if (m_capture->isRunning()) onStopCapture();
    });
    connect(m_wifiControl, &WifiControl::aiOn, this, [this]() {
        if (!m_detectionEnabled) onAiDetectClick();
    });
    connect(m_wifiControl, &WifiControl::aiOff, this, [this]() {
        if (m_detectionEnabled) onAiDetectClick();
    });
    connect(m_wifiControl, &WifiControl::yoloOn, this, [this]() {
        if (!m_yoloEnabled) onYoloDetectClick();
    });
    connect(m_wifiControl, &WifiControl::yoloOff, this, [this]() {
        if (m_yoloEnabled) onYoloDetectClick();
    });
    connect(m_wifiControl, &WifiControl::switchToPcie, this, [this]() {
        if (m_currentSource != InputSource::PCIE) switchToSource(InputSource::PCIE);
    });
    connect(m_wifiControl, &WifiControl::switchToWifi, this, [this]() {
        if (m_currentSource != InputSource::WIFI) switchToSource(InputSource::WIFI);
    });
    connect(m_wifiControl, &WifiControl::switchToIpCamera, this, [this]() {
        if (!m_ipCameraReceiver->hasUrl()) {
            m_statusLabel->setText(QStringLiteral("请先从 Windows 端设置手机 IP Cam 地址"));
            refreshDedicatedDisplayHud();
            return;
        }
        if (m_currentSource != InputSource::IP_CAMERA)
            switchToSource(InputSource::IP_CAMERA);
    });
    connect(m_wifiControl, &WifiControl::ipCameraUrlChanged,
            this, [this](const QString &url) {
        QString error;
        if (m_ipCameraReceiver->setUrl(url, &error)) {
            if (m_currentSource == InputSource::IP_CAMERA) {
                m_ipCameraReceiver->stop();
                m_frameBuffer->reset();
                m_ipCameraReceiver->start();
                m_statusLabel->setText(QStringLiteral("手机 IP Cam 地址已更新，正在重连..."));
            } else {
                m_statusLabel->setText(QStringLiteral("手机 IP Cam 地址已更新"));
            }
        } else {
            m_statusLabel->setText(QStringLiteral("手机 IP Cam 地址无效: ") + error);
        }
        refreshDedicatedDisplayHud();
    });
    connect(m_wifiControl, &WifiControl::fpgaFmc, this, [this]() {
        if (m_device->isOpened() && m_fpgaInputSource != FpgaInputSource::FMC)
            selectFpgaInputSource(FpgaInputSource::FMC);
    });
    connect(m_wifiControl, &WifiControl::fpgaHdmi, this, [this]() {
        if (m_device->isOpened() && m_fpgaInputSource != FpgaInputSource::HDMI)
            selectFpgaInputSource(FpgaInputSource::HDMI);
    });
    connect(m_wifiControl, &WifiControl::fpgaFiber, this, [this]() {
        if (m_device->isOpened() && m_fpgaInputSource != FpgaInputSource::Fiber)
            selectFpgaInputSource(FpgaInputSource::Fiber);
    });
    connect(m_wifiControl, &WifiControl::autoBrightnessOn, this, [this]() {
        if (m_device->isOpened()) m_autoBrightnessCheckBox->setChecked(true);
    });
    connect(m_wifiControl, &WifiControl::autoBrightnessOff, this, [this]() {
        m_autoBrightnessCheckBox->setChecked(false);
    });
    connect(m_wifiControl, &WifiControl::saveFrame,
            this, &MainWindow::onRemoteSaveImage);
    // Keep remote keyboard-equivalent controls on exactly the same code paths
    // as EglfsVideoWindow::keyPressEvent(), so local and WiFi behavior cannot
    // drift apart as features evolve.
    connect(m_wifiControl, &WifiControl::toggleInputSource,
            this, &MainWindow::toggleInputSource);
    connect(m_wifiControl, &WifiControl::toggleFpgaInputSource,
            this, &MainWindow::toggleFpgaInputSource);
    connect(m_wifiControl, &WifiControl::recoverSystemPipeline,
            this, &MainWindow::restartRkPciePipeline);
    connect(m_wifiControl, &WifiControl::toggleAi,
            this, &MainWindow::onAiDetectClick);
    connect(m_wifiControl, &WifiControl::toggleEnhancedLpr,
            this, &MainWindow::onEnhancedLprClick);
    connect(m_wifiControl, &WifiControl::togglePreprocessStage,
            this, &MainWindow::onPreprocessStageClick);
    connect(m_wifiControl, &WifiControl::toggleHelmet,
            this, &MainWindow::onHelmetDetectClick);
    connect(m_wifiControl, &WifiControl::toggleTraffic,
            this, &MainWindow::onTrafficDetectClick);
    connect(m_wifiControl, &WifiControl::toggleAdas,
            this, &MainWindow::onDetect3dClick);
    connect(m_wifiControl, &WifiControl::toggleYieldDetection,
            this, &MainWindow::onYieldYoloDetectClick);
    connect(m_wifiControl, &WifiControl::quitApplication,
            qApp, &QCoreApplication::quit);

    // ---- 创建推理线程 (两个 Worker 共用, Qt 事件循环天然串行化 NPU) ----
    m_inferenceThread = new QThread(this);
    m_inferenceThread->start();

    // OBB + LPRNet Worker
    m_detectionWorker = new DetectionWorker();
    m_detectionWorker->setFrameBuffer(m_frameBuffer);
    m_detectionWorker->setWifiControl(m_wifiControl);
    m_detectionWorker->moveToThread(m_inferenceThread);

    // YOLOv8 Worker (标准轴对齐框, 与 OBB 共用线程)
    m_yoloWorker = new YoloV8Worker();
    m_yoloWorker->setFrameBuffer(m_frameBuffer);
    m_yoloWorker->moveToThread(m_inferenceThread);
    m_yieldYoloWorker = new YoloV8Worker();
    m_yieldYoloWorker->setFrameBuffer(m_frameBuffer);
    m_yieldYoloWorker->setYieldDetectionEnabled(true);
    m_yieldYoloWorker->moveToThread(m_inferenceThread);
    m_detect3dWorker = new YoloV8Worker();
    m_detect3dWorker->setFrameBuffer(m_frameBuffer);
    m_detect3dWorker->moveToThread(m_inferenceThread);

    // Helmet inference has its own event-loop thread, independent of display,
    // capture, OBB/LPR and the existing traffic YOLO worker.
    m_helmetThread = new QThread(this);
    m_helmetWorker = new HelmetWorker();
    m_helmetWorker->setFrameBuffer(m_frameBuffer);
    m_helmetWorker->moveToThread(m_helmetThread);
    m_helmetThread->start();

    // Independent traffic inference thread and RKNN context.
    m_trafficThread = new QThread(this);
    m_trafficWorker = new TrafficWorker();
    m_trafficWorker->setFrameBuffer(m_frameBuffer);
    m_trafficWorker->moveToThread(m_trafficThread);
    m_trafficThread->start();

    qRegisterMetaType<DetectResult>("DetectResult");

    setupUI();
    QShortcut *yieldYoloShortcut = new QShortcut(QKeySequence(Qt::Key_7), this);
    yieldYoloShortcut->setAutoRepeat(false);
    connect(yieldYoloShortcut, &QShortcut::activated,
            this, &MainWindow::onYieldYoloDetectClick);
    // Detect3D keyboard toggle for the normal Qt window. EGLFS has the same
    // mapping in EglfsVideoWindow::keyPressEvent(). Disable auto-repeat so one
    // physical key press cannot start and immediately stop inference.
    QShortcut *detect3dShortcut = new QShortcut(QKeySequence(Qt::Key_6), this);
    detect3dShortcut->setAutoRepeat(false);
    connect(detect3dShortcut, &QShortcut::activated,
            this, &MainWindow::onDetect3dClick);
    QShortcut *trafficShortcut = new QShortcut(QKeySequence(Qt::Key_5), this);
    connect(trafficShortcut, &QShortcut::activated,
            this, &MainWindow::onTrafficDetectClick);
    setUIEnabled(false);

    // ---- 采集线程信号 ----
    connect(m_capture, &FrameCapture::frameReady,
            this,      &MainWindow::onFrameReady,
            Qt::QueuedConnection);
    connect(m_capture, &FrameCapture::statsUpdated,
            this,      &MainWindow::onStatsUpdated,
            Qt::QueuedConnection);
    connect(m_capture, &FrameCapture::captureError,
            this,      &MainWindow::onCaptureError,
            Qt::QueuedConnection);
    connect(m_capture, &FrameCapture::frameTimeout,
            this,      &MainWindow::onFrameTimeout,
            Qt::QueuedConnection);
    connect(m_capture, &FrameCapture::frameResumed,
            this,      &MainWindow::onFrameResumed,
            Qt::QueuedConnection);

    // ---- 推理线程信号 ----
    connect(m_detectionWorker, &DetectionWorker::inferenceDone,
            this,              &MainWindow::onAsyncInferenceDone,
            Qt::QueuedConnection);
    connect(m_detectionWorker, &DetectionWorker::statusMessage,
            this,              &MainWindow::onInferenceStatus,
            Qt::QueuedConnection);
    connect(m_detectionWorker, &DetectionWorker::preprocessStateChanged,
            this,              &MainWindow::onPreprocessStateChanged,
            Qt::QueuedConnection);

    // ---- YOLOv8 推理信号 ----
    connect(m_yoloWorker, &YoloV8Worker::inferenceDone,
            this,         &MainWindow::onYoloInferenceDone,
            Qt::QueuedConnection);
    connect(m_yoloWorker, &YoloV8Worker::statusMessage,
            this,         &MainWindow::onYoloStatus,
            Qt::QueuedConnection);
    connect(m_yieldYoloWorker, &YoloV8Worker::inferenceDone,
            this, &MainWindow::onYieldYoloInferenceDone,
            Qt::QueuedConnection);
    connect(m_yieldYoloWorker, &YoloV8Worker::statusMessage,
            this, &MainWindow::onYoloStatus,
            Qt::QueuedConnection);
    connect(m_detect3dWorker, &YoloV8Worker::inferenceDone,
            this, &MainWindow::onDetect3dInferenceDone, Qt::QueuedConnection);
    connect(m_detect3dWorker, &YoloV8Worker::statusMessage,
            this, &MainWindow::onYoloStatus, Qt::QueuedConnection);
    connect(m_helmetWorker, &HelmetWorker::inferenceDone,
            this, &MainWindow::onHelmetInferenceDone, Qt::QueuedConnection);
    connect(m_helmetWorker, &HelmetWorker::statusMessage,
            this, &MainWindow::onHelmetStatus, Qt::QueuedConnection);
    connect(m_trafficWorker, &TrafficWorker::inferenceDone,
            this, &MainWindow::onTrafficInferenceDone, Qt::QueuedConnection);
    connect(m_trafficWorker, &TrafficWorker::statusMessage,
            this, &MainWindow::onTrafficStatus, Qt::QueuedConnection);

    // ---- WiFi 接收信号 ----
    connect(m_wifiReceiver, &WifiReceiver::frameReady,
            this,          &MainWindow::onWifiFrameReady,
            Qt::QueuedConnection);
    connect(m_wifiReceiver, &WifiReceiver::connected,
            this,          &MainWindow::onWifiConnected,
            Qt::QueuedConnection);
    connect(m_wifiReceiver, &WifiReceiver::disconnected,
            this,          &MainWindow::onWifiDisconnected,
            Qt::QueuedConnection);
    connect(m_wifiReceiver, &WifiReceiver::connectionError,
            this,          &MainWindow::onWifiConnectionError,
            Qt::QueuedConnection);
    connect(m_wifiReceiver, &WifiReceiver::statsUpdated,
            this,          &MainWindow::onWifiStatsUpdated,
            Qt::QueuedConnection);

    // ---- 手机 HTTP MJPEG 接收信号 ----
    connect(m_ipCameraReceiver, &IpCameraReceiver::connected,
            this, &MainWindow::onIpCameraConnected, Qt::QueuedConnection);
    connect(m_ipCameraReceiver, &IpCameraReceiver::disconnected,
            this, &MainWindow::onIpCameraDisconnected, Qt::QueuedConnection);
    connect(m_ipCameraReceiver, &IpCameraReceiver::connectionError,
            this, &MainWindow::onIpCameraError, Qt::QueuedConnection);
    connect(m_ipCameraReceiver, &IpCameraReceiver::statsUpdated,
            this, &MainWindow::onIpCameraStatsUpdated, Qt::QueuedConnection);

    // ---- 自动发现信号 ----
    connect(m_wifiDiscovery, &WifiDiscovery::senderFound,
            this,           &MainWindow::onSenderDiscovered,
            Qt::QueuedConnection);

    // 显示刷新定时器: 16ms (~60Hz)
    m_displayTimer = new QTimer(this);
    connect(m_displayTimer, &QTimer::timeout,
            this,           &MainWindow::onDisplayUpdate);
    m_displayTimer->setTimerType(Qt::PreciseTimer);
    // Frame signal (frameReady) now drives display updates for PCIe sources.
    // This timer is only a fallback for non-PCIe sources (WiFi/IP camera),
    // so 16 ms polling is enough and keeps the UI thread mostly free.
    m_displayTimer->setInterval(16);

    m_statusLabel = new QLabel("就绪");
    statusBar()->addPermanentWidget(m_statusLabel);

    m_useDedicatedEglfsDisplay =
        QGuiApplication::platformName().contains("eglfs", Qt::CaseInsensitive);
    if (m_useDedicatedEglfsDisplay) {
        m_eglfsVideoWindow = new EglfsVideoWindow();
        m_eglfsVideoWindow->showPlaceholder(QStringLiteral("Waiting for capture..."));
        m_eglfsVideoWindow->setToggleAiHandler([this]() { onAiDetectClick(); });
        m_eglfsVideoWindow->setToggleYieldYoloHandler(
            [this]() { onYieldYoloDetectClick(); });
        m_eglfsVideoWindow->setToggleDetect3dHandler([this]() { onDetect3dClick(); });
        m_eglfsVideoWindow->setToggleHelmetHandler([this]() { onHelmetDetectClick(); });
        m_eglfsVideoWindow->setToggleTrafficHandler([this]() { onTrafficDetectClick(); });
        m_eglfsVideoWindow->setToggleSourceHandler([this]() { toggleInputSource(); });
        m_eglfsVideoWindow->setToggleFpgaSourceHandler([this]() { toggleFpgaInputSource(); });
        m_eglfsVideoWindow->setTogglePreprocessHandler([this]() { onPreprocessClick(); });
        m_eglfsVideoWindow->setToggleEnhancedLprHandler([this]() { onEnhancedLprClick(); });
        m_eglfsVideoWindow->setTogglePreprocessStageHandler(
            [this](int stage) { onPreprocessStageClick(stage); });
        m_eglfsVideoWindow->setSystemRecoveryHandler([this]() { restartRkPciePipeline(); });
        scheduleDedicatedEglfsStartup();
    }

    this->setWindowTitle("高速PCIe摄像头采集 v3.0 — 自驱动异步AI");
    this->resize(1320, 820);
}

void MainWindow::setWifiHost(const QString &host, quint16 port)
{
    m_wifiReceiver->connectToHost(host, port);
}

bool MainWindow::setIpCameraUrl(const QString &url, QString *errorMessage)
{
    // Command-line URL configures the optional phone source only.  Keep the
    // normal startup source as PCIe; the user can select IP Cam later with Q.
    return m_ipCameraReceiver->setUrl(url, errorMessage);
}


MainWindow::~MainWindow()
{
    prepareForShutdown();

    // QImage in the EGLFS window references m_displayBuffer.  Destroy that
    // view before releasing the backing storage.
    m_cachedDisplayImage = QImage();
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->showPlaceholder(QString());
        delete m_eglfsVideoWindow;
        m_eglfsVideoWindow = nullptr;
    }

    if (m_inferenceThread && m_inferenceThread->isRunning()) {
        if (m_detectionWorker)
            QMetaObject::invokeMethod(m_detectionWorker, "stopInference",
                                      Qt::BlockingQueuedConnection);
        if (m_yoloWorker)
            QMetaObject::invokeMethod(m_yoloWorker, "stopInference",
                                      Qt::BlockingQueuedConnection);
        if (m_yieldYoloWorker)
            QMetaObject::invokeMethod(m_yieldYoloWorker, "stopInference",
                                      Qt::BlockingQueuedConnection);
        if (m_detect3dWorker)
            QMetaObject::invokeMethod(m_detect3dWorker, "stopInference",
                                      Qt::BlockingQueuedConnection);
    }
    if (m_inferenceThread) {
        m_inferenceThread->quit();
        m_inferenceThread->wait();
    }
    if (m_helmetThread && m_helmetThread->isRunning()) {
        if (m_helmetWorker)
            QMetaObject::invokeMethod(m_helmetWorker, "stopInference",
                                      Qt::BlockingQueuedConnection);
        m_helmetThread->quit();
        m_helmetThread->wait();
    }
    if (m_trafficThread && m_trafficThread->isRunning()) {
        if (m_trafficWorker)
            QMetaObject::invokeMethod(m_trafficWorker, "stopInference",
                                      Qt::BlockingQueuedConnection);
        m_trafficThread->quit();
        m_trafficThread->wait();
    }

    delete m_detectionWorker;
    delete m_yoloWorker;
    delete m_yieldYoloWorker;
    delete m_detect3dWorker;
    delete m_helmetWorker;
    delete m_trafficWorker;
    delete m_capture;
    m_capture = nullptr;
    delete m_wifiReceiver;
    m_wifiReceiver = nullptr;
    delete m_ipCameraReceiver;
    m_ipCameraReceiver = nullptr;
    if (m_device) {
        m_device->closeDevice();
        delete m_device;
        m_device = nullptr;
    }
    delete m_frameBuffer;
    m_frameBuffer = nullptr;
    delete[] m_displayBuffer;
    m_displayBuffer = nullptr;
    qCritical() << "[shutdown-guard] all producers stopped; shared buffers released";
}

void MainWindow::prepareForShutdown()
{
    if (m_shuttingDown)
        return;

    m_shuttingDown = true;
    if (m_displayTimer) m_displayTimer->stop();
    if (m_statsTimer) m_statsTimer->stop();
    if (m_capture && m_capture->isRunning()) {
        m_capture->stopCapture();
    }
    bool fpgaStopped = true;
    if (m_device && m_device->isOpened()) {
        fpgaStopped = m_device->stopCaptureChecked(300);
    }
    if (m_capture && m_capture->isRunning()) {
        // A live capture reader must never outlive the coherent DMA buffer.
        m_capture->wait();
    }
    if (m_wifiReceiver) m_wifiReceiver->stop();
    if (m_ipCameraReceiver) m_ipCameraReceiver->stop();
    if (m_wifiDiscovery) m_wifiDiscovery->stop();
    if (m_wifiControl) m_wifiControl->stop();
    qCritical().nospace()
        << "[shutdown-guard] DMA stopped before Qt teardown confirmed="
        << fpgaStopped;
}

// ===================================================================
//  UI 构建
// ===================================================================
void MainWindow::setupUI()
{
    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    QHBoxLayout *mainLayout = new QHBoxLayout(central);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // ---- 左侧: 视频显示区 ----
    QVBoxLayout *videoLayout = new QVBoxLayout();
    m_videoWidget = new VideoFrameWidget();
    m_videoWidget->setFixedSize(FRAME_WIDTH, FRAME_HEIGHT);
    QFont font; font.setPointSize(36); font.setBold(true);
    m_videoWidget->setFont(font);
    m_videoWidget->setStyleSheet(
        "QWidget { background-color: #1a1a1a; border: 2px solid #555; color: #666; }");
    m_videoWidget->showPlaceholder("等待采集...");
    videoLayout->addWidget(m_videoWidget);

    // 统计行
    QHBoxLayout *statsLayout = new QHBoxLayout();
    m_inputFpsLabel      = new QLabel("输入: 0 FPS");
    m_displayFpsLabel    = new QLabel("显示: 0 FPS");
    m_linkStatusLabel    = new QLabel("● 未连接");

    m_inputFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #08c;");
    m_displayFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #a0a;");
    m_linkStatusLabel->setStyleSheet("font-size: 11px; color: #888;");

    statsLayout->addWidget(m_inputFpsLabel);
    statsLayout->addWidget(m_displayFpsLabel);
    statsLayout->addWidget(m_linkStatusLabel);
    statsLayout->addStretch();
    videoLayout->addLayout(statsLayout);
    mainLayout->addLayout(videoLayout, 3);

    // ---- 右侧: 控制面板 ----
    QVBoxLayout *ctrlLayout = new QVBoxLayout();
    ctrlLayout->setSpacing(8);

    QGroupBox *devGroup = new QGroupBox("PCIe 设备信息");
    QGridLayout *devGrid = new QGridLayout(devGroup);
    devGrid->addWidget(new QLabel("Vendor ID:"),  0, 0);
    m_vendorIdEdit   = new QLineEdit(); m_vendorIdEdit->setReadOnly(true);
    devGrid->addWidget(m_vendorIdEdit,             0, 1);
    devGrid->addWidget(new QLabel("Link Speed:"),  1, 0);
    m_linkSpeedEdit  = new QLineEdit(); m_linkSpeedEdit->setReadOnly(true);
    devGrid->addWidget(m_linkSpeedEdit,            1, 1);
    devGrid->addWidget(new QLabel("Link Width:"),  2, 0);
    m_linkWidthEdit  = new QLineEdit(); m_linkWidthEdit->setReadOnly(true);
    devGrid->addWidget(m_linkWidthEdit,            2, 1);
    devGrid->addWidget(new QLabel("Max Payload:"), 3, 0);
    m_maxPayloadEdit = new QLineEdit(); m_maxPayloadEdit->setReadOnly(true);
    devGrid->addWidget(m_maxPayloadEdit,           3, 1);
    ctrlLayout->addWidget(devGroup);

    // 当前画面中的全部车牌，按跟踪ID分别显示。
    QGroupBox *plateGroup = new QGroupBox("检测到的车牌");
    QVBoxLayout *plateLayout = new QVBoxLayout(plateGroup);
    plateLayout->setContentsMargins(6, 6, 6, 6);
    plateLayout->setSpacing(4);
    m_plateCountLabel = new QLabel("当前 0 个");
    m_plateCountLabel->setStyleSheet("font-size: 11px; color: #666;");
    m_plateListWidget = new QListWidget();
    m_plateListWidget->setMinimumHeight(130);
    m_plateListWidget->setSelectionMode(QAbstractItemView::NoSelection);
    m_plateListWidget->setFocusPolicy(Qt::NoFocus);
    m_plateListWidget->setStyleSheet(
        "QListWidget { font-size: 12px; border: 1px solid #bbb; background: #fafafa; }"
        "QListWidget::item { padding: 3px 4px; }");
    m_plateListWidget->addItem("未检测到车牌");
    plateLayout->addWidget(m_plateCountLabel);
    plateLayout->addWidget(m_plateListWidget);
    ctrlLayout->addWidget(plateGroup);

    m_btnOpenDevice = new QPushButton("打开设备");
    m_btnOpenDevice->setMinimumHeight(40);
    m_btnOpenDevice->setStyleSheet(
        "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1a8cd8; }");
    connect(m_btnOpenDevice, &QPushButton::clicked, this, &MainWindow::onOpenDevice);
    ctrlLayout->addWidget(m_btnOpenDevice);

    m_btnStartCapture = new QPushButton("开始采集");
    m_btnStartCapture->setMinimumHeight(40);
    m_btnStartCapture->setEnabled(false);
    m_btnStartCapture->setStyleSheet(
        "QPushButton { background-color: #107c10; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1a9c1a; }"
        "QPushButton:disabled { background-color: #555; }");
    connect(m_btnStartCapture, &QPushButton::clicked, this, &MainWindow::onStartCapture);
    ctrlLayout->addWidget(m_btnStartCapture);

    m_btnFpgaSource = new QPushButton("FPGA input: HDMI");
    m_btnFpgaSource->setMinimumHeight(36);
    m_btnFpgaSource->setEnabled(false);
    connect(m_btnFpgaSource, &QPushButton::clicked,
            this, [this]() { toggleFpgaInputSource(); });
    ctrlLayout->addWidget(m_btnFpgaSource);

    m_btnAiDetect = new QPushButton("[A] 车牌识别");
    m_btnAiDetect->setMinimumHeight(40);
    m_btnAiDetect->setEnabled(false);
    m_btnAiDetect->setStyleSheet(
        "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1a8cd8; }"
        "QPushButton:disabled { background-color: #555; }");
    connect(m_btnAiDetect, &QPushButton::clicked, this, &MainWindow::onAiDetectClick);
    ctrlLayout->addWidget(m_btnAiDetect);

    m_btnYoloDetect = new QPushButton("[6] 自动驾驶辅助");
    m_btnYoloDetect->setMinimumHeight(40);
    m_btnYoloDetect->setEnabled(false);
    m_btnYoloDetect->setStyleSheet(
        "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1a8cd8; }"
        "QPushButton:disabled { background-color: #555; }");
    connect(m_btnYoloDetect, &QPushButton::clicked,
            this, &MainWindow::onDetect3dClick);
    ctrlLayout->addWidget(m_btnYoloDetect);

    m_btnDetect3d = new QPushButton("[7] 斑马线礼让检测");
    m_btnDetect3d->setMinimumHeight(40);
    m_btnDetect3d->setEnabled(false);
    m_btnDetect3d->setStyleSheet(
        "QPushButton { background-color: #006c67; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #008c84; }"
        "QPushButton:disabled { background-color: #555; }");
    connect(m_btnDetect3d, &QPushButton::clicked,
            this, &MainWindow::onYieldYoloDetectClick);
    ctrlLayout->addWidget(m_btnDetect3d);

    m_btnPreprocess = new QPushButton("[1]sci:OFF [2]ffdnet:OFF [3]deblur:OFF");
    m_btnPreprocess->setMinimumHeight(36);
    m_btnPreprocess->setEnabled(true);
    m_btnPreprocess->setStyleSheet(
        "QPushButton { background-color: #8B4513; color: white; font-size: 13px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #A0522D; }"
        "QPushButton:disabled { background-color: #555; }");
    connect(m_btnPreprocess, &QPushButton::clicked, this, &MainWindow::onPreprocessClick);
    ctrlLayout->addWidget(m_btnPreprocess);

    m_autoBrightnessCheckBox = new QCheckBox("开启自适应亮度");
    m_autoBrightnessCheckBox->setChecked(false);
    m_autoBrightnessCheckBox->setEnabled(false);
    connect(m_autoBrightnessCheckBox, &QCheckBox::toggled,
            this, &MainWindow::onAutoBrightnessToggled);
    ctrlLayout->addWidget(m_autoBrightnessCheckBox);

    m_btnSaveImage = new QPushButton("保存当前帧");
    m_btnSaveImage->setMinimumHeight(36);
    m_btnSaveImage->setEnabled(false);
    connect(m_btnSaveImage, &QPushButton::clicked, this, &MainWindow::onSaveImage);
    ctrlLayout->addWidget(m_btnSaveImage);

    ctrlLayout->addStretch();

    QLabel *versionLabel = new QLabel("v3.0 — 自驱动异步AI\nRK3568 + Pango FPGA");
    versionLabel->setStyleSheet("color: #888; font-size: 11px;");
    versionLabel->setAlignment(Qt::AlignCenter);
    ctrlLayout->addWidget(versionLabel);

    mainLayout->addLayout(ctrlLayout, 1);
}

void MainWindow::setUIEnabled(bool enabled)
{
    m_btnStartCapture->setEnabled(enabled);
    m_btnFpgaSource->setEnabled(enabled);
    m_btnSaveImage->setEnabled(enabled);
    m_autoBrightnessCheckBox->setEnabled(enabled);
}

bool MainWindow::usesDedicatedEglfsDisplay() const
{
    return m_useDedicatedEglfsDisplay;
}

void MainWindow::scheduleDedicatedEglfsStartup()
{
    QTimer::singleShot(0, this, [this]() {
        if (!m_eglfsVideoWindow) return;
        // Do not enter initializeGL()/paintGL() re-entrantly from the
        // MainWindow constructor.  Showing after app.exec() starts also makes
        // repeated EGLFS launches substantially more reliable.
        m_eglfsVideoWindow->showFullScreen();
        // Force a placeholder frame now.  Starting capture after an arbitrary
        // 250-ms delay was racy: on this Qt5/Rockchip stack the first real
        // camera frame sometimes caused the initial GBM/KMS FB allocation,
        // and that one-off DDR stall filled the FPGA FIFO to 8192 pixels.
        m_eglfsVideoWindow->update();
        startDefaultPcieAfterDisplayReady(200); // 2 s maximum, 10 ms polling
    });
}

void MainWindow::startDefaultPcieAfterDisplayReady(int retriesRemaining)
{
    if (!m_eglfsVideoWindow)
        return;

    if (!m_eglfsVideoWindow->hasCompletedFirstSwap() && retriesRemaining > 0) {
        QTimer::singleShot(10, this, [this, retriesRemaining]() {
            startDefaultPcieAfterDisplayReady(retriesRemaining - 1);
        });
        return;
    }

    if (!m_eglfsVideoWindow->hasCompletedFirstSwap())
        qWarning() << "[display-startup] first KMS swap timeout; starting PCIe fallback";
    else
        qWarning() << "[display-startup] first KMS swap complete before PCIe START";

    // A command-line network source may already be running by the time EGLFS
    // becomes ready.  Do not replace it with the default PCIe source.
    if (m_currentSource == InputSource::NONE ||
        m_currentSource == InputSource::PCIE) {
        if (!m_device->isOpened())
            onOpenDevice();
        if (m_device->isOpened() && !m_capture->isRunning())
            onStartCapture();
    }
    if (qEnvironmentVariableIsSet("HS_CAMERA_AUTO_AI") && !m_detectionEnabled)
        onAiDetectClick();
    if (qEnvironmentVariableIsSet("HS_CAMERA_AUTO_YOLO") && !m_adasEnabled)
        onDetect3dClick();
    if (qEnvironmentVariableIsSet("HS_CAMERA_AUTO_HELMET") && !m_helmetEnabled)
        onHelmetDetectClick();
    if (qEnvironmentVariableIsSet("HS_CAMERA_AUTO_TRAFFIC") &&
        !m_crossingFeatureEnabled)
        onTrafficDetectClick();
    refreshDedicatedDisplayHud();
}

void MainWindow::showDisplayPlaceholder(const QString &text)
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->showPlaceholder(text);
    } else if (m_videoWidget) {
        m_videoWidget->showPlaceholder(text);
    }
}

void MainWindow::setDisplayObbOverlay(const DetectResult &result,
                                      const char *const *classNames, int numClasses)
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->setObbOverlay(result, classNames, numClasses);
    } else if (m_videoWidget) {
        m_videoWidget->setObbOverlay(result, classNames, numClasses);
    }
}

void MainWindow::clearDisplayObbOverlay()
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->clearObbOverlay();
    } else if (m_videoWidget) {
        m_videoWidget->clearObbOverlay();
    }
}

void MainWindow::setDisplayYoloOverlay(const YoloDetectResult &result,
                                       const char *const *classNames, int numClasses)
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->setYoloOverlay(result, classNames, numClasses);
    } else if (m_videoWidget) {
        m_videoWidget->setYoloOverlay(result, classNames, numClasses);
    }
}

void MainWindow::clearDisplayYoloOverlay()
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->clearYoloOverlay();
    } else if (m_videoWidget) {
        m_videoWidget->clearYoloOverlay();
    }
}

void MainWindow::setDisplayYieldYoloOverlay(
    const YoloDetectResult &result, const char *const *classNames,
    int numClasses)
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->setYieldYoloOverlay(result, classNames, numClasses);
    } else if (m_videoWidget) {
        m_videoWidget->setYieldYoloOverlay(result, classNames, numClasses);
    }
}

void MainWindow::clearDisplayYieldYoloOverlay()
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->clearYieldYoloOverlay();
    } else if (m_videoWidget) {
        m_videoWidget->clearYieldYoloOverlay();
    }
}

void MainWindow::presentDisplayFrame(const QImage &frame)
{
    if (m_eglfsVideoWindow) {
        m_eglfsVideoWindow->presentFrame(frame);
    } else if (m_videoWidget) {
        m_videoWidget->presentFrame(frame);
    }
}

void MainWindow::reportCriticalError(const QString &title, const QString &text)
{
    qCritical() << "[ui-error]" << title << text;
    m_statusLabel->setText(title + ": " + text);
    showDisplayPlaceholder(text);
    if (!m_useDedicatedEglfsDisplay)
        QMessageBox::critical(this, title, text);
}

void MainWindow::refreshDedicatedDisplayHud()
{
    if (!m_eglfsVideoWindow)
        return;

    // Count completed paintGL callbacks, not presentFrame()/update() requests.
    // Qt coalesces update requests, which previously let the HUD report 80
    // while KMS actually presented 60 frames per second.
    const double actualDisplayFps = m_eglfsVideoWindow->measuredPaintFps();
    QStringList lines;
    const auto feature = [](const QString &key, const QString &name,
                            bool enabled) {
        const QString color = enabled ? QStringLiteral("39FF6A")
                                      : QStringLiteral("FF6464");
        return QStringLiteral("<#FFFFFF>[%1] %2  <#%3>%4<#FFFFFF>")
            .arg(key, name, color,
                 enabled ? QStringLiteral("ON") : QStringLiteral("OFF"));
    };
    lines << QString("<#FFFFFF>%1 | 输入 %2 FPS | 显示 %3 FPS")
                 .arg(m_sourceName.isEmpty() ? "NONE" : m_sourceName)
                 .arg(m_inputFPS, 0, 'f', 1)
                 .arg(actualDisplayFps, 0, 'f', 1);
    const int shownPreprocessMask = m_preprocessRequestPending
        ? m_preprocessRequestedMask : m_preprocessMask;
    lines << feature(QStringLiteral("A"), QStringLiteral("车牌识别"),
                     m_detectionEnabled)
             + QStringLiteral("\t<#FFFFFF>[H] 视频切换")
             + QStringLiteral("\t")
             + feature(QStringLiteral("E"), QStringLiteral("增强识别"),
                       m_enhancedLprEnabled);
    lines << feature(QStringLiteral("1"), QStringLiteral("sci"),
                     shownPreprocessMask & PlatePreprocessSci)
             + QStringLiteral("\t")
             + feature(QStringLiteral("2"), QStringLiteral("ffdnet"),
                       shownPreprocessMask & PlatePreprocessFfdnet)
             + QStringLiteral("\t")
             + feature(QStringLiteral("3"), QStringLiteral("deblur"),
                       shownPreprocessMask & PlatePreprocessDeblur);
    lines << feature(QStringLiteral("4"), QStringLiteral("头盔检测"),
                     m_helmetEnabled)
             + QStringLiteral("\t")
             + feature(QStringLiteral("5"), QStringLiteral("行人过马路检测"),
                       m_crossingFeatureEnabled);
    lines << feature(QStringLiteral("6"), QStringLiteral("自动驾驶辅助"),
                     m_adasEnabled)
             + QStringLiteral("\t")
             + feature(QStringLiteral("7"), QStringLiteral("斑马线礼让检测"),
                       m_yieldFeatureEnabled);
    {
        QString enhancedText = QStringLiteral("增强识别结果：--");
        if (m_enhancedLprEnabled && m_detectionWorker) {
            QString enhancedPlate;
            int enhancedSeq = -1;
            if (m_detectionWorker->getLatestEnhancedPlate(
                    enhancedPlate, enhancedSeq, m_lastEnhancedPlateSeq)) {
                m_lastEnhancedPlateSeq = enhancedSeq;
                enhancedText = QStringLiteral("增强识别结果：%1").arg(enhancedPlate);
            } else if (!m_enhancedLprDisplayText.isEmpty()) {
                // No new result: keep the last recognised plate instead of waiting.
                enhancedText = m_enhancedLprDisplayText;
            }
        }
        m_enhancedLprDisplayText = enhancedText;
        lines << enhancedText;
    }

    int plateCount = 0;
    for (const ObbBox &box : m_stableDetectResult.boxes) {
        if (box.classId < 0 || box.classId >= kPlateClassCount) continue;
        ++plateCount;
        const QString text = plateDisplayText(box);
        lines << (box.trackId >= 0
            ? QString("#%1  %2").arg(box.trackId).arg(text)
            : QString("待确认  %1").arg(text));
        if (plateCount >= 6) break;
    }
    if (plateCount == 0) lines << QStringLiteral("车牌: 暂无");
    m_eglfsVideoWindow->setHudLines(lines);
}

void MainWindow::refreshPreprocessPreview()
{
    if (!m_eglfsVideoWindow || !m_detectionWorker || !m_preprocessEnabled)
        return;

    QImage orig, processed;
    int seq = -1;
    if (m_detectionWorker->getLatestPlateComparison(
            orig, processed, seq, m_lastDisplayedPlatePairSeq)) {
        m_lastDisplayedPlatePairSeq = seq;
        const QImage empty;
        m_eglfsVideoWindow->setPlateComparison(orig, empty, empty, processed);
    }
}

void MainWindow::updatePlateList(const DetectResult &result)
{
    if (!m_plateListWidget || !m_plateCountLabel) return;

    // Build the rows first; rebuild the QListWidget only when the text
    // actually changes. clear()+addItem() on every inference callback forced
    // a full layout pass on the UI thread and contributed to input lag.
    std::set<int> shownTrackIds;
    QStringList rows;
    int plateCount = 0;
    int recognizedCount = 0;

    for (const ObbBox &box : result.boxes) {
        if (box.classId < 0 || box.classId >= kPlateClassCount) continue;
        if (box.trackId >= 0 && !shownTrackIds.insert(box.trackId).second) continue;

        ++plateCount;
        const bool recognized = !box.plateName.empty();
        if (recognized) ++recognizedCount;
        const QString plateText = plateDisplayText(box);
        const QString idText = box.trackId >= 0
            ? QString("#%1").arg(box.trackId)
            : QStringLiteral("待确认");
        rows << QString("%1  %2  %3%")
                .arg(idText)
                .arg(plateText)
                .arg(box.confidence * 100.0f, 0, 'f', 1);
    }

    if (plateCount == 0) rows << QStringLiteral("未检测到车牌");
    const QString key = rows.join(QStringLiteral("\n"));
    if (key == m_plateListCacheKey)
        return;
    m_plateListCacheKey = key;

    m_plateListWidget->clear();
    for (const QString &row : rows)
        m_plateListWidget->addItem(row);
    m_plateCountLabel->setText(
        QString("当前 %1 个 · 已识别 %2 个").arg(plateCount).arg(recognizedCount));
}

void MainWindow::sendObbRecognitionResult(const DetectResult &result, int frameSeq)
{
    if (!m_wifiControl) return;

    QJsonObject root;
    root.insert(QStringLiteral("type"), QStringLiteral("obb_lpr"));
    root.insert(QStringLiteral("frame_seq"), frameSeq);
    root.insert(QStringLiteral("timestamp_ms"),
                static_cast<double>(QDateTime::currentMSecsSinceEpoch()));
    root.insert(QStringLiteral("source"), m_sourceName);

    QJsonArray boxes;
    for (const ObbBox &box : result.boxes) {
        QJsonObject item;
        item.insert(QStringLiteral("track_id"), box.trackId);
        item.insert(QStringLiteral("class_id"), box.classId);
        if (box.classId >= 0 && box.classId < m_detectionWorker->numClasses())
            item.insert(QStringLiteral("class_name"),
                        QString::fromUtf8(m_detectionWorker->className(box.classId)));
        item.insert(QStringLiteral("confidence"), box.confidence);
        item.insert(QStringLiteral("cx"), box.cx);
        item.insert(QStringLiteral("cy"), box.cy);
        item.insert(QStringLiteral("width"), box.w);
        item.insert(QStringLiteral("height"), box.h);
        item.insert(QStringLiteral("angle"), box.angle);
        item.insert(QStringLiteral("plate"), QString::fromStdString(box.plateName));
        item.insert(QStringLiteral("vehicle_label"),
                    QString::fromStdString(box.vehicleLabel));
        item.insert(QStringLiteral("recognized_this_frame"),
                    box.plateRecognizedThisFrame);
        boxes.append(item);
    }
    root.insert(QStringLiteral("boxes"), boxes);
    m_wifiControl->sendRecognitionResult(
        QJsonDocument(root).toJson(QJsonDocument::Compact));
}

void MainWindow::sendYoloRecognitionResult(const YoloDetectResult &result, int frameSeq)
{
    if (!m_wifiControl) return;

    QJsonObject root;
    root.insert(QStringLiteral("type"), QStringLiteral("yolo"));
    root.insert(QStringLiteral("frame_seq"), frameSeq);
    root.insert(QStringLiteral("timestamp_ms"),
                static_cast<double>(QDateTime::currentMSecsSinceEpoch()));
    root.insert(QStringLiteral("source"), m_sourceName);

    QJsonArray boxes;
    for (const YoloBox &box : result.boxes) {
        QJsonObject item;
        item.insert(QStringLiteral("class_id"), box.classId);
        if (box.classId >= 0 && box.classId < m_yoloWorker->numClasses())
            item.insert(QStringLiteral("class_name"),
                        QString::fromUtf8(m_yoloWorker->className(box.classId)));
        item.insert(QStringLiteral("confidence"), box.confidence);
        item.insert(QStringLiteral("left"), box.left);
        item.insert(QStringLiteral("top"), box.top);
        item.insert(QStringLiteral("right"), box.right);
        item.insert(QStringLiteral("bottom"), box.bottom);
        item.insert(QStringLiteral("yield_violation"), box.yieldViolation);
        boxes.append(item);
    }
    root.insert(QStringLiteral("boxes"), boxes);
    root.insert(QStringLiteral("yield_violation"), result.yieldViolationAlert);
    QJsonObject lane;
    lane.insert(QStringLiteral("valid"), result.laneGeometryValid);
    lane.insert(QStringLiteral("alert"), result.laneDepartureAlert);
    lane.insert(QStringLiteral("direction"),
                result.laneDepartureDirection < 0 ? QStringLiteral("left") :
                result.laneDepartureDirection > 0 ? QStringLiteral("right") :
                                                    QStringLiteral("none"));
    lane.insert(QStringLiteral("offset_ratio"), result.laneOffsetRatio);
    lane.insert(QStringLiteral("confidence"), result.laneGeometryConfidence);
    lane.insert(QStringLiteral("left_samples"), result.laneLeftSamples);
    lane.insert(QStringLiteral("right_samples"), result.laneRightSamples);
    lane.insert(QStringLiteral("fit_status"), result.laneFitStatus);
    lane.insert(QStringLiteral("road_direction"),
                result.roadDirection < 0 ? QStringLiteral("left") :
                result.roadDirection > 0 ? QStringLiteral("right") :
                                           QStringLiteral("straight"));
    lane.insert(QStringLiteral("road_direction_ratio"), result.roadDirectionRatio);
    lane.insert(QStringLiteral("road_red_yellow_offset_ratio"),
                result.roadDirectionRatio);
    lane.insert(QStringLiteral("road_left_red_yellow_offset_px"),
                result.roadLeftCurvature);
    lane.insert(QStringLiteral("road_right_red_yellow_offset_px"),
                result.roadRightCurvature);
    lane.insert(QStringLiteral("road_left_curve_valid"),
                result.roadLeftCurveValid);
    lane.insert(QStringLiteral("road_right_curve_valid"),
                result.roadRightCurveValid);
    lane.insert(QStringLiteral("road_curve_top_y"), result.roadCurveTopY);
    lane.insert(QStringLiteral("road_left_curve_confidence"),
                result.roadLeftCurveConfidence);
    lane.insert(QStringLiteral("road_right_curve_confidence"),
                result.roadRightCurveConfidence);
    lane.insert(QStringLiteral("road_direction_confidence"),
                result.roadDirectionConfidence);
    lane.insert(QStringLiteral("road_direction_valid"),
                result.roadDirectionConfidence > 0.0f);
    lane.insert(QStringLiteral("camera_center_x"), result.cameraCenterX);
    lane.insert(QStringLiteral("lane_center_x"), result.laneCenterX);
    lane.insert(QStringLiteral("top_y"), result.laneGeometryTopY);
    lane.insert(QStringLiteral("bottom_y"), result.laneGeometryBottomY);
    lane.insert(QStringLiteral("left_top_x"), result.laneLeftTopX);
    lane.insert(QStringLiteral("left_bottom_x"), result.laneLeftBottomX);
    lane.insert(QStringLiteral("right_top_x"), result.laneRightTopX);
    lane.insert(QStringLiteral("right_bottom_x"), result.laneRightBottomX);
    root.insert(QStringLiteral("lane_departure"), lane);
    m_wifiControl->sendRecognitionResult(
        QJsonDocument(root).toJson(QJsonDocument::Compact));
}

int MainWindow::estimateSceneLuma() const
{
    const uint8_t *src = m_lastDisplaySrc ? m_lastDisplaySrc : m_displayBuffer;
    if (!src) return 0;
    qint64 sum = 0;
    int samples = 0;
    constexpr int sampleStep = 8;
    const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);
    for (int y = 0; y < FRAME_HEIGHT; y += sampleStep) {
        const uint16_t *row = src16 + y * FRAME_WIDTH;
        for (int x = 0; x < FRAME_WIDTH; x += sampleStep) {
            const uint16_t px = row[x];
            const int r5 = (px >> 11) & 0x1F;
            const int g6 = (px >> 5)  & 0x3F;
            const int b5 = px & 0x1F;
            const int r = (r5 << 3) | (r5 >> 2);
            const int g = (g6 << 2) | (g6 >> 4);
            const int b = (b5 << 3) | (b5 >> 2);
            sum += (77 * r + 150 * g + 29 * b) >> 8;
            ++samples;
        }
    }
    return samples > 0 ? static_cast<int>(sum / samples) : 0;
}

void MainWindow::updateAutoBrightnessControl()
{
    if (!m_device || !m_device->isOpened()) return;
    if (!m_autoBrightnessCheckBox || !m_autoBrightnessCheckBox->isChecked()) return;

    ++m_autoBrightnessFrameCounter;
    if ((m_autoBrightnessFrameCounter % kAutoBrightnessUpdateInterval) != 0) return;

    const int meanLuma = estimateSceneLuma();
    m_lastSceneLuma = meanLuma;

    int nextLevel = m_currentBrightnessLevel;
    switch (m_currentBrightnessLevel) {
    case 0: if (meanLuma < 78) nextLevel = 1; break;
    case 1: if (meanLuma < 60) nextLevel = 2; else if (meanLuma > 98) nextLevel = 0; break;
    case 2: if (meanLuma < 42) nextLevel = 3; else if (meanLuma > 82) nextLevel = 1; break;
    default: if (meanLuma > 62) nextLevel = 2; break;
    }
    if (nextLevel == m_currentBrightnessLevel) return;

    m_currentBrightnessLevel = nextLevel;
    m_device->pioWriteReg(kPioBrightnessLevelOffset,
                          static_cast<unsigned int>(m_currentBrightnessLevel));
}

void MainWindow::resetPlateHistory()
{
    m_plateHistory.clear();
    if (m_plateListWidget) updatePlateList(DetectResult());
}

void MainWindow::applyPlateHistory(DetectResult &result, int frameSeq)
{
    std::vector<PlateHistoryEntry> nextHistory;
    nextHistory.reserve(std::max(result.boxes.size(), m_plateHistory.size()));
    for (const PlateHistoryEntry &entry : m_plateHistory) {
        if (frameSeq - entry.lastSeenFrameSeq <= kPlateHistoryMaxAgeFrames)
            nextHistory.push_back(entry);
    }
    for (ObbBox &box : result.boxes) {
        if (box.classId < 0 || box.classId >= kPlateClassCount) {
            box.plateName.clear();
            box.vehicleLabel.clear();
            continue;
        }
        // LPRNet has already completed legality validation. Display the first
        // non-empty worker result immediately instead of validating it twice.
        const bool currentValid = !box.plateName.empty();
        auto entryIt = std::find_if(nextHistory.begin(), nextHistory.end(),
                                    [&box](const PlateHistoryEntry &entry) {
                                        return box.trackId >= 0 && entry.trackId == box.trackId;
                                    });
        if (currentValid) {
            if (entryIt != nextHistory.end()) {
                entryIt->lastValidPlate = box.plateName;
                entryIt->lastVehicleLabel = box.vehicleLabel;
                entryIt->classId = box.classId;
                entryIt->lastSeenFrameSeq = frameSeq;
            } else {
                // Provisional results share trackId -1. Show them immediately
                // but never persist them under that common history key.
                if (box.trackId >= 0) {
                    PlateHistoryEntry entry;
                    entry.lastValidPlate = box.plateName;
                    entry.lastVehicleLabel = box.vehicleLabel;
                    entry.trackId = box.trackId;
                    entry.classId = box.classId;
                    entry.lastSeenFrameSeq = frameSeq;
                    nextHistory.push_back(std::move(entry));
                }
            }
        } else {
            if (entryIt != nextHistory.end()) {
                box.plateName = entryIt->lastValidPlate;
                box.vehicleLabel = entryIt->lastVehicleLabel;
                entryIt->lastSeenFrameSeq = frameSeq;
            } else {
                box.plateName.clear();
                box.vehicleLabel.clear();
            }
        }
    }
    m_plateHistory = std::move(nextHistory);
}

DetectResult MainWindow::makeDisplayDetectResult(int currentFrameSeq) const
{
    if (m_stableDetectFrameSeq < 0 || m_stableDetectResult.boxes.empty())
        return DetectResult();
    const qint64 resultAgeMs = m_detectResultTimer.isValid() ? m_detectResultTimer.elapsed() : 0;
    const int resultAgeFrames = currentFrameSeq - m_stableDetectFrameSeq;
    if (resultAgeFrames < 0 || resultAgeFrames > kDisplayResultMaxAgeFrames ||
        resultAgeMs > kDisplayResultMaxAgeMs)
        return DetectResult();
    return m_stableDetectResult;
}

void MainWindow::onAutoBrightnessToggled(bool enabled)
{
    m_autoBrightnessFrameCounter = 0;
    m_lastSceneLuma = 0;
    if (!enabled) {
        m_currentBrightnessLevel = 0;
        if (m_device && m_device->isOpened())
            m_device->pioWriteReg(kPioBrightnessLevelOffset, 0);
        return;
    }
    if (m_device && m_device->isOpened())
        m_device->pioWriteReg(kPioBrightnessLevelOffset, 0);
}

// ===================================================================
//  设备控制
// ===================================================================
void MainWindow::onOpenDevice()
{
    if (m_device->isOpened()) { onCloseDevice(); return; }
    if (!m_device->openDevice()) {
        qWarning() << "[device] openDevice failed";
        reportCriticalError("错误",
            "无法打开 PCIe 设备!\n请检查:\n1. 驱动是否加载\n2. FPGA 是否上电\n3. 权限是否正确");
        return;
    }
    // A previous process may have been killed before its destructor stopped
    // FPGA DMA.  Stop it before allocating/programming a fresh DMA mapping.
    if (!m_device->stopCaptureChecked(300)) {
        qWarning() << "[capture-guard] refusing DMA mapping because STOP was not confirmed";
        m_device->closeDevice();
        return;
    }
    if (!m_device->setupDma()) {
        qWarning() << "[device] setupDma failed";
        reportCriticalError("错误", "DMA 映射失败!");
        m_device->closeDevice();
        return;
    }
    if (!m_device->programFpgaDmaBases()) {
        qWarning() << "[device] programFpgaDmaBases failed";
        reportCriticalError("错误", "FPGA DMA 基地址配置失败!");
        m_device->closeDevice();
        return;
    }
    m_device->setFpgaInputSource(m_fpgaInputSource);
    m_device->tryMmapDma();

    updateDeviceInfo();
    m_btnOpenDevice->setText("关闭设备");
    m_btnOpenDevice->setStyleSheet(
        "QPushButton { background-color: #d83b01; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #e85d30; }");
    m_linkStatusLabel->setText("● 已连接");
    m_linkStatusLabel->setStyleSheet("font-size: 11px; color: #0a0; font-weight: bold;");
    setUIEnabled(true);
    m_statusLabel->setText("设备已连接 - 点击[开始采集]");
    refreshDedicatedDisplayHud();
}

void MainWindow::onCloseDevice()
{
    if (m_capture->isRunning()) onStopCapture();
    m_device->closeDevice();
    m_btnOpenDevice->setText("打开设备");
    m_btnOpenDevice->setStyleSheet(
        "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1a8cd8; }");
    m_linkStatusLabel->setText("● 未连接");
    m_linkStatusLabel->setStyleSheet("font-size: 11px; color: #888;");
    setUIEnabled(false);
    m_statusLabel->setText("就绪");
    showDisplayPlaceholder("等待采集...");
    m_inputFpsLabel->setText("输入: 0 FPS");
    m_displayFpsLabel->setText("显示: 0 FPS");
    updatePlateList(DetectResult());
    refreshDedicatedDisplayHud();
}

void MainWindow::updateDeviceInfo()
{
    const auto &info = m_device->deviceInfo();
    m_vendorIdEdit->setText(
        QString("0x%1").arg(info.vendor_id, 4, 16, QLatin1Char('0')).toUpper());
    m_linkSpeedEdit->setText(QString("Gen%1 (5.0 GT/s)").arg(info.link_speed));
    m_linkWidthEdit->setText(QString("×%1").arg(info.link_width));
    m_maxPayloadEdit->setText(QString("%1 bytes").arg(info.mps));
}

// ===================================================================
//  采集控制
// ===================================================================
void MainWindow::onStartCapture()
{
    if (m_capture->isRunning()) { onStopCapture(); return; }
    m_inputFPS = 0.0; m_displayFPS = 0.0;
    m_displayFrameCount = 0;
    m_lastFrameSeq = -1;
    m_lastDisplaySrc = nullptr;
    m_cachedDisplayImage = QImage();
    m_hasCachedDisplayImage = false;
    m_cachedDisplayFrameSeq = -1;
    m_lastDisplayedPlatePairSeq = -1;
    m_stableDetectFrameSeq = -1;
    m_stableDetectResult = DetectResult();
    resetPlateHistory();
    m_autoBrightnessFrameCounter = 0;
    m_currentBrightnessLevel = 0; m_lastSceneLuma = 0;
    m_displayFpsTimer.restart();
    m_detectResultTimer.invalidate();

    m_displayTimer->stop();
    m_frameBuffer->reset();
    if (!m_device->rearmFpgaInputSource(m_fpgaInputSource)) {
        qWarning() << "[fpga-source] startup rearm failed; using direct selection";
        m_device->setFpgaInputSource(m_fpgaInputSource);
    }
    // Always refresh the four physical destinations immediately before
    // START. A killed/restarted userspace process can receive a new coherent
    // DMA allocation while the FPGA still holds the previous process's
    // addresses. Programming only in onOpenDevice leaves a large window for
    // an endpoint/source rearm to retain stale capture state.
    if (!m_device->programFpgaDmaBases()) {
        qWarning() << "[dma-config] refresh before capture failed";
        reportCriticalError(QStringLiteral("DMA error"),
                            QStringLiteral("Failed to refresh FPGA DMA addresses"));
        return;
    }
    usleep(1000);
    if (!m_device->startCaptureChecked(150)) {
        qWarning() << "[capture-guard] refusing capture thread because START was not confirmed";
        reportCriticalError(QStringLiteral("DMA error"),
                            QStringLiteral("FPGA START command was not confirmed"));
        return;
    }
    m_capture->start();
    m_device->pioWriteReg(kPioBrightnessLevelOffset, 0);
    // PCIe frames drive onDisplayUpdate() through frameReady.  Keeping the
    // 16 ms network fallback timer active here adds a second display producer,
    // which can race the per-frame page flip during source changes.
    m_displayTimer->stop();

    m_btnAiDetect->setEnabled(true);
    m_btnYoloDetect->setEnabled(true);
    m_btnDetect3d->setEnabled(true);
    m_currentSource = InputSource::PCIE;
    m_sourceName = fpgaSourceDisplayName(m_fpgaInputSource);

    m_btnStartCapture->setText("停止采集");
    m_btnStartCapture->setStyleSheet(
        "QPushButton { background-color: #d83b01; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #e85d30; }");
    m_btnOpenDevice->setEnabled(false);
    m_statusLabel->setText("采集中...");
    refreshDedicatedDisplayHud();
}

void MainWindow::onStopCapture()
{
    m_displayTimer->stop();
    m_capture->stopCapture();
    m_device->pioWriteReg(kPioBrightnessLevelOffset, 0);
    (void)m_device->stopCaptureChecked(300);
    m_currentBrightnessLevel = 0; m_lastSceneLuma = 0;
    m_autoBrightnessFrameCounter = 0;
    m_displayFPS = 0.0;
    m_displayFrameCount = 0;
    m_stableDetectFrameSeq = -1;
    m_lastDisplaySrc = nullptr;
    m_cachedDisplayImage = QImage();
    m_hasCachedDisplayImage = false;
    m_cachedDisplayFrameSeq = -1;
    m_stableDetectResult = DetectResult();
    resetPlateHistory();
    usleep(2000);
    m_capture->wait(3000);

    if (m_detectionEnabled) {
        onToggleDetection(false);
        m_btnAiDetect->setText("[A] 车牌识别");
        m_btnAiDetect->setStyleSheet(
            "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #1a8cd8; }"
            "QPushButton:disabled { background-color: #555; }");
    }
    m_btnAiDetect->setEnabled(false);

    m_adasEnabled = false;
    if (m_detect3dEnabled)
        onToggleDetect3d(false);
    if (m_yoloEnabled)
        onToggleYoloDetection(false);
    m_btnYoloDetect->setText("[6] 自动驾驶辅助");
    m_btnYoloDetect->setStyleSheet(
        "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1a8cd8; }"
        "QPushButton:disabled { background-color: #555; }");
    m_btnYoloDetect->setEnabled(false);

    m_crossingFeatureEnabled = false;
    if (m_trafficEnabled)
        onToggleTrafficDetection(false);
    m_yieldFeatureEnabled = false;
    updateSharedYieldYoloBackend();
    m_btnDetect3d->setText("[7] 斑马线礼让检测");
    m_btnDetect3d->setStyleSheet(
        "QPushButton { background-color: #006c67; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #008c84; }"
        "QPushButton:disabled { background-color: #555; }");
    m_btnDetect3d->setEnabled(false);

    if (m_helmetEnabled)
        onToggleHelmetDetection(false);

    m_btnStartCapture->setText("开始采集");
    m_btnStartCapture->setStyleSheet(
        "QPushButton { background-color: #107c10; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1a9c1a; }");
    m_btnOpenDevice->setEnabled(true);
    m_statusLabel->setText("采集已停止");
    showDisplayPlaceholder("等待采集...");
    m_displayFpsLabel->setText("显示: 0 FPS");
    refreshDedicatedDisplayHud();
}

// ===================================================================
//  帧处理 — 显示与推理完全解耦
// ===================================================================
void MainWindow::onFrameReady(int frameSeq, qint64 captureReadyUs)
{
    // PCIe capture signals arrive per frame; process immediately to remove
    // the up-to-8ms polling delay and the resulting vsync phase misses.
    const qint64 guiReceiveUs =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    if (captureReadyUs > 0 && guiReceiveUs >= captureReadyUs) {
        const qint64 queueUs = guiReceiveUs - captureReadyUs;
        m_captureSignalQueueTotalUs += queueUs;
        m_captureSignalQueueMaxUs = qMax(m_captureSignalQueueMaxUs, queueUs);
        ++m_captureSignalQueueCount;
    }
    if (m_lastCaptureSignalSeq >= 0 && frameSeq > m_lastCaptureSignalSeq + 1)
        m_captureSignalGapFrames +=
            static_cast<quint64>(frameSeq - m_lastCaptureSignalSeq - 1);
    m_lastCaptureSignalSeq = frameSeq;
    onDisplayUpdate();
}

void MainWindow::setDisplayDetect3dOverlay(const YoloDetectResult &result,
                                           const char *const *classNames,
                                           int numClasses)
{
    if (m_eglfsVideoWindow)
        m_eglfsVideoWindow->setDetect3dOverlay(result, classNames, numClasses);
    else if (m_videoWidget)
        m_videoWidget->setDetect3dOverlay(result, classNames, numClasses);
}

void MainWindow::clearDisplayDetect3dOverlay()
{
    if (m_eglfsVideoWindow)
        m_eglfsVideoWindow->clearDetect3dOverlay();
    else if (m_videoWidget)
        m_videoWidget->clearDetect3dOverlay();
}

void MainWindow::onDisplayUpdate()
{
    const qint64 nowUs = m_dispProfTimer.nsecsElapsed() / 1000;
    if (m_lastDispUpdateUs != 0) {
        const qint64 pollUs = nowUs - m_lastDispUpdateUs;
        m_dispPollTotalUs += pollUs;
        m_dispPollMaxUs = qMax(m_dispPollMaxUs, pollUs);
    }
    m_lastDispUpdateUs = nowUs;
    ++m_dispPollCount;

    int frameSeq = -1;
    const uint8_t *frameView = nullptr;
    bool hasFrame = m_frameBuffer->acquireLatestFrameView(frameView, frameSeq);
    bool hasNewDisplayFrame = false;
    const qint64 workStartUs = m_dispProfTimer.nsecsElapsed() / 1000;
    if (hasFrame && frameSeq != m_cachedDisplayFrameSeq) {
        m_lastFrameSeq = frameSeq;

        // The capture thread already copied the uncached DMA frame into a
        // cached FrameBuffer slot (valid for ~3 frame periods), so GL can read
        // this memory directly.  Wrap the slot instead of copying 1.84 MB a
        // second time on the UI thread; that extra copy starved input events.
        m_lastDisplaySrc = frameView;
        updateAutoBrightnessControl();

        m_cachedDisplayImage = QImage(frameView,
                                      FRAME_WIDTH, FRAME_HEIGHT,
                                      FRAME_WIDTH * 2, QImage::Format_RGB16);
        m_hasCachedDisplayImage = !m_cachedDisplayImage.isNull();
        m_cachedDisplayFrameSeq = frameSeq;
        hasNewDisplayFrame = m_hasCachedDisplayImage;
    }

    // Do not resubmit stale textures when no new input frame arrived.
    if (hasNewDisplayFrame) {
        // Inference callbacks submit overlays only when a result changes.  Do
        // not recopy both result vectors and request extra GL updates.
        presentDisplayFrame(m_cachedDisplayImage);

        // 显示 FPS 统计
        m_displayFrameCount++;
        const qint64 displayElapsed = m_displayFpsTimer.elapsed();
        if (displayElapsed >= 1000) {
            m_displayFPS = m_displayFrameCount * 1000.0 / displayElapsed;
            m_displayFpsLabel->setText(QString("显示: %1 FPS").arg(m_displayFPS, 0, 'f', 1));
            if (m_displayFPS >= 50.0)
                m_displayFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #a0a;");
            else if (m_displayFPS >= 20.0)
                m_displayFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #fa0;");
            else
                m_displayFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #f00;");
            m_displayFrameCount = 0;
            m_displayFpsTimer.restart();
            refreshDedicatedDisplayHud();
        }
    }
    const qint64 workUs = m_dispProfTimer.nsecsElapsed() / 1000 - workStartUs;
    if (hasNewDisplayFrame) {
        m_dispWorkTotalUs += workUs;
        m_dispWorkMaxUs = qMax(m_dispWorkMaxUs, workUs);
        ++m_dispWorkCount;
    }

    // ---- 1 Hz display instrumentation (output even without new frames) ----
    if (nowUs - m_dispProfWindowUs >= 1000000) {
        qWarning("[disp-prof] poll_avg_us=%lld poll_max_us=%lld "
                 "work_avg_us=%lld work_max_us=%lld frames=%d "
                 "signal_queue_avg_us=%lld signal_queue_max_us=%lld "
                 "signal_events=%d signal_seq_gaps=%llu",
                 m_dispPollCount ? m_dispPollTotalUs / m_dispPollCount : 0,
                 m_dispPollMaxUs,
                 m_dispWorkCount ? m_dispWorkTotalUs / m_dispWorkCount : 0,
                 m_dispWorkMaxUs,
                 m_dispWorkCount,
                 m_captureSignalQueueCount
                     ? m_captureSignalQueueTotalUs / m_captureSignalQueueCount : 0,
                 m_captureSignalQueueMaxUs,
                 m_captureSignalQueueCount,
                 static_cast<unsigned long long>(m_captureSignalGapFrames));
        m_dispProfWindowUs = nowUs;
        m_dispPollTotalUs = 0;
        m_dispPollMaxUs = 0;
        m_dispPollCount = 0;
        m_dispWorkTotalUs = 0;
        m_dispWorkMaxUs = 0;
        m_dispWorkCount = 0;
        m_captureSignalQueueTotalUs = 0;
        m_captureSignalQueueMaxUs = 0;
        m_captureSignalQueueCount = 0;
        m_captureSignalGapFrames = 0;
    }
}


// ===================================================================
//  推理回调 — Worker 完成一轮推理
// ===================================================================
void MainWindow::onFastDetectionReady(const DetectResult &result, int frameSeq)
{
    if (!m_detectionEnabled || frameSeq < m_stableDetectFrameSeq)
        return;

    DetectResult displayResult = result;
    applyPlateHistory(displayResult, frameSeq);
    m_stableDetectResult = std::move(displayResult);
    m_stableDetectFrameSeq = frameSeq;
    m_detectResultTimer.restart();

    const char *const *names = m_plateClassNames.empty()
        ? nullptr : m_plateClassNames.data();
    setDisplayObbOverlay(m_stableDetectResult, names,
                         static_cast<int>(m_plateClassNames.size()));
}

void MainWindow::onAsyncInferenceDone(int frameSeq)
{
    Q_UNUSED(frameSeq);
    if (!m_detectionEnabled) return;

    // 从 Worker 读取原始检测结果 (用于 plate history 等后处理)
    DetectResult result;
    int sourceSeq = -1;
    if (m_detectionWorker->getLatestResult(result, sourceSeq)) {
        applyPlateHistory(result, sourceSeq);
        updatePlateList(result);
        sendObbRecognitionResult(result, sourceSeq);
        m_stableDetectResult = std::move(result);
        m_stableDetectFrameSeq = sourceSeq;
        m_detectResultTimer.restart();
        const char *const *names = m_plateClassNames.empty()
            ? nullptr : m_plateClassNames.data();
        setDisplayObbOverlay(m_stableDetectResult, names,
                             static_cast<int>(m_plateClassNames.size()));
    }

    refreshPreprocessPreview();
    refreshDedicatedDisplayHud();
}

void MainWindow::onInferenceStatus(const QString &msg)
{
    m_statusLabel->setText(msg);
    refreshDedicatedDisplayHud();
}

// ===================================================================
//  统计 / 错误
// ===================================================================
void MainWindow::onStatsUpdated(unsigned int inputFrames, unsigned int processedFrames,
                                unsigned int dropped, double inputFps, double processFps)
{
    if (m_currentSource != InputSource::PCIE) return;
    Q_UNUSED(inputFrames); Q_UNUSED(processedFrames);
    Q_UNUSED(dropped); Q_UNUSED(processFps);
    m_inputFPS      = inputFps;

    m_inputFpsLabel->setText(QString("输入: %1 FPS").arg(inputFps, 0, 'f', 1));

    if (inputFps >= 50.0)
        m_inputFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #08c;");
    else if (inputFps >= 25.0)
        m_inputFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #fa0;");
    else
        m_inputFpsLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #f00;");
    refreshDedicatedDisplayHud();
}

void MainWindow::onCaptureError(const QString &msg)
{
    m_statusLabel->setText("错误: " + msg);
    refreshDedicatedDisplayHud();
    onStopCapture();
}

void MainWindow::onFrameTimeout()
{
    m_statusLabel->setText("警告: 帧超时");
    m_linkStatusLabel->setText("● 帧超时");
    m_linkStatusLabel->setStyleSheet("font-size: 11px; color: #f00; font-weight: bold;");
    refreshDedicatedDisplayHud();
}

void MainWindow::onFrameResumed()
{
    m_statusLabel->setText("采集中...");
    m_linkStatusLabel->setText("● 已连接");
    m_linkStatusLabel->setStyleSheet("font-size: 11px; color: #0a0; font-weight: bold;");
    refreshDedicatedDisplayHud();
}

// ===================================================================
//  保存图像
// ===================================================================
void MainWindow::onSaveImage()
{
    QPixmap pix = m_videoWidget->grab();
    if (pix.isNull()) { QMessageBox::information(this, "提示", "当前无图像可保存"); return; }
    QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", "/userdata");
    if (dir.isEmpty()) return;
    static int saveNum = 0;
    QString filePath = QString("%1/capture_%2.png")
                       .arg(dir).arg(saveNum++, 5, 10, QLatin1Char('0'));
    if (pix.save(filePath, "PNG"))
        m_statusLabel->setText("已保存: " + filePath);
    else
        QMessageBox::warning(this, "错误", "保存失败: " + filePath);
}

void MainWindow::onRemoteSaveImage()
{
    if (!m_hasCachedDisplayImage || m_cachedDisplayImage.isNull()) {
        m_statusLabel->setText(QStringLiteral("远程保存失败：当前没有视频帧"));
        return;
    }

    QImage image;
    if (m_eglfsVideoWindow && m_eglfsVideoWindow->isExposed())
        image = m_eglfsVideoWindow->grabFramebuffer();
    if (image.isNull() && m_videoWidget)
        image = m_videoWidget->grab().toImage();
    if (image.isNull())
        image = m_cachedDisplayImage.copy();

    QDir saveDir(QStringLiteral("/userdata/captures"));
    if (!saveDir.exists() && !saveDir.mkpath(QStringLiteral("."))) {
        m_statusLabel->setText(QStringLiteral("远程保存失败：无法创建 /userdata/captures"));
        return;
    }

    const QString fileName = QStringLiteral("capture_%1.png").arg(
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    const QString filePath = saveDir.filePath(fileName);
    if (image.save(filePath, "PNG")) {
        m_statusLabel->setText(QStringLiteral("远程保存成功：") + filePath);
    } else {
        m_statusLabel->setText(QStringLiteral("远程保存失败：") + filePath);
    }
    refreshDedicatedDisplayHud();
}

// ===================================================================
//  AI 检测开关
// ===================================================================
void MainWindow::onAiDetectClick()
{
    if (m_detectionEnabled) {
        onToggleDetection(false);
        m_btnAiDetect->setText("[A] 车牌识别");
        m_btnAiDetect->setStyleSheet(
            "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #1a8cd8; }"
            "QPushButton:disabled { background-color: #555; }");
    } else {
        onToggleDetection(true);
        if (m_detectionEnabled) {
            m_btnAiDetect->setText("[A] 停止车牌识别");
            m_btnAiDetect->setStyleSheet(
                "QPushButton { background-color: #d83b01; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
                "QPushButton:hover { background-color: #e85d30; }"
                "QPushButton:disabled { background-color: #555; }");
        }
    }
}

void MainWindow::onToggleDetection(bool enabled)
{
    resetPlateHistory();

    if (enabled) {
        clearDisplayObbOverlay();
        // 加载模型 (如果尚未加载)
        if (!m_detectionWorker->isLoaded()) {
            const QString modelPath = resolveFirstExistingPath({
                "/userdata/model/yolov8n_obb.rknn",
                "/userdata/model/yolov8n_obb_plate.rknn"
            });
            const QString labelPath = resolveFirstExistingPath({
                "/userdata/model/yolov8_obb_labels_list.txt",
                "/userdata/model/yolov8_obb_plate_labels_list.txt"
            });

            ObbModelParams params;
            // 多车牌场景：保留置信度稍低的第二块车牌，并避免相邻同类车牌被 NMS 提前压掉。
            // 真正的同一车牌重复框在 LPR 完成后按字符识别结果再次去重。
            params.boxThreshold = 0.25f;
            params.nmsThreshold = 0.60f;

            if (!m_detectionWorker->loadModel(modelPath.toStdString().c_str(),
                                              labelPath.toStdString().c_str(),
                                              params)) {
                QMessageBox::warning(this, "AI 检测",
                    QString("模型加载失败!\n%1\n%2").arg(modelPath).arg(labelPath));
                return;
            }

            // 缓存类别名称
            m_plateClassNames.clear();
            m_plateClassNames.reserve(m_detectionWorker->numClasses());
            for (int i = 0; i < m_detectionWorker->numClasses(); ++i)
                m_plateClassNames.push_back(m_detectionWorker->className(i));

            // 加载 LPRNet 模型
            if (!m_detectionWorker->isLprLoaded()) {
                if (!m_detectionWorker->loadLprModels(
                        "/userdata/model/lprnet_blue.rknn",
                        "/userdata/model/lprnet.rknn",
                        "/userdata/model/doubleyellow.rknn")) {
                    QMessageBox::warning(this, "LPRNet",
                        "车牌识别模型加载失败!");
                    return;
                }
            }

            // SCI + FFDNet 模型不在此处加载，由 P 键/按钮首次按下时按需加载

            m_statusLabel->setText("车牌检测+字符识别已加载");
        }

        // 启动推理循环
        QMetaObject::invokeMethod(m_detectionWorker, "startInference",
                                  Qt::QueuedConnection);

        m_stableDetectResult = DetectResult();
        m_stableDetectFrameSeq = -1;
        m_detectResultTimer.invalidate();
    } else {
        // 停止推理循环, 释放模型
        QMetaObject::invokeMethod(m_detectionWorker, "stopInference",
                                  Qt::QueuedConnection);

        clearDisplayObbOverlay();
        m_plateClassNames.clear();
        m_stableDetectResult = DetectResult();
        m_stableDetectFrameSeq = -1;
        m_detectResultTimer.invalidate();
    }

    m_detectionEnabled = enabled;
    refreshDedicatedDisplayHud();
}

// ===================================================================
//  YOLOv8 检测开关 (第二路, 标准轴对齐框)
// ===================================================================
void MainWindow::onYoloDetectClick()
{
    // Keep the existing remote YOLO command compatible with the new feature
    // layout: it now toggles the complete key-6 assistance feature too.
    onDetect3dClick();
}

void MainWindow::onPreprocessClick()
{
    requestPreprocessMask(m_preprocessMask == PlatePreprocessNone
        ? PlatePreprocessAll : PlatePreprocessNone);
}

void MainWindow::onPreprocessStageClick(int stage)
{
    if (stage < 1 || stage > 3 || m_preprocessRequestPending) return;
    requestPreprocessMask(m_preprocessMask ^ (1 << (stage - 1)));
}

void MainWindow::onEnhancedLprClick()
{
    if (!m_detectionWorker) return;
    m_enhancedLprEnabled = !m_enhancedLprEnabled;
    if (!m_enhancedLprEnabled) {
        m_lastEnhancedPlateSeq = -1;
    }
    QMetaObject::invokeMethod(
        m_detectionWorker, "setEnhancedLprEnabled", Qt::QueuedConnection,
        Q_ARG(bool, m_enhancedLprEnabled));
    refreshDedicatedDisplayHud();
}

void MainWindow::requestPreprocessMask(int enabledMask)
{
    if (!m_detectionWorker || m_preprocessRequestPending) return;

    enabledMask &= PlatePreprocessAll;
    m_preprocessRequestedMask = enabledMask;
    m_preprocessRequestPending = true;
    updatePreprocessControls();
    refreshDedicatedDisplayHud();

    const QString sciModelPath = resolveFirstExistingPath(QStringList()
        << QStringLiteral("/userdata/model/sci_plate_32x2_fp16_rk3568.rknn")
        << QStringLiteral("/mnt/usb/model/sci_plate_32x2_fp16_rk3568.rknn"));
    const QString ffdnetModelPath = resolveFirstExistingPath(QStringList()
        << QStringLiteral("/userdata/model/ffdnet_color_rk3568.rknn")
        << QStringLiteral("/mnt/usb/model/ffdnet_color_rk3568.rknn"));
    const QString mimoModelPath = resolveFirstExistingPath(QStringList()
        << QStringLiteral("/userdata/model/mimo_plus_stage5_188x48_fp16_rk3568.rknn")
        << QStringLiteral("/mnt/usb/model/mimo_plus_stage5_188x48_fp16_rk3568.rknn"));
    const bool queued = QMetaObject::invokeMethod(
        m_detectionWorker, "configurePreprocess", Qt::QueuedConnection,
        Q_ARG(int, enabledMask),
        // Fixed serial cascade; the final image remains display-only.
        Q_ARG(QString, sciModelPath),
        Q_ARG(QString, ffdnetModelPath),
        Q_ARG(QString, mimoModelPath));
    if (!queued) {
        onPreprocessStateChanged(
            m_preprocessMask, QStringLiteral("车牌串联增强请求入队失败"));
    }
}

void MainWindow::onPreprocessStateChanged(int enabledMask, const QString &message)
{
    m_preprocessRequestPending = false;
    m_preprocessMask = enabledMask & PlatePreprocessAll;
    m_preprocessRequestedMask = m_preprocessMask;
    m_preprocessEnabled = m_preprocessMask != PlatePreprocessNone;
    m_statusLabel->setText(message);

    // Never leave a result produced by the previous switch mask on screen.
    // The next throttled cascade result will repopulate the preview.
    m_lastDisplayedPlatePairSeq = -1;
    if (m_eglfsVideoWindow) {
        const QImage empty;
        m_eglfsVideoWindow->setPlateComparison(empty, empty, empty, empty);
    }
    updatePreprocessControls();
    refreshDedicatedDisplayHud();
}

void MainWindow::updatePreprocessControls()
{
    if (!m_btnPreprocess) return;
    m_btnPreprocess->setEnabled(!m_preprocessRequestPending);
    if (m_preprocessRequestPending) {
        m_btnPreprocess->setText(QStringLiteral("正在应用串联开关..."));
    } else {
        m_btnPreprocess->setText(QString("[1]sci:%1 [2]ffdnet:%2 [3]deblur:%3")
            .arg(m_preprocessMask & PlatePreprocessSci ? "ON" : "OFF")
            .arg(m_preprocessMask & PlatePreprocessFfdnet ? "ON" : "OFF")
            .arg(m_preprocessMask & PlatePreprocessDeblur ? "ON" : "OFF"));
    }
    if (m_preprocessEnabled) {
        m_btnPreprocess->setStyleSheet(
            "QPushButton { background-color: #d83b01; color: white; font-size: 12px; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #e85d30; }"
            "QPushButton:disabled { background-color: #805030; }");
    } else {
        m_btnPreprocess->setStyleSheet(
            "QPushButton { background-color: #8B4513; color: white; font-size: 12px; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #A0522D; }"
            "QPushButton:disabled { background-color: #555; }");
    }
}

void MainWindow::onToggleYoloDetection(bool enabled)
{
    if (enabled) {
        clearDisplayYoloOverlay();
        if (!m_yoloWorker->isLoaded()) {
            YoloV8Params params;
            params.numClasses = 80;
            params.boxThreshold = 0.30f;
            params.nmsThreshold = 0.45f;
            params.modelWidth = 640;
            params.modelHeight = 384;
            // Forward-camera lane departure controls. These environment
            // variables allow board-side calibration without rebuilding.
            const QByteArray laneDisabled = qgetenv("HS_CAMERA_DISABLE_LANE_DEPARTURE");
            params.laneDepartureEnabled = !(laneDisabled == "1" ||
                                             laneDisabled.toLower() == "true");
            bool centerOk = false;
            const float centerRatio = qgetenv("HS_CAMERA_CENTER_RATIO").toFloat(&centerOk);
            if (centerOk) params.cameraCenterRatio = centerRatio;
            bool thresholdOk = false;
            const float threshold = qgetenv("HS_CAMERA_LANE_DEPARTURE_THRESHOLD")
                                        .toFloat(&thresholdOk);
            if (thresholdOk) params.laneDepartureThreshold = threshold;
            bool framesOk = false;
            const int confirmFrames = qgetenv("HS_CAMERA_LANE_CONFIRM_FRAMES")
                                          .toInt(&framesOk);
            if (framesOk) params.laneDepartureConfirmFrames = confirmFrames;

            const QString modelPath = resolveFirstExistingPath(QStringList()
                << QStringLiteral("/userdata/model/yolopv2_384x640_int8.rknn")
                << QStringLiteral("/mnt/usb/model/yolopv2_384x640_int8.rknn")
                << QStringLiteral("./yolopv2_384x640_int8.rknn")
                << QStringLiteral("/userdata/model/yolopv2_384x640_fp16.rknn")
                << QStringLiteral("/mnt/usb/model/yolopv2_384x640_fp16.rknn")
                << QStringLiteral("./yolopv2_384x640_fp16.rknn"));
            const QByteArray modelPathBytes = QFile::encodeName(modelPath);
            if (!m_yoloWorker->loadModel(modelPathBytes.constData(), nullptr,
                                         params)) {
                QMessageBox::warning(
                    this, QStringLiteral("YOLOPv2"),
                    QStringLiteral("YOLOPv2 model load failed:\n%1")
                        .arg(modelPath));
                return;
            }

            m_yoloClassNames.clear();
            m_yoloClassNames.reserve(m_yoloWorker->numClasses());
            for (int i = 0; i < m_yoloWorker->numClasses(); ++i)
                m_yoloClassNames.push_back(m_yoloWorker->className(i));
        }

        QMetaObject::invokeMethod(m_yoloWorker, "startInference", Qt::QueuedConnection);
    } else {
        QMetaObject::invokeMethod(m_yoloWorker, "stopInference", Qt::QueuedConnection);
        clearDisplayYoloOverlay();
        m_yoloClassNames.clear();
    }

    m_yoloEnabled = enabled;
    refreshDedicatedDisplayHud();
}

void MainWindow::onYoloInferenceDone(int /*frameSeq*/)
{
    YoloDetectResult result;
    int sourceSeq = -1;
    const char *const *names = m_yoloClassNames.empty()
        ? nullptr : m_yoloClassNames.data();
    if (m_yoloEnabled && m_yoloWorker->getLatestResult(result, sourceSeq)) {
        sendYoloRecognitionResult(result, sourceSeq);
        setDisplayYoloOverlay(result, names,
                              static_cast<int>(m_yoloClassNames.size()));
    } else {
        clearDisplayYoloOverlay();
    }
}

void MainWindow::onYieldYoloDetectClick()
{
    const bool requested = !m_yieldFeatureEnabled;
    m_yieldFeatureEnabled = requested;
    updateSharedYieldYoloBackend();
    if (requested && !m_yieldYoloEnabled) {
        // Model/label loading failed. Do not show the feature as enabled.
        m_yieldFeatureEnabled = false;
        updateSharedYieldYoloBackend();
    }
    m_btnDetect3d->setText(m_yieldFeatureEnabled
        ? QStringLiteral("[7] 停止斑马线礼让检测")
        : QStringLiteral("[7] 斑马线礼让检测"));
    m_btnDetect3d->setStyleSheet(m_yieldFeatureEnabled
        ? "QPushButton { background-color: #d83b01; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
          "QPushButton:hover { background-color: #e85d30; }"
          "QPushButton:disabled { background-color: #555; }"
        : "QPushButton { background-color: #006c67; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
          "QPushButton:hover { background-color: #008c84; }"
          "QPushButton:disabled { background-color: #555; }");
    refreshDedicatedDisplayHud();
}

void MainWindow::updateSharedYieldYoloBackend()
{
    const bool needed = m_crossingFeatureEnabled || m_yieldFeatureEnabled;
    if (needed != m_yieldYoloEnabled)
        onToggleYieldYoloDetection(needed);
}

void MainWindow::updateTrafficCrossingAlert()
{
    if (!m_crossingFeatureEnabled || !m_trafficEnabled ||
        m_latestTrafficSourceSeq < 0) {
        return;
    }

    YoloDetectResult displayResult = m_latestTrafficForCrossing;
    const int frameGap = m_latestTrafficSourceSeq > m_latestYieldYoloSourceSeq
        ? m_latestTrafficSourceSeq - m_latestYieldYoloSourceSeq
        : m_latestYieldYoloSourceSeq - m_latestTrafficSourceSeq;
    const bool sameScene = m_latestYieldYoloSourceSeq >= 0 &&
                           frameGap <= kDisplayResultMaxAgeFrames;
    displayResult.redLightCrosswalkPedestrianAlert = sameScene &&
        hasRedLightCrosswalkPedestrian(m_latestTrafficForCrossing,
                                       m_latestYieldYoloForCrossing,
                                       m_yieldYoloClassNames);
    const char *const *names = m_trafficClassNames.empty()
        ? nullptr : m_trafficClassNames.data();
    setDisplayTrafficOverlay(displayResult, names,
                             static_cast<int>(m_trafficClassNames.size()));
}

void MainWindow::onToggleYieldYoloDetection(bool enabled)
{
    if (!m_yieldYoloWorker) return;

    if (enabled) {
        clearDisplayYieldYoloOverlay();
        if (!m_yieldYoloWorker->isLoaded()) {
            YoloV8Params params;
            // Keep ARM_final1's validated traffic-model thresholds.
            params.boxThreshold = 0.50f;
            params.nmsThreshold = 0.45f;
            params.laneDepartureEnabled = false;

            const QString modelPath = resolveFirstExistingPath(QStringList()
                << QStringLiteral("/userdata/model/yolov8_traffic.rknn")
                << QStringLiteral("/mnt/usb/model/yolov8_traffic.rknn")
                << QStringLiteral("./yolov8_traffic.rknn"));
            const QString labelPath = resolveFirstExistingPath(QStringList()
                << QStringLiteral("/userdata/model/yolov8_traffic_labels_list.txt")
                << QStringLiteral("/mnt/usb/model/yolov8_traffic_labels_list.txt")
                << QStringLiteral("./yolov8_traffic_labels_list.txt"));

            if (!QFileInfo::exists(modelPath) || !QFileInfo::exists(labelPath)) {
                const QString message = QStringLiteral(
                    "YOLOv8礼让模型或标签不存在:\n%1\n%2")
                    .arg(modelPath, labelPath);
                qWarning() << message;
                m_statusLabel->setText(message);
                if (!m_useDedicatedEglfsDisplay)
                    QMessageBox::warning(this, QStringLiteral("YOLOv8礼让"),
                                         message);
                refreshDedicatedDisplayHud();
                return;
            }

            const QByteArray modelBytes = QFile::encodeName(modelPath);
            const QByteArray labelBytes = QFile::encodeName(labelPath);
            if (!m_yieldYoloWorker->loadModel(modelBytes.constData(),
                                               labelBytes.constData(), params)) {
                const QString message = QStringLiteral(
                    "YOLOv8礼让模型加载失败:\n%1").arg(modelPath);
                qWarning() << message;
                m_statusLabel->setText(message);
                if (!m_useDedicatedEglfsDisplay)
                    QMessageBox::warning(this, QStringLiteral("YOLOv8礼让"),
                                         message);
                refreshDedicatedDisplayHud();
                return;
            }

            m_yieldYoloClassNames.clear();
            m_yieldYoloClassNames.reserve(m_yieldYoloWorker->numClasses());
            for (int i = 0; i < m_yieldYoloWorker->numClasses(); ++i)
                m_yieldYoloClassNames.push_back(
                    m_yieldYoloWorker->className(i));
        }
        QMetaObject::invokeMethod(m_yieldYoloWorker, "startInference",
                                  Qt::QueuedConnection);
    } else {
        QMetaObject::invokeMethod(m_yieldYoloWorker, "stopInference",
                                  Qt::QueuedConnection);
        clearDisplayYieldYoloOverlay();
        m_yieldYoloClassNames.clear();
        m_latestYieldYoloForCrossing = YoloDetectResult();
        m_latestYieldYoloSourceSeq = -1;
    }

    m_yieldYoloEnabled = enabled;
    refreshDedicatedDisplayHud();
}

void MainWindow::onYieldYoloInferenceDone(int /*frameSeq*/)
{
    YoloDetectResult result;
    int sourceSeq = -1;
    const char *const *names = m_yieldYoloClassNames.empty()
        ? nullptr : m_yieldYoloClassNames.data();
    if (m_yieldYoloEnabled &&
        (m_crossingFeatureEnabled || m_yieldFeatureEnabled) &&
        m_yieldYoloWorker->getLatestResult(result, sourceSeq)) {
        if (!m_yieldFeatureEnabled) {
            // Key 5 needs the shared YOLOv8 boxes, but the explicit
            // failure-to-yield warning belongs to key 7.
            result.yieldViolationAlert = false;
            for (YoloBox &box : result.boxes)
                box.yieldViolation = false;
        }
        m_latestYieldYoloForCrossing = result;
        m_latestYieldYoloSourceSeq = sourceSeq;
        updateTrafficCrossingAlert();
        sendYoloRecognitionResult(result, sourceSeq);
        setDisplayYieldYoloOverlay(
            result, names, static_cast<int>(m_yieldYoloClassNames.size()));
    } else {
        m_latestYieldYoloForCrossing = YoloDetectResult();
        m_latestYieldYoloSourceSeq = -1;
        updateTrafficCrossingAlert();
        clearDisplayYieldYoloOverlay();
    }
}

void MainWindow::onYoloStatus(const QString &msg)
{
    m_statusLabel->setText(msg);
    refreshDedicatedDisplayHud();
}

void MainWindow::onDetect3dClick()
{
    const bool requested = !m_adasEnabled;
    if (requested) {
        const bool yoloWasEnabled = m_yoloEnabled;
        if (!m_yoloEnabled)
            onToggleYoloDetection(true);
        if (!m_yoloEnabled) return;

        if (!m_detect3dEnabled)
            onToggleDetect3d(true);
        if (!m_detect3dEnabled) {
            if (!yoloWasEnabled)
                onToggleYoloDetection(false);
            return;
        }
        m_adasEnabled = true;
    } else {
        m_adasEnabled = false;
        if (m_detect3dEnabled)
            onToggleDetect3d(false);
        if (m_yoloEnabled)
            onToggleYoloDetection(false);
    }

    m_btnYoloDetect->setText(m_adasEnabled
        ? QStringLiteral("[6] 停止自动驾驶辅助")
        : QStringLiteral("[6] 自动驾驶辅助"));
    m_btnYoloDetect->setStyleSheet(m_adasEnabled
        ? "QPushButton { background-color: #d83b01; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
          "QPushButton:hover { background-color: #e85d30; }"
          "QPushButton:disabled { background-color: #555; }"
        : "QPushButton { background-color: #0078d4; color: white; font-size: 14px; font-weight: bold; border-radius: 4px; }"
          "QPushButton:hover { background-color: #1a8cd8; }"
          "QPushButton:disabled { background-color: #555; }");
    refreshDedicatedDisplayHud();
}

void MainWindow::onToggleDetect3d(bool enabled)
{
    if (!m_detect3dWorker) return;
    if (enabled) {
        clearDisplayDetect3dOverlay();
        if (!m_detect3dWorker->isLoaded()) {
            YoloV8Params params;
            params.numClasses = 3;
            params.boxThreshold = 0.30f;
            params.modelWidth = 1280;
            params.modelHeight = 416;
            params.laneDepartureEnabled = false;
            const QString modelPath = resolveFirstExistingPath(QStringList()
                << QStringLiteral("/userdata/model/chen-yolo26n-3d-seed0-rk3568-fp16.rknn")
                << QStringLiteral("/mnt/usb/model/chen-yolo26n-3d-seed0-rk3568-fp16.rknn")
                << QStringLiteral("./models/detect3d/chen-yolo26n-3d-seed0-rk3568-fp16.rknn")
                << QStringLiteral("./chen-yolo26n-3d-seed0-rk3568-fp16.rknn"));
            const QByteArray pathBytes = QFile::encodeName(modelPath);
            if (!m_detect3dWorker->loadModel(pathBytes.constData(), nullptr, params)) {
                QMessageBox::warning(this, QStringLiteral("Detect3D"),
                    QStringLiteral("Detect3D model load failed:\n%1").arg(modelPath));
                return;
            }
            m_detect3dClassNames.clear();
            for (int i = 0; i < m_detect3dWorker->numClasses(); ++i)
                m_detect3dClassNames.push_back(m_detect3dWorker->className(i));
        }
        QMetaObject::invokeMethod(m_detect3dWorker, "startInference", Qt::QueuedConnection);
    } else {
        QMetaObject::invokeMethod(m_detect3dWorker, "stopInference", Qt::QueuedConnection);
        clearDisplayDetect3dOverlay();
        m_detect3dClassNames.clear();
    }
    m_detect3dEnabled = enabled;
    refreshDedicatedDisplayHud();
}

void MainWindow::onDetect3dInferenceDone(int /*frameSeq*/)
{
    YoloDetectResult result;
    int sourceSeq = -1;
    const char *const *names = m_detect3dClassNames.empty()
        ? nullptr : m_detect3dClassNames.data();
    if (m_detect3dEnabled && m_detect3dWorker->getLatestResult(result, sourceSeq))
        setDisplayDetect3dOverlay(result, names,
                                  static_cast<int>(m_detect3dClassNames.size()));
    else
        clearDisplayDetect3dOverlay();
}

void MainWindow::onHelmetDetectClick()
{
    onToggleHelmetDetection(!m_helmetEnabled);
}

void MainWindow::onToggleHelmetDetection(bool enabled)
{
    if (!m_helmetWorker) return;
    if (enabled) {
        clearDisplayHelmetOverlay();
        if (!m_helmetWorker->isLoaded()) {
            YoloV8Params params;
            params.numClasses = 3;
            params.boxThreshold = 0.25f;
            params.nmsThreshold = 0.45f;
            params.modelWidth = 640;
            params.modelHeight = 640;
            const QString helmetModelPath = resolveFirstExistingPath(QStringList()
                << QStringLiteral("/userdata/model/helmet.rknn")
                << QStringLiteral("/mnt/usb/model/helmet.rknn"));
            const QString helmetLabelPath = resolveFirstExistingPath(QStringList()
                << QStringLiteral("/userdata/model/helmet_labels_list.txt")
                << QStringLiteral("/mnt/usb/model/helmet_labels_list.txt"));
            const QByteArray helmetModelBytes = QFile::encodeName(helmetModelPath);
            const QByteArray helmetLabelBytes = QFile::encodeName(helmetLabelPath);
            if (!m_helmetWorker->loadModel(
                    helmetModelBytes.constData(), helmetLabelBytes.constData(), params)) {
                m_statusLabel->setText(QStringLiteral("helmet RKNN load failed: %1").arg(helmetModelPath));
                return;
            }
            m_helmetClassNames.clear();
            for (int i = 0; i < 3; ++i)
                m_helmetClassNames.push_back(m_helmetWorker->className(i));
        }
        m_helmetEnabled = true;
        QMetaObject::invokeMethod(m_helmetWorker, "startInference",
                                  Qt::QueuedConnection);
    } else {
        m_helmetEnabled = false;
        QMetaObject::invokeMethod(m_helmetWorker, "stopInference",
                                  Qt::QueuedConnection);
        clearDisplayHelmetOverlay();
        m_helmetClassNames.clear();
    }
    refreshDedicatedDisplayHud();
}

void MainWindow::onHelmetInferenceDone(int /*frameSeq*/)
{
    YoloDetectResult result;
    int sourceSeq = -1;
    if (m_helmetEnabled &&
        m_helmetWorker->getLatestResult(result, sourceSeq)) {
        result.helmetNoHelmetAlert = hasNoHelmetDetection(result);
        const char *const *names = m_helmetClassNames.empty()
            ? nullptr : m_helmetClassNames.data();
        setDisplayHelmetOverlay(result, names,
                                static_cast<int>(m_helmetClassNames.size()));
    } else {
        clearDisplayHelmetOverlay();
    }
}

void MainWindow::onHelmetStatus(const QString &msg)
{
    m_statusLabel->setText(msg);
    refreshDedicatedDisplayHud();
}

void MainWindow::onTrafficDetectClick()
{
    const bool requested = !m_crossingFeatureEnabled;
    if (requested) {
        onToggleTrafficDetection(true);
        if (!m_trafficEnabled) return;
        m_crossingFeatureEnabled = true;
        updateSharedYieldYoloBackend();
        if (!m_yieldYoloEnabled) {
            m_crossingFeatureEnabled = false;
            onToggleTrafficDetection(false);
            updateSharedYieldYoloBackend();
            return;
        }
    } else {
        m_crossingFeatureEnabled = false;
        onToggleTrafficDetection(false);
        updateSharedYieldYoloBackend();
    }
    refreshDedicatedDisplayHud();
}

void MainWindow::onToggleTrafficDetection(bool enabled)
{
    if (!m_trafficWorker) return;
    if (enabled) {
        clearDisplayTrafficOverlay();
        if (!m_trafficWorker->isLoaded()) {
            const QString modelPath = resolveFirstExistingPath(QStringList()
                << QStringLiteral("/userdata/model/traffic_signal_crosswalk_int8.rknn")
                << QStringLiteral("/mnt/usb/model/traffic_signal_crosswalk_int8.rknn"));
            const QByteArray modelPathBytes = QFile::encodeName(modelPath);
            if (!m_trafficWorker->loadModel(modelPathBytes.constData())) {
                m_statusLabel->setText(
                    QStringLiteral("traffic RKNN load failed: %1").arg(modelPath));
                return;
            }
            m_trafficClassNames.clear();
            for (int i = 0; i < 3; ++i)
                m_trafficClassNames.push_back(m_trafficWorker->className(i));
        }
        m_trafficEnabled = true;
        QMetaObject::invokeMethod(m_trafficWorker, "startInference",
                                  Qt::QueuedConnection);
    } else {
        m_trafficEnabled = false;
        QMetaObject::invokeMethod(m_trafficWorker, "stopInference",
                                  Qt::QueuedConnection);
        clearDisplayTrafficOverlay();
        m_trafficClassNames.clear();
        m_latestTrafficForCrossing = YoloDetectResult();
        m_latestTrafficSourceSeq = -1;
    }
    refreshDedicatedDisplayHud();
}

void MainWindow::onTrafficInferenceDone(int /*frameSeq*/)
{
    YoloDetectResult result;
    int sourceSeq = -1;
    if (m_trafficEnabled &&
        m_trafficWorker->getLatestResult(result, sourceSeq)) {
        m_latestTrafficForCrossing = result;
        m_latestTrafficSourceSeq = sourceSeq;
        updateTrafficCrossingAlert();
    } else {
        m_latestTrafficForCrossing = YoloDetectResult();
        m_latestTrafficSourceSeq = -1;
        clearDisplayTrafficOverlay();
    }
}

void MainWindow::onTrafficStatus(const QString &msg)
{
    m_statusLabel->setText(msg);
    refreshDedicatedDisplayHud();
}
// ============================================================================
// WiFi 回调
// ============================================================================

void MainWindow::onWifiFrameReady(int /*frameSeq*/)
{
    // 实际显示由 onDisplayUpdate 驱动的 acquireLatestFrameView 负责
}

void MainWindow::onWifiConnected()
{
    m_statusLabel->setText("WiFi 已连接");
    refreshDedicatedDisplayHud();
}

void MainWindow::onWifiDisconnected()
{
    m_statusLabel->setText("WiFi 断开");
    if (m_currentSource == InputSource::WIFI) {
        showDisplayPlaceholder("WiFi 连接断开 - 按 Q 切换源");
        m_currentSource = InputSource::NONE;
        m_sourceName = QStringLiteral("NONE");
    }
    refreshDedicatedDisplayHud();
}

void MainWindow::onWifiConnectionError(const QString &msg)
{
    m_statusLabel->setText("WiFi 错误: " + msg);
    qWarning() << "[WiFi] 连接错误:" << msg;
    refreshDedicatedDisplayHud();
}

void MainWindow::onWifiStatsUpdated(unsigned int frames, double fps, double kbps)
{
    Q_UNUSED(frames);
    Q_UNUSED(kbps);
    if (m_currentSource == InputSource::WIFI) {
        m_inputFPS = fps;
    }
    refreshDedicatedDisplayHud();
}

void MainWindow::onIpCameraConnected()
{
    if (m_currentSource != InputSource::IP_CAMERA) return;
    m_statusLabel->setText(QStringLiteral("手机 IP 摄像头已连接"));
    refreshDedicatedDisplayHud();
}

void MainWindow::onIpCameraDisconnected()
{
    if (m_currentSource != InputSource::IP_CAMERA) return;
    m_statusLabel->setText(QStringLiteral("手机 IP 摄像头断开，正在重连..."));
    showDisplayPlaceholder(QStringLiteral("手机摄像头断开，正在自动重连..."));
    refreshDedicatedDisplayHud();
}

void MainWindow::onIpCameraError(const QString &msg)
{
    if (m_currentSource != InputSource::IP_CAMERA) return;
    m_statusLabel->setText(QStringLiteral("手机摄像头错误: ") + msg);
    qWarning() << "[IpCamera]" << msg;
    refreshDedicatedDisplayHud();
}

void MainWindow::onIpCameraStatsUpdated(unsigned int frames, double fps, double kbps)
{
    Q_UNUSED(frames);
    Q_UNUSED(kbps);
    if (m_currentSource == InputSource::IP_CAMERA) m_inputFPS = fps;
    refreshDedicatedDisplayHud();
}

// ============================================================================
// 自动发现
// ============================================================================

void MainWindow::onSenderDiscovered(const QString &host, quint16 port,
                                     int /*width*/, int /*height*/)
{
    m_statusLabel->setText(QString("发现发送器: %1:%2").arg(host).arg(port));

    // 设置连接目标 (无论当前状态如何)
    m_wifiReceiver->connectToHost(host, port);

    // 如果当前不在 WiFi 模式, 自动切换过去
    if (m_currentSource != InputSource::WIFI &&
        m_currentSource != InputSource::IP_CAMERA) {
        switchToSource(InputSource::WIFI);
    }
    // 如果已经在 WiFi 模式, receiver 的 run() 循环会自动检测到 m_host/m_port 并连接
}

// ============================================================================
// 源切换核心逻辑
// ============================================================================

void MainWindow::toggleInputSource()
{
    if (m_currentSource == InputSource::PCIE) {
        switchToSource(InputSource::WIFI);
    } else if (m_currentSource == InputSource::WIFI) {
        if (m_ipCameraReceiver->hasUrl())
            switchToSource(InputSource::IP_CAMERA);
        else
            switchToSource(InputSource::PCIE);
    } else {
        switchToSource(InputSource::PCIE);
    }
}

void MainWindow::toggleFpgaInputSource()
{
    if (!m_device || !m_device->isOpened())
        return;

    selectFpgaInputSource(nextFpgaInputSource(m_fpgaInputSource));
}

void MainWindow::selectFpgaInputSource(FpgaInputSource requested)
{
    if (!m_device || !m_device->isOpened() || requested == m_fpgaInputSource)
        return;

    // SOURCE_STATUS (0x310) and frame completion (0x140) share the FPGA PIO
    // read path, so first stop the ARM polling thread.  Also stop FPGA DMA
    // before changing the physical input: switching a live asynchronous FIFO
    // can leave part of the old frame ahead of the new source and produces the
    // characteristic top/bottom displacement.  With START low the FPGA source
    // mux changes immediately and both tx/FIFO pipelines restart from the new
    // source's next frame boundary.
    const bool restartCaptureThread = m_capture && m_capture->isRunning();
    if (restartCaptureThread) {
        m_capture->stopCapture();
        if (!m_capture->wait(1000)) {
            qWarning() << "[fpga-source] capture thread did not stop in time";
            return;
        }
        m_device->stopCapture();
        usleep(2000);
    }

    if (!m_device->setFpgaInputSource(requested)) {
        if (restartCaptureThread) {
            m_device->startCapture();
            m_capture->start();
        }
        return;
    }

    m_fpgaInputSource = requested;
    const QString shortName = fpgaSourceShortName(requested);

    // Do not keep presenting a frame from the old physical input while the
    // FPGA waits for the new source's next VSYNC and the capture thread drops
    // its warm-up frames.
    if (m_currentSource == InputSource::PCIE) {
        m_frameBuffer->reset();
        if (m_detectionEnabled) {
            QMetaObject::invokeMethod(m_detectionWorker, "resetSceneState",
                                      Qt::BlockingQueuedConnection);
        }
        if (m_yoloEnabled) {
            QMetaObject::invokeMethod(m_yoloWorker, "resetSceneState",
                                      Qt::BlockingQueuedConnection);
        }
        if (m_yieldYoloEnabled) {
            QMetaObject::invokeMethod(m_yieldYoloWorker, "resetSceneState",
                                      Qt::BlockingQueuedConnection);
            clearDisplayYieldYoloOverlay();
        }
        if (m_detect3dEnabled) {
            QMetaObject::invokeMethod(m_detect3dWorker, "resetSceneState",
                                      Qt::BlockingQueuedConnection);
        }
        if (m_helmetEnabled) {
            QMetaObject::invokeMethod(m_helmetWorker, "resetSceneState",
                                      Qt::BlockingQueuedConnection);
        }
        if (m_trafficEnabled) {
            QMetaObject::invokeMethod(m_trafficWorker, "resetSceneState",
                                      Qt::BlockingQueuedConnection);
            clearDisplayTrafficOverlay();
        }
        m_lastDisplaySrc = nullptr;
        m_cachedDisplayImage = QImage();
        m_hasCachedDisplayImage = false;
        m_cachedDisplayFrameSeq = -1;
        m_lastDisplayedPlatePairSeq = -1;
        if (m_eglfsVideoWindow) {
            const QImage empty;
            m_eglfsVideoWindow->setPlateComparison(empty, empty, empty, empty);
        }
        showDisplayPlaceholder(QStringLiteral("正在切换到 %1 输入...").arg(shortName));
    }

    m_btnFpgaSource->setText(QStringLiteral("FPGA input: %1").arg(shortName));
    if (m_currentSource == InputSource::PCIE)
        m_sourceName = fpgaSourceDisplayName(requested);

    // Tracking/history must not leak across two physically different scenes.
    m_stableDetectFrameSeq = -1;
    m_stableDetectResult = DetectResult();
    resetPlateHistory();
    updatePlateList(DetectResult());
    m_statusLabel->setText(QStringLiteral("正在切换 FPGA 输入到 %1...").arg(shortName));
    if (restartCaptureThread) {
        m_device->startCapture();
        m_capture->start();
    }
    refreshDedicatedDisplayHud();
}

void MainWindow::setDisplayHelmetOverlay(const YoloDetectResult &result,
                                         const char *const *classNames,
                                         int numClasses)
{
    if (m_eglfsVideoWindow)
        m_eglfsVideoWindow->setHelmetOverlay(result, classNames, numClasses);
    else if (m_videoWidget)
        m_videoWidget->setHelmetOverlay(result, classNames, numClasses);
}

void MainWindow::clearDisplayHelmetOverlay()
{
    if (m_eglfsVideoWindow)
        m_eglfsVideoWindow->clearHelmetOverlay();
    else if (m_videoWidget)
        m_videoWidget->clearHelmetOverlay();
}

void MainWindow::setDisplayTrafficOverlay(const YoloDetectResult &result,
                                          const char *const *classNames,
                                          int numClasses)
{
    if (m_eglfsVideoWindow)
        m_eglfsVideoWindow->setTrafficOverlay(result, classNames, numClasses);
    else if (m_videoWidget)
        m_videoWidget->setTrafficOverlay(result, classNames, numClasses);
}

void MainWindow::clearDisplayTrafficOverlay()
{
    if (m_eglfsVideoWindow)
        m_eglfsVideoWindow->clearTrafficOverlay();
    else if (m_videoWidget)
        m_videoWidget->clearTrafficOverlay();
}
void MainWindow::restartRkPciePipeline()
{
    const QString appDir = QCoreApplication::applicationDirPath();
    const QString script = appDir + QStringLiteral("/recover_rk_system_pipeline.sh");
    if (!QFileInfo::exists(script)) {
        qWarning() << "[rk-system-recovery] missing script" << script;
        m_statusLabel->setText(QStringLiteral("Missing RK recovery script"));
        refreshDedicatedDisplayHud();
        return;
    }

    qint64 helperPid = 0;
    const bool started = QProcess::startDetached(
        QStringLiteral("/bin/sh"),
        QStringList() << script << QStringLiteral("--from-app"),
        appDir,
        &helperPid);
    if (!started) {
        qWarning() << "[rk-system-recovery] failed to start" << script;
        m_statusLabel->setText(QStringLiteral("Failed to start RK recovery helper"));
        refreshDedicatedDisplayHud();
        return;
    }

    qWarning().nospace()
        << "[rk-system-recovery] helper_pid=" << helperPid
        << " action=quit-release-dma-reload-driver-pci-reset-restart";
    m_statusLabel->setText(QStringLiteral("RK PCIe system recovery starting..."));
    refreshDedicatedDisplayHud();
    QTimer::singleShot(100, qApp, &QCoreApplication::quit);
}

void MainWindow::switchToSource(InputSource newSource)
{
    if (newSource == m_currentSource)
        return;

    // Network-source startup intentionally skips the default EGLFS PCIe
    // initialization.  When the user later selects PCIe, prepare the device
    // before stopping the currently working source.  If initialization fails,
    // keep the current source alive instead of leaving the display on NONE.
    if (newSource == InputSource::PCIE &&
        (!m_device || !m_device->isOpened())) {
        m_statusLabel->setText(QStringLiteral("正在初始化 PCIe 设备..."));
        onOpenDevice();
        if (!m_device || !m_device->isOpened()) {
            m_statusLabel->setText(QStringLiteral("PCIe 设备打开失败，保持当前输入源"));
            refreshDedicatedDisplayHud();
            return;
        }
    }

    // Quiesce the old source's periodic display producer before stopping or
    // rearming capture. Otherwise a queued timeout can submit an old frame
    // while the first PCIe frame is being presented and DRM rejects one of
    // the overlapping page flips with EBUSY. WiFi/IP restart this timer in
    // their cases below; PCIe remains frameReady-driven.
    m_displayTimer->stop();

    // ---- 停止 AI 推理 ----
    if (m_detectionEnabled) {
        QMetaObject::invokeMethod(m_detectionWorker, "stopInference",
                                  Qt::QueuedConnection);
    }
    if (m_yoloEnabled) {
        QMetaObject::invokeMethod(m_yoloWorker, "stopInference",
                                  Qt::QueuedConnection);
    }
    if (m_yieldYoloEnabled) {
        QMetaObject::invokeMethod(m_yieldYoloWorker, "stopInference",
                                  Qt::QueuedConnection);
        clearDisplayYieldYoloOverlay();
    }
    if (m_detect3dEnabled) {
        QMetaObject::invokeMethod(m_detect3dWorker, "stopInference",
                                  Qt::QueuedConnection);
    }
    if (m_helmetEnabled) {
        QMetaObject::invokeMethod(m_helmetWorker, "stopInference",
                                  Qt::QueuedConnection);
    }
    if (m_trafficEnabled) {
        QMetaObject::invokeMethod(m_trafficWorker, "resetSceneState",
                                  Qt::BlockingQueuedConnection);
        clearDisplayTrafficOverlay();
    }

    // ---- 停止当前源 ----
    if (m_currentSource == InputSource::PCIE) {
        if (m_capture && m_capture->isRunning()) {
            m_capture->stopCapture();
        }
        if (m_device && m_device->isOpened()) {
            m_device->stopCapture();
        }
        if (m_capture && m_capture->isRunning()) {
            m_capture->wait(3000);
        }
    } else if (m_currentSource == InputSource::WIFI) {
        if (m_wifiReceiver && m_wifiReceiver->isRunning()) {
            m_wifiReceiver->stop();
        }
    } else if (m_currentSource == InputSource::IP_CAMERA) {
        if (m_ipCameraReceiver && m_ipCameraReceiver->isRunning()) {
            m_ipCameraReceiver->stop();
        }
    }

    // ---- 重置 FrameBuffer 和统计 ----
    m_frameBuffer->reset();
    m_inputFPS = 0.0;
    m_displayFPS = 0.0;
    m_displayFrameCount = 0;
    m_lastFrameSeq = -1;
    m_lastDisplaySrc = nullptr;
    m_cachedDisplayImage = QImage();
    m_hasCachedDisplayImage = false;
    m_cachedDisplayFrameSeq = -1;
    m_lastDisplayedPlatePairSeq = -1;
    if (m_eglfsVideoWindow) {
        const QImage empty;
        m_eglfsVideoWindow->setPlateComparison(empty, empty, empty, empty);
    }
    m_stableDetectFrameSeq = -1;
    m_stableDetectResult = DetectResult();
    resetPlateHistory();
    m_latestYieldYoloForCrossing = YoloDetectResult();
    m_latestYieldYoloSourceSeq = -1;
    m_latestTrafficForCrossing = YoloDetectResult();
    m_latestTrafficSourceSeq = -1;

    // ---- 启动新源 ----
    switch (newSource) {
    case InputSource::PCIE:
        if (m_device && m_device->isOpened()) {
            m_frameBuffer->reset();
            if (!m_device->rearmFpgaInputSource(m_fpgaInputSource)) {
                qWarning() << "[fpga-source] resume rearm failed; using direct selection";
                m_device->setFpgaInputSource(m_fpgaInputSource);
            }
            if (!m_device->programFpgaDmaBases()) {
                qWarning() << "[dma-config] refresh while resuming PCIe failed";
                m_currentSource = InputSource::NONE;
                m_sourceName = QStringLiteral("NONE");
                m_statusLabel->setText(QStringLiteral("PCIe DMA configuration failed"));
                break;
            }
            usleep(1000);
            m_device->startCapture();
            m_capture->start();
            // PCIe is event-driven: one display submission per captured frame.
            // The periodic timer is reserved for WiFi/IP sources only.
            m_displayTimer->stop();
            m_currentSource = InputSource::PCIE;
            m_sourceName = fpgaSourceDisplayName(m_fpgaInputSource);
            m_btnAiDetect->setEnabled(true);
            m_btnYoloDetect->setEnabled(true);
            m_btnDetect3d->setEnabled(true);
            m_statusLabel->setText("PCIe 采集");
        } else {
            showDisplayPlaceholder("PCIe 设备未打开 - 按 Q 切换源");
            m_currentSource = InputSource::NONE;
            m_sourceName = QStringLiteral("NONE");
            m_displayTimer->stop();
        }
        break;

    case InputSource::WIFI:
        m_currentSource = InputSource::WIFI;
        m_sourceName = QStringLiteral("WiFi");
        m_wifiReceiver->start();
        m_displayTimer->start();
        m_statusLabel->setText("WiFi 收集中...");

        // 启动自动发现 (如果还没连上)
        if (!m_wifiReceiver->isConnected() && !m_wifiDiscovery->isListening()) {
            m_wifiDiscovery->start();
        }
        break;

    case InputSource::IP_CAMERA:
        if (!m_ipCameraReceiver->hasUrl()) {
            m_currentSource = InputSource::NONE;
            m_sourceName = QStringLiteral("NONE");
            m_displayTimer->stop();
            showDisplayPlaceholder(QStringLiteral("未设置手机 IP 摄像头 URL"));
            break;
        }
        m_currentSource = InputSource::IP_CAMERA;
        m_sourceName = QStringLiteral("IP Cam");
        m_ipCameraReceiver->start();
        m_displayTimer->start();
        m_statusLabel->setText(QStringLiteral("正在连接手机 IP 摄像头..."));
        break;

    case InputSource::NONE:
        m_currentSource = InputSource::NONE;
        m_sourceName = QStringLiteral("NONE");
        m_displayTimer->stop();
        showDisplayPlaceholder("无输入源 - 按 Q 切换");
        break;
    }

    // Source switching pauses the workers only long enough to discard the old
    // scene. Keep the user's feature switches unchanged and resume every
    // backend that was active once the new source is running.
    if (m_currentSource != InputSource::NONE) {
        if (m_detectionEnabled)
            QMetaObject::invokeMethod(m_detectionWorker, "startInference",
                                      Qt::QueuedConnection);
        if (m_yoloEnabled)
            QMetaObject::invokeMethod(m_yoloWorker, "startInference",
                                      Qt::QueuedConnection);
        if (m_yieldYoloEnabled)
            QMetaObject::invokeMethod(m_yieldYoloWorker, "startInference",
                                      Qt::QueuedConnection);
        if (m_detect3dEnabled)
            QMetaObject::invokeMethod(m_detect3dWorker, "startInference",
                                      Qt::QueuedConnection);
        if (m_helmetEnabled)
            QMetaObject::invokeMethod(m_helmetWorker, "startInference",
                                      Qt::QueuedConnection);
    }

    refreshDedicatedDisplayHud();
}
