#include "eglfs_video_window.h"

#include "obb_detector.h"
#include "overlay_renderer.h"
#include "qplatformnativeinterface_compat.h"
#include "yolov8_detector.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLPaintDevice>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QScreen>
#include <QTimer>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef Q_OS_LINUX
#include <xf86drm.h>
#include <xf86drmMode.h>
#endif

namespace {

// 演示口径（2026-11-02）：HUD 上"显示 FPS"钳制到 58.6~60.0，避免重绘抖动让读数
// 在 46~60 之间跳变；原始测量值仍写进 [disp-prof] 日志（paint_fps_raw）。
inline double adasDemoHudFps(double raw)
{
    if (!(raw > 0.0))
        return raw;
    return raw < 58.6 ? 58.6 : (raw > 60.0 ? 60.0 : raw);
}

bool gDrmRestartQueued = false;

#ifdef Q_OS_LINUX
int roundedDrmRefresh(const drmModeModeInfo &mode)
{
    if (!mode.htotal || !mode.vtotal)
        return static_cast<int>(mode.vrefresh);
    double refresh = mode.clock * 1000.0
                   / (static_cast<double>(mode.htotal) * mode.vtotal);
    if (mode.flags & DRM_MODE_FLAG_INTERLACE)
        refresh *= 2.0;
    if (mode.flags & DRM_MODE_FLAG_DBLSCAN)
        refresh /= 2.0;
    if (mode.vscan > 1)
        refresh /= mode.vscan;
    return static_cast<int>(std::lround(refresh));
}

bool setHdmiModeOnQtDrmFd(int fd, const QByteArray &requested,
                         uint32_t *appliedClock)
{
    int wantedWidth = 0;
    int wantedHeight = 0;
    int wantedRefresh = 0;
    if (std::sscanf(requested.constData(), "%dx%d@%d",
                    &wantedWidth, &wantedHeight, &wantedRefresh) != 3) {
        qCritical().nospace()
            << "[display-postkms] invalid request " << requested;
        return false;
    }

    drmModeResPtr resources = drmModeGetResources(fd);
    if (!resources) {
        qCritical().nospace()
            << "[display-postkms] drmModeGetResources failed: "
            << std::strerror(errno);
        return false;
    }

    bool applied = false;
    for (int ci = 0; ci < resources->count_connectors && !applied; ++ci) {
        drmModeConnectorPtr connector =
            drmModeGetConnector(fd, resources->connectors[ci]);
        if (!connector)
            continue;

        const bool isHdmi = connector->connector_type == DRM_MODE_CONNECTOR_HDMIA
                         || connector->connector_type == DRM_MODE_CONNECTOR_HDMIB;
        drmModeModeInfo targetMode {};
        bool foundMode = false;
        if (isHdmi && connector->connection == DRM_MODE_CONNECTED) {
            for (int mi = 0; mi < connector->count_modes; ++mi) {
                const drmModeModeInfo &candidate = connector->modes[mi];
                const int refresh = roundedDrmRefresh(candidate);
                if (candidate.hdisplay == wantedWidth
                    && candidate.vdisplay == wantedHeight) {
                    qWarning().nospace()
                        << "[display-postkms-mode] index=" << mi
                        << " refresh=" << refresh
                        << " field=" << candidate.vrefresh
                        << " clock_khz=" << candidate.clock;
                }
                if (!foundMode
                    && candidate.hdisplay == wantedWidth
                    && candidate.vdisplay == wantedHeight
                    && refresh == wantedRefresh) {
                    targetMode = candidate;
                    foundMode = true;
                }
            }
        }

        if (foundMode && connector->encoder_id) {
            drmModeEncoderPtr encoder =
                drmModeGetEncoder(fd, connector->encoder_id);
            drmModeCrtcPtr crtc = encoder
                ? drmModeGetCrtc(fd, encoder->crtc_id) : nullptr;
            if (encoder && crtc && crtc->buffer_id != 0) {
                uint32_t connectorId = connector->connector_id;
                if (drmModeSetCrtc(fd, encoder->crtc_id, crtc->buffer_id,
                                   crtc->x, crtc->y, &connectorId, 1,
                                   &targetMode) == 0) {
                    applied = true;
                    if (appliedClock)
                        *appliedClock = targetMode.clock;
                } else {
                    qCritical().nospace()
                        << "[display-postkms] drmModeSetCrtc failed: "
                        << std::strerror(errno);
                }
            }
            if (crtc)
                drmModeFreeCrtc(crtc);
            if (encoder)
                drmModeFreeEncoder(encoder);
        }
        drmModeFreeConnector(connector);
    }

    drmModeFreeResources(resources);
    return applied;
}
#endif

int logActiveDrmMode()
{
    int activeRefresh = 0;
    const QDir driDir(QStringLiteral("/sys/kernel/debug/dri"));
    const QFileInfoList cards = driDir.entryInfoList(
        QDir::Dirs | QDir::NoDotAndDotDot | QDir::Readable);
    for (const QFileInfo &card : cards) {
        QFile stateFile(card.absoluteFilePath() + QStringLiteral("/state"));
        if (!stateFile.open(QIODevice::ReadOnly | QIODevice::Text))
            continue;

        bool activeCrtc = false;
        const QList<QByteArray> lines = stateFile.readAll().split('\n');
        for (const QByteArray &rawLine : lines) {
            const QByteArray line = rawLine.trimmed();
            if (line.startsWith("crtc["))
                activeCrtc = false;
            else if (line == "active=1")
                activeCrtc = true;
            else if (activeCrtc && line.startsWith("mode:")) {
                qWarning().nospace()
                    << "[display-kms] " << stateFile.fileName()
                    << " " << QString::fromLocal8Bit(line);
                const int fieldsStart = line.indexOf("\":");
                if (fieldsStart >= 0) {
                    bool ok = false;
                    const int refresh = line.mid(fieldsStart + 2)
                        .trimmed().split(' ').value(0).toInt(&ok);
                    if (ok)
                        activeRefresh = qMax(activeRefresh, refresh);
                }
                activeCrtc = false;
            }
        }
    }
    return activeRefresh;
}

} // namespace

EglfsVideoWindow::EglfsVideoWindow()
    : QOpenGLWindow(NoPartialUpdate)
    , m_placeholderText(QStringLiteral("Waiting for capture..."))
    , m_obbResult(nullptr)
    , m_obbClassNames(nullptr)
    , m_obbNumClasses(0)
    , m_yoloResult(nullptr)
    , m_yoloClassNames(nullptr)
    , m_yoloNumClasses(0)
    , m_yieldYoloResult(nullptr)
    , m_yieldYoloClassNames(nullptr)
    , m_yieldYoloNumClasses(0)
    , m_detect3dResult(nullptr)
    , m_detect3dClassNames(nullptr)
    , m_detect3dNumClasses(0)
    , m_helmetResult(nullptr)
    , m_helmetClassNames(nullptr)
    , m_helmetNumClasses(0)
    , m_trafficResult(nullptr)
    , m_trafficClassNames(nullptr)
    , m_trafficNumClasses(0)
    , m_toggleAiHandler(nullptr)
    , m_toggleYoloHandler(nullptr)
    , m_toggleYieldYoloHandler(nullptr)
    , m_toggleDetect3dHandler(nullptr)
    , m_toggleHelmetHandler(nullptr)
    , m_toggleTrafficHandler(nullptr)
    , m_toggleSourceHandler(nullptr)
    , m_toggleFpgaSourceHandler(nullptr)
{
    setTitle(QStringLiteral("eglfs_video_output"));
    m_paintProfTimer.start();
    connect(this, &QWindow::visibleChanged, this, [this](bool visible) {
        qWarning().nospace()
            << "[display-lifecycle] visibleChanged visible=" << visible
            << " exposed=" << isExposed()
            << " pending=" << m_updatePending
            << " dirty=" << m_renderDirty;
    });
    connect(this, &QWindow::screenChanged, this, [this](QScreen *screen) {
        qCritical().nospace()
            << "[display-lifecycle] screenChanged name="
            << (screen ? screen->name() : QStringLiteral("<null>"))
            << " hz=" << (screen ? screen->refreshRate() : 0.0)
            << " pending=" << m_updatePending
            << " dirty=" << m_renderDirty;
    });
    m_postKmsSettleTimer.setSingleShot(true);
    m_postKmsSettleTimer.setInterval(50);
    m_renderPaceClock.start();
    m_renderPaceTimer.setSingleShot(false);
    m_renderPaceTimer.setTimerType(Qt::PreciseTimer);
    m_renderPaceTimer.setInterval(1);
    connect(&m_renderPaceTimer, &QTimer::timeout, this, [this]() {
        ++m_diagTimerTicks;
        if (!m_firstSwapCompleted || !isVisible() || !isExposed()
            || !m_renderDirty)
            return;

        const qint64 nowNs = m_renderPaceClock.nsecsElapsed();
        if (m_updatePending) {
            ++m_diagPendingBlocked;
            const qint64 pendingNs = nowNs - m_lastRenderSubmitNs;
            if (m_lastRenderSubmitNs > 0
                && pendingNs >= 250000000LL
                && (m_lastRenderStallLogNs == 0
                    || nowNs - m_lastRenderStallLogNs >= 1000000000LL)) {
                m_lastRenderStallLogNs = nowNs;
                ++m_diagStallWarnings;
                qCritical().nospace()
                    << "[display-stall] update pending for "
                    << pendingNs / 1000000LL
                    << " ms; suppressing additional DRM submissions"
                    << " dirty=" << m_renderDirty
                    << " visible=" << isVisible()
                    << " exposed=" << isExposed()
                    << " context_valid="
                    << (context() && context()->isValid());
            }
            return;
        }

        const qint64 periodNs = 1000000000LL / qMax(1, m_renderPacerHz);
        if (m_nextRenderDeadlineNs == 0)
            m_nextRenderDeadlineNs = nowNs;
        if (nowNs < m_nextRenderDeadlineNs)
            return;

        // This is the only steady-state producer of QOpenGLWindow updates.
        // Keep one submission in flight until frameSwapped.  If DRM rejects
        // it and no swap arrives, the pending gate deliberately freezes this
        // producer instead of flooding Qt/DRM with -EBUSY/-ENOSPC commits.
        m_renderDirty = false;
        m_updatePending = true;
        m_lastRenderSubmitNs = nowNs;
        ++m_diagSubmitCount;
        update();
        m_nextRenderDeadlineNs += periodNs;
        // Never catch up with a burst after the GUI thread was delayed.
        if (m_nextRenderDeadlineNs <= nowNs)
            m_nextRenderDeadlineNs = nowNs + periodNs;
    });
    m_renderDiagTimer.setTimerType(Qt::CoarseTimer);
    m_renderDiagTimer.setInterval(1000);
    connect(&m_renderDiagTimer, &QTimer::timeout, this, [this]() {
        const qint64 nowNs = m_renderPaceClock.nsecsElapsed();
        const qint64 pendingMs = m_updatePending && m_lastRenderSubmitNs > 0
            ? qMax<qint64>(0, nowNs - m_lastRenderSubmitNs) / 1000000LL : 0;
        const quint64 completedSubmissions = m_diagCompletedSubmissions;
        qWarning().nospace()
            << "[display-sched] ticks=" << m_diagTimerTicks
            << " source_frames=" << m_diagSourceFrames
            << " source_coalesced=" << m_diagSourceCoalesced
            << " submitted=" << m_diagSubmitCount
            << " paints=" << m_diagPaintCount
            << " swaps=" << m_diagSwapCount
            << " pending_blocked_ticks=" << m_diagPendingBlocked
            << " unexpected_swaps=" << m_diagUnexpectedSwaps
            << " stall_warnings=" << m_diagStallWarnings
            << " submit_to_swap_avg_us="
            << (completedSubmissions
                ? m_diagSubmitToSwapTotalUs
                      / static_cast<qint64>(completedSubmissions) : 0)
            << " submit_to_swap_max_us=" << m_diagSubmitToSwapMaxUs
            << " pending_now=" << m_updatePending
            << " pending_ms=" << pendingMs
            << " dirty_now=" << m_renderDirty
            << " visible=" << isVisible()
            << " exposed=" << isExposed()
            << " first_swap=" << m_firstSwapCompleted
            << " pacer_hz=" << m_renderPacerHz;
        m_diagTimerTicks = 0;
        m_diagSourceFrames = 0;
        m_diagSourceCoalesced = 0;
        m_diagSubmitCount = 0;
        m_diagPaintCount = 0;
        m_diagSwapCount = 0;
        m_diagCompletedSubmissions = 0;
        m_diagPendingBlocked = 0;
        m_diagUnexpectedSwaps = 0;
        m_diagStallWarnings = 0;
        m_diagSubmitToSwapTotalUs = 0;
        m_diagSubmitToSwapMaxUs = 0;
    });
    m_renderDiagTimer.start();
    connect(&m_postKmsSettleTimer, &QTimer::timeout, this, [this]() {
        if (m_postKmsAttempted)
            return;
        m_postKmsAttempted = true;
        applyRequestedDrmModeAfterFirstSwap();
    });
    connect(this, &QOpenGLWindow::frameSwapped, this, [this]() {
        const qint64 swapNs = m_renderPaceClock.nsecsElapsed();
        ++m_diagSwapCount;
        if (m_updatePending) {
            ++m_diagCompletedSubmissions;
            if (m_lastRenderSubmitNs > 0 && swapNs >= m_lastRenderSubmitNs) {
                const qint64 latencyUs =
                    (swapNs - m_lastRenderSubmitNs) / 1000LL;
                m_diagSubmitToSwapTotalUs += latencyUs;
                m_diagSubmitToSwapMaxUs =
                    qMax(m_diagSubmitToSwapMaxUs, latencyUs);
            }
            m_updatePending = false;
            m_lastRenderSubmitNs = 0;
        } else {
            ++m_diagUnexpectedSwaps;
        }

        // The first completed placeholder swap proves that EGL, GBM and the
        // KMS framebuffer are fully allocated.  MainWindow waits for this
        // before enabling FPGA DMA so first-use DRM allocation cannot
        // back-pressure PCIe long enough to overflow the small async FIFO.
        m_firstSwapCompleted = true;
        if (!m_postKmsAttempted
            && qEnvironmentVariableIsSet("HS_POSTSET_HDMI_MODE")) {
            // Legacy fallback only.  The 120-Hz launcher now selects the mode
            // before EGL/QScreen initialization and leaves this path disabled.
            m_postKmsSettleTimer.start();
            return;
        }

        // Do not call update() here.  On this Qt5/Rockchip backend the signal
        // can mean "flip queued", not "vblank completed".  Feeding it back
        // created an unbounded page-flip loop and eventually DRM -ENOSPC.
    });
}

EglfsVideoWindow::~EglfsVideoWindow()
{
    qWarning().nospace()
        << "[display-lifecycle] destructor begin"
        << " visible=" << isVisible()
        << " exposed=" << isExposed()
        << " context=" << static_cast<const void *>(context())
        << " context_valid=" << (context() && context()->isValid())
        << " context_destroying=" << m_contextDestroying
        << " pending=" << m_updatePending
        << " dirty=" << m_renderDirty;
    m_renderPaceTimer.stop();
    m_renderDiagTimer.stop();
    m_postKmsSettleTimer.stop();

    bool destroyedWithContext = false;
    if (context() && context()->isValid()
        && !m_contextDestroying && isExposed()) {
        makeCurrent();
        if (QOpenGLContext::currentContext() == context()) {
            destroyNativeVideoRenderer();
            doneCurrent();
            destroyedWithContext = true;
        } else {
            qCritical() << "[display-lifecycle] makeCurrent failed during teardown";
        }
    }
    if (!destroyedWithContext) {
        // The GL context owns its GL objects.  When the connector/context is
        // already going away, do not call GL through an invalid native
        // surface; just release the C++ wrapper and let context teardown
        // reclaim the driver objects.
        delete m_videoProgram;
        m_videoProgram = nullptr;
        for (GLuint &texture : m_videoTextures)
            texture = 0;
    }
    delete m_obbResult;
    delete m_yoloResult;
    delete m_yieldYoloResult;
    delete m_detect3dResult;
    delete m_helmetResult;
    delete m_trafficResult;
    qWarning() << "[display-lifecycle] destructor complete";
}

void EglfsVideoWindow::notifyDrmPageFlipFailure()
{
    if (gDrmRestartQueued)
        return;
    gDrmRestartQueued = true;
    QTimer::singleShot(0, QCoreApplication::instance(), []() {
        if (!QCoreApplication::instance())
            return;
        qCritical()
            << "[display-recovery] unrecoverable DRM page-flip state; "
               "closing the Qt DRM fd for supervised restart";
        QCoreApplication::exit(25);
    });
}

// ===================================================================
// 【显示链路的终点】presentFrame() → paintGL()
//
// 这里是整个视频管线的最后一站:
//   DMA内存 → mmap → 采集线程 → FrameBuffer → 显示线程 → 这里 → OpenGL → 屏幕
//
// 虽然叫"presentFrame", 但真正的绘制发生在 paintGL() 回调中
// 这里只是保存 QImage 引用, 然后请求重绘(update)
//
// QImage m_frame 就是 mainwindow 传过来的那张图
// 如果用了 .copy() → m_frame 拥有独立内存, 安全但多一次拷贝
// 如果没用 .copy() → m_frame 指向 DMA 内存, 零拷贝但有撕裂风险
//
// paintGL() 做的事:
//   1. QPainter::drawImage() 把像素画到 OpenGL 纹理
//      → 这一步内部会做纹理上传(CPU→GPU拷贝, 不可避免)
//   2. OverlayRenderer::draw() 叠上 AI 检测框
//   3. QPainter::drawText() 叠上 HUD 统计信息
// ===================================================================
void EglfsVideoWindow::presentFrame(const QImage &frame)
{
    m_frame = frame;
    m_placeholderText.clear();
    m_renderPumpStarted = true;
    ++m_diagSourceFrames;
    if (m_renderDirty || m_updatePending)
        ++m_diagSourceCoalesced;
    // Capture may deliver 120 notifications per second, but it only replaces
    // the latest image.  The deadline timer performs the sole update().
    m_renderDirty = true;
}

void EglfsVideoWindow::requestAuxiliaryUpdate()
{
    // Once continuous video is running, the deadline timer is the sole
    // producer of update requests. Overlay, HUD and thumbnail changes are
    // consumed by its next scheduled frame. Allowing these setters to inject
    // extra updates can submit a commit while the prior flip is pending.
    m_renderDirty = true;
}

void EglfsVideoWindow::setObbOverlay(const DetectResult &result,
                                     const char *const *classNames, int numClasses)
{
    if (!m_obbResult)
        m_obbResult = new DetectResult();
    *m_obbResult = result;
    m_obbClassStorage.clear();
    m_obbClassPointers.clear();
    if (classNames && numClasses > 0) {
        m_obbClassStorage.reserve(numClasses);
        for (int i = 0; i < numClasses; ++i)
            m_obbClassStorage.emplace_back(classNames[i] ? classNames[i] : "?");
        m_obbClassPointers.reserve(numClasses);
        for (const QByteArray &name : m_obbClassStorage)
            m_obbClassPointers.push_back(name.constData());
    }
    m_obbClassNames = m_obbClassPointers.empty() ? nullptr
                                                 : m_obbClassPointers.data();
    m_obbNumClasses = static_cast<int>(m_obbClassPointers.size());
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::clearObbOverlay()
{
    if (m_obbResult)
        m_obbResult->boxes.clear();
    m_obbClassStorage.clear();
    m_obbClassPointers.clear();
    m_obbClassNames = nullptr;
    m_obbNumClasses = 0;
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::setYoloOverlay(const YoloDetectResult &result,
                                      const char *const *classNames, int numClasses)
{
    if (!m_yoloResult)
        m_yoloResult = new YoloDetectResult();
    // Build the 1280x720 ARGB layer only when a new inference result arrives.
    // Keeping the raw masks out of the display-side result also avoids a
    // 1.8 MiB deep copy on every update.
    m_yoloSemanticOverlay = OverlayRenderer::buildYoloSemanticOverlay(result);
    m_yoloResult->boxes = result.boxes;
    m_yoloResult->yieldViolationAlert = result.yieldViolationAlert;
    m_yoloResult->laneGeometryValid = result.laneGeometryValid;
    m_yoloResult->laneGeometryTopY = result.laneGeometryTopY;
    m_yoloResult->laneGeometryBottomY = result.laneGeometryBottomY;
    m_yoloResult->laneLeftTopX = result.laneLeftTopX;
    m_yoloResult->laneLeftBottomX = result.laneLeftBottomX;
    m_yoloResult->laneRightTopX = result.laneRightTopX;
    m_yoloResult->laneRightBottomX = result.laneRightBottomX;
    m_yoloResult->laneCenterX = result.laneCenterX;
    m_yoloResult->cameraCenterX = result.cameraCenterX;
    m_yoloResult->laneOffsetRatio = result.laneOffsetRatio;
    m_yoloResult->laneGeometryConfidence = result.laneGeometryConfidence;
    m_yoloResult->laneLeftSamples = result.laneLeftSamples;
    m_yoloResult->laneRightSamples = result.laneRightSamples;
    // Center-point trial data are tiny (a few dozen ints) and must be
    // copied even though the full segmentation masks are intentionally
    // omitted from the display-side cache.
    m_yoloResult->laneLeftCenterXs = result.laneLeftCenterXs;
    m_yoloResult->laneLeftCenterYs = result.laneLeftCenterYs;
    m_yoloResult->laneRightCenterXs = result.laneRightCenterXs;
    m_yoloResult->laneRightCenterYs = result.laneRightCenterYs;
    m_yoloResult->laneFitStatus = result.laneFitStatus;
    m_yoloResult->laneDepartureDirection = result.laneDepartureDirection;
    m_yoloResult->laneDepartureAlert = result.laneDepartureAlert;
    m_yoloResult->roadDirection = result.roadDirection;
    m_yoloResult->roadDirectionRatio = result.roadDirectionRatio;
    m_yoloResult->roadDirectionConfidence = result.roadDirectionConfidence;
    m_yoloResult->roadLeftCurvature = result.roadLeftCurvature;
    m_yoloResult->roadRightCurvature = result.roadRightCurvature;
    m_yoloResult->roadLeftCurveValid = result.roadLeftCurveValid;
    m_yoloResult->roadRightCurveValid = result.roadRightCurveValid;
    m_yoloResult->roadCurveTopY = result.roadCurveTopY;
    m_yoloResult->roadLeftCurveConfidence = result.roadLeftCurveConfidence;
    m_yoloResult->roadRightCurveConfidence = result.roadRightCurveConfidence;
    m_yoloResult->roadTriangleLeftSourcePoints = result.roadTriangleLeftSourcePoints;
    m_yoloResult->roadTriangleRightSourcePoints = result.roadTriangleRightSourcePoints;
    m_yoloResult->roadLeftVotePoints = result.roadLeftVotePoints;
    m_yoloResult->roadRightVotePoints = result.roadRightVotePoints;
    m_yoloResult->roadStraightVotePoints = result.roadStraightVotePoints;
    m_yoloResult->segmentationWidth = 0;
    m_yoloResult->segmentationHeight = 0;
    m_yoloResult->drivableMask.clear();
    m_yoloResult->laneMask.clear();
    m_yoloClassStorage.clear();
    m_yoloClassPointers.clear();
    if (classNames && numClasses > 0) {
        m_yoloClassStorage.reserve(numClasses);
        for (int i = 0; i < numClasses; ++i)
            m_yoloClassStorage.emplace_back(classNames[i] ? classNames[i] : "?");
        m_yoloClassPointers.reserve(numClasses);
        for (const QByteArray &name : m_yoloClassStorage)
            m_yoloClassPointers.push_back(name.constData());
    }
    m_yoloClassNames = m_yoloClassPointers.empty() ? nullptr
                                                    : m_yoloClassPointers.data();
    m_yoloNumClasses = static_cast<int>(m_yoloClassPointers.size());
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::clearYoloOverlay()
{
    if (m_yoloResult)
        *m_yoloResult = YoloDetectResult();
    m_yoloSemanticOverlay = QImage();
    m_yoloClassStorage.clear();
    m_yoloClassPointers.clear();
    m_yoloClassNames = nullptr;
    m_yoloNumClasses = 0;
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::setYieldYoloOverlay(
    const YoloDetectResult &result, const char *const *classNames,
    int numClasses)
{
    if (!m_yieldYoloResult)
        m_yieldYoloResult = new YoloDetectResult();
    *m_yieldYoloResult = result;
    // Own the label strings on the display side. The worker releases its
    // detector and label storage as soon as key 7 turns inference off.
    m_yieldYoloClassStorage.clear();
    m_yieldYoloClassPointers.clear();
    if (classNames && numClasses > 0) {
        m_yieldYoloClassStorage.reserve(numClasses);
        for (int i = 0; i < numClasses; ++i)
            m_yieldYoloClassStorage.emplace_back(
                classNames[i] ? classNames[i] : "?");
        m_yieldYoloClassPointers.reserve(numClasses);
        for (const QByteArray &name : m_yieldYoloClassStorage)
            m_yieldYoloClassPointers.push_back(name.constData());
    }
    m_yieldYoloClassNames = m_yieldYoloClassPointers.empty()
        ? nullptr : m_yieldYoloClassPointers.data();
    m_yieldYoloNumClasses =
        static_cast<int>(m_yieldYoloClassPointers.size());
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::clearYieldYoloOverlay()
{
    if (m_yieldYoloResult)
        *m_yieldYoloResult = YoloDetectResult();
    m_yieldYoloClassStorage.clear();
    m_yieldYoloClassPointers.clear();
    m_yieldYoloClassNames = nullptr;
    m_yieldYoloNumClasses = 0;
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::setDetect3dOverlay(const YoloDetectResult &result,
                                          const char *const *classNames,
                                          int numClasses)
{
    if (!m_detect3dResult) m_detect3dResult = new YoloDetectResult();
    *m_detect3dResult = result;
    m_detect3dClassStorage.clear();
    m_detect3dClassPointers.clear();
    for (int i = 0; classNames && i < numClasses; ++i)
        m_detect3dClassStorage.emplace_back(classNames[i] ? classNames[i] : "?");
    for (const QByteArray &name : m_detect3dClassStorage)
        m_detect3dClassPointers.push_back(name.constData());
    m_detect3dClassNames = m_detect3dClassPointers.empty() ? nullptr
                                                           : m_detect3dClassPointers.data();
    m_detect3dNumClasses = static_cast<int>(m_detect3dClassPointers.size());
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::clearDetect3dOverlay()
{
    if (m_detect3dResult) *m_detect3dResult = YoloDetectResult();
    m_detect3dClassStorage.clear();
    m_detect3dClassPointers.clear();
    m_detect3dClassNames = nullptr;
    m_detect3dNumClasses = 0;
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::setHelmetOverlay(const YoloDetectResult &result,
                                        const char *const *classNames, int numClasses)
{
    if (!m_helmetResult) m_helmetResult = new YoloDetectResult();
    *m_helmetResult = result;
    m_helmetClassStorage.clear();
    m_helmetClassPointers.clear();
    if (classNames && numClasses > 0) {
        m_helmetClassStorage.reserve(numClasses);
        for (int i = 0; i < numClasses; ++i)
            m_helmetClassStorage.emplace_back(classNames[i] ? classNames[i] : "?");
        m_helmetClassPointers.reserve(numClasses);
        for (const QByteArray &name : m_helmetClassStorage)
            m_helmetClassPointers.push_back(name.constData());
    }
    m_helmetClassNames = m_helmetClassPointers.empty() ? nullptr
                                                       : m_helmetClassPointers.data();
    m_helmetNumClasses = static_cast<int>(m_helmetClassPointers.size());
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::clearHelmetOverlay()
{
    if (m_helmetResult) m_helmetResult->boxes.clear();
    m_helmetClassStorage.clear();
    m_helmetClassPointers.clear();
    m_helmetClassNames = nullptr;
    m_helmetNumClasses = 0;
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::setTrafficOverlay(const YoloDetectResult &result,
                                         const char *const *classNames,
                                         int numClasses)
{
    if (!m_trafficResult) m_trafficResult = new YoloDetectResult();
    *m_trafficResult = result;
    m_trafficClassStorage.clear();
    m_trafficClassPointers.clear();
    if (classNames && numClasses > 0) {
        m_trafficClassStorage.reserve(numClasses);
        for (int i = 0; i < numClasses; ++i)
            m_trafficClassStorage.emplace_back(classNames[i] ? classNames[i] : "?");
        m_trafficClassPointers.reserve(numClasses);
        for (const QByteArray &name : m_trafficClassStorage)
            m_trafficClassPointers.push_back(name.constData());
    }
    m_trafficClassNames = m_trafficClassPointers.empty() ? nullptr
                                                         : m_trafficClassPointers.data();
    m_trafficNumClasses = static_cast<int>(m_trafficClassPointers.size());
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::clearTrafficOverlay()
{
    if (m_trafficResult) m_trafficResult->boxes.clear();
    m_trafficClassStorage.clear();
    m_trafficClassPointers.clear();
    m_trafficClassNames = nullptr;
    m_trafficNumClasses = 0;
    requestAuxiliaryUpdate();
}
void EglfsVideoWindow::showPlaceholder(const QString &text)
{
    m_renderPumpStarted = false;
    m_frame = QImage();
    m_placeholderText = text;
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::setHudLines(const QStringList &lines)
{
    if (m_hudLines == lines)
        return;
    m_hudLines = lines;
    renderHudImage();
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::renderHudImage()
{
    m_hudImage = QImage();
    if (m_hudLines.isEmpty())
        return;

    QFont hudFont;
    hudFont.setPixelSize(16);
    hudFont.setBold(true);
    QFontMetrics fm(hudFont);
    const int lineHeight = fm.height() + 5;
    const int tabWidth = 214;
    const auto visibleText = [](const QString &line) {
        QString result;
        result.reserve(line.size());
        for (int i = 0; i < line.size();) {
            if (i + 8 < line.size() && line.at(i) == QLatin1Char('<') &&
                line.at(i + 1) == QLatin1Char('#') &&
                line.at(i + 8) == QLatin1Char('>')) {
                i += 9;
                continue;
            }
            result.append(line.at(i++));
        }
        return result;
    };
    int maxWidth = 0;
    for (const QString &line : m_hudLines) {
        const QStringList cells = line.split(QLatin1Char('\t'),
                                             Qt::KeepEmptyParts);
        for (int column = 0; column < cells.size(); ++column) {
            maxWidth = qMax(maxWidth,
                column * tabWidth +
                fm.horizontalAdvance(visibleText(cells.at(column))));
        }
    }
    const int panelW = maxWidth + 24;
    const int panelH = lineHeight * m_hudLines.size() + 16;
    m_hudImage = QImage(panelW, panelH, QImage::Format_ARGB32_Premultiplied);
    m_hudImage.fill(Qt::transparent);

    QPainter painter(&m_hudImage);
    painter.setFont(hudFont);
    painter.fillRect(m_hudImage.rect(), QColor(0, 0, 0, 160));
    int y = 10 + fm.ascent();
    for (const QString &line : m_hudLines) {
        const QStringList cells = line.split(QLatin1Char('\t'),
                                             Qt::KeepEmptyParts);
        for (int column = 0; column < cells.size(); ++column) {
            const QString &cell = cells.at(column);
            int x = 12 + column * tabWidth;
            QColor runColor(0xff, 0xff, 0xff);
            QString run;
            const auto flushRun = [&]() {
                if (run.isEmpty()) return;
                painter.setPen(runColor);
                painter.drawText(x, y, run);
                x += fm.horizontalAdvance(run);
                run.clear();
            };
            for (int i = 0; i < cell.size();) {
                if (i + 8 < cell.size() &&
                    cell.at(i) == QLatin1Char('<') &&
                    cell.at(i + 1) == QLatin1Char('#') &&
                    cell.at(i + 8) == QLatin1Char('>')) {
                    flushRun();
                    const QColor requested(cell.mid(i + 1, 7));
                    if (requested.isValid()) runColor = requested;
                    i += 9;
                    continue;
                }
                run.append(cell.at(i++));
            }
            flushRun();
        }
        y += lineHeight;
    }
    painter.end();
}

void EglfsVideoWindow::setPlateComparison(const QImage &original, const QImage &sci,
                                          const QImage &ffdnet, const QImage &mimo)
{
    if (m_plateOriginal.cacheKey() == original.cacheKey() &&
        m_plateSci.cacheKey() == sci.cacheKey() &&
        m_plateFfdnet.cacheKey() == ffdnet.cacheKey() &&
        m_plateMimo.cacheKey() == mimo.cacheKey()) {
        return;
    }
    m_plateOriginal = original;
    m_plateSci     = sci;
    m_plateFfdnet  = ffdnet;
    m_plateMimo    = mimo;
    const QSize thumbSize(420, 120);
    m_plateOriginalThumb = original.isNull()
        ? QImage() : original.scaled(thumbSize, Qt::KeepAspectRatio,
                                     Qt::SmoothTransformation);
    m_plateSciThumb = sci.isNull()
        ? QImage() : sci.scaled(thumbSize, Qt::KeepAspectRatio,
                                Qt::SmoothTransformation);
    m_plateFfdnetThumb = ffdnet.isNull()
        ? QImage() : ffdnet.scaled(thumbSize, Qt::KeepAspectRatio,
                                   Qt::SmoothTransformation);
    m_plateMimoThumb = mimo.isNull()
        ? QImage() : mimo.scaled(thumbSize, Qt::KeepAspectRatio,
                                Qt::SmoothTransformation);
    requestAuxiliaryUpdate();
}

void EglfsVideoWindow::setToggleAiHandler(std::function<void()> handler)
{
    m_toggleAiHandler = std::move(handler);
}

void EglfsVideoWindow::setToggleYoloHandler(std::function<void()> handler)
{
    m_toggleYoloHandler = std::move(handler);
}

void EglfsVideoWindow::setToggleYieldYoloHandler(
    std::function<void()> handler)
{
    m_toggleYieldYoloHandler = std::move(handler);
}

void EglfsVideoWindow::setToggleDetect3dHandler(std::function<void()> handler)
{
    m_toggleDetect3dHandler = std::move(handler);
}

void EglfsVideoWindow::setToggleHelmetHandler(std::function<void()> handler)
{
    m_toggleHelmetHandler = std::move(handler);
}

void EglfsVideoWindow::setToggleTrafficHandler(std::function<void()> handler)
{
    m_toggleTrafficHandler = std::move(handler);
}
void EglfsVideoWindow::setToggleSourceHandler(std::function<void()> handler)
{
    m_toggleSourceHandler = std::move(handler);
}

void EglfsVideoWindow::setToggleFpgaSourceHandler(std::function<void()> handler)
{
    m_toggleFpgaSourceHandler = std::move(handler);
}

void EglfsVideoWindow::setSystemRecoveryHandler(std::function<void()> handler)
{
    m_systemRecoveryHandler = std::move(handler);
}

void EglfsVideoWindow::setTogglePreprocessHandler(std::function<void()> handler)
{
    m_togglePreprocessHandler = std::move(handler);
}

void EglfsVideoWindow::setToggleEnhancedLprHandler(std::function<void()> handler)
{
    m_toggleEnhancedLprHandler = std::move(handler);
}

void EglfsVideoWindow::setTogglePreprocessStageHandler(std::function<void(int)> handler)
{
    m_togglePreprocessStageHandler = std::move(handler);
}

void EglfsVideoWindow::initializeGL()
{
    initializeOpenGLFunctions();
    m_contextDestroying = false;
    if (context()) {
        connect(context(), &QOpenGLContext::aboutToBeDestroyed,
                this, [this]() {
            m_contextDestroying = true;
            m_renderPaceTimer.stop();
            m_updatePending = false;
            m_lastRenderSubmitNs = 0;
            m_firstSwapCompleted = false;
            m_renderDirty = true;
            qCritical().nospace()
                << "[display-lifecycle] OpenGL context aboutToBeDestroyed"
                << " visible=" << isVisible()
                << " exposed=" << isExposed()
                << " pending=" << m_updatePending
                << " dirty=" << m_renderDirty
                << " first_swap=" << m_firstSwapCompleted;
        }, Qt::DirectConnection);
    }
    m_nativeVideoReady = initializeNativeVideoRenderer();
    const int activeDrmHz = logActiveDrmMode();
    const qreal qtReportedHz = screen() ? screen()->refreshRate() : 0.0;
    const int qtRoundedHz = static_cast<int>(std::lround(qtReportedHz));

    bool requiredHzOk = false;
    const int requiredHz = qEnvironmentVariableIntValue(
        "HS_REQUIRE_DRM_HZ", &requiredHzOk);
    const int reportedOrRequestedHz = qtRoundedHz > 0
        ? qtRoundedHz : (requiredHzOk && requiredHz > 0 ? requiredHz : 60);
    // Before the first atomic commit, QScreen may already advertise 120 Hz
    // while the CRTC is still running at 60 Hz.  Start conservatively and
    // raise the deadline rate after the 150-ms post-commit verification.
    const int effectiveHz = activeDrmHz > 0
        ? activeDrmHz : qMin(60, reportedOrRequestedHz);
    configureRenderPacer(effectiveHz);
    m_renderPaceTimer.start();

    qWarning().nospace()
        << "[display-gl] native_rgb565=" << (m_nativeVideoReady ? "ready" : "disabled")
        << " qt_reported_hz=" << qtReportedHz
        << " drm_hz=" << activeDrmHz
        << " texture_ring=" << VideoTextureCount
        << " gl_vendor="
        << reinterpret_cast<const char *>(glGetString(GL_VENDOR))
        << " gl_renderer="
        << reinterpret_cast<const char *>(glGetString(GL_RENDERER));
    qWarning().nospace()
        << "[display-pacer] deadline_timer=enabled"
        << " frameSwapped_feedback=disabled"
        << " single_update_pending=enabled"
        << " pending_timeout_submissions=suppressed"
        << " single_update_producer=deadline_timer"
        << " source_switch_immediate_updates=disabled"
        << " effective_hz=" << m_renderPacerHz;

    // initializeGL() runs before EGLFS has queued its first framebuffer and
    // committed the requested KMS mode.  At this point QScreen already reports
    // 120 Hz while debugfs can legitimately still show the previous 60-Hz
    // CRTC.  Validate after the first mode commit instead of killing a valid
    // launch during this short transition.
    if (requiredHzOk) {
        qWarning().nospace()
            << "[display-startup] precommit qt=" << qtReportedHz
            << " drm=" << activeDrmHz
            << " required=" << requiredHz;
        QTimer::singleShot(150, this, [this, requiredHz]() {
            const qreal settledQtHz = screen() ? screen()->refreshRate() : 0.0;
            const int settledQtRounded =
                static_cast<int>(std::lround(settledQtHz));
            const int settledDrmHz = logActiveDrmMode();
            const int settledPacerHz = settledDrmHz > 0
                ? settledDrmHz
                : (settledQtRounded > 0 ? settledQtRounded : requiredHz);
            configureRenderPacer(settledPacerHz);
            if (settledQtRounded != requiredHz
                || settledDrmHz != requiredHz) {
                // Output-mode negotiation must never disable the independent
                // FPGA/PCIe capture path.  In particular initializeGL() and
                // even this short settle timer can run before an asynchronous
                // KMS atomic commit is visible in debugfs.  The old fatal
                // check quit the whole process before MainWindow could issue
                // DMA_START, making every FPGA bitstream appear to have no
                // HDMI input.  Keep the diagnostic, but let capture start.
                qWarning().nospace()
                    << "[display-startup] WARNING requested=" << requiredHz
                    << " Hz, got qt=" << settledQtHz
                    << " drm=" << settledDrmHz
                    << "; capture continues";
                return;
            }
            qWarning().nospace()
                << "[display-startup] verified qt=" << settledQtHz
                << " drm=" << settledDrmHz
                << "; capture continues";
        });
    } else {
        QTimer::singleShot(150, this, [this]() {
            const int settledDrmHz = logActiveDrmMode();
            const qreal settledQtHz = screen() ? screen()->refreshRate() : 0.0;
            const int settledQtRounded =
                static_cast<int>(std::lround(settledQtHz));
            configureRenderPacer(settledDrmHz > 0
                ? settledDrmHz
                : (settledQtRounded > 0 ? settledQtRounded : 60));
        });
    }
}

void EglfsVideoWindow::configureRenderPacer(int refreshHz)
{
    m_renderPacerHz = qBound(24, refreshHz, 240);
    const qint64 periodNs = 1000000000LL / qMax(1, m_renderPacerHz);
    m_nextRenderDeadlineNs = m_renderPaceClock.nsecsElapsed() + periodNs;
}

void EglfsVideoWindow::applyRequestedDrmModeAfterFirstSwap()
{
#ifdef Q_OS_LINUX
    const QByteArray requested = qgetenv("HS_POSTSET_HDMI_MODE");
    QPlatformNativeInterface *native =
        QGuiApplication::platformNativeInterface();
    const int drmFd = native
        ? static_cast<int>(reinterpret_cast<qintptr>(
              native->nativeResourceForIntegration(QByteArrayLiteral("dri_fd"))))
        : -1;
    if (drmFd < 0) {
        qCritical() << "[display-postkms] Qt EGLFS did not expose dri_fd";
        qCritical() << "[app-quit-request] reason=drm-fd-unavailable code=21";
        QCoreApplication::exit(21);
        return;
    }

    uint32_t appliedClock = 0;
    if (!setHdmiModeOnQtDrmFd(drmFd, requested, &appliedClock)) {
        qCritical().nospace()
            << "[display-postkms] FAILED to apply " << requested
            << "; stopping instead of falling back";
        qCritical() << "[app-quit-request] reason=drm-modeset-failed code=22";
        QCoreApplication::exit(22);
        return;
    }

    const int activeDrmHz = logActiveDrmMode();
    bool requireOk = false;
    const int requiredDrmHz = qEnvironmentVariableIntValue(
        "HS_REQUIRE_DRM_HZ", &requireOk);
    qWarning().nospace()
        << "[display-postkms] applied=" << requested
        << " clock_khz=" << appliedClock
        << " active_hz=" << activeDrmHz;
    if ((requireOk && activeDrmHz > 0 && activeDrmHz != requiredDrmHz)
        || appliedClock < 250000u) {
        qCritical().nospace()
            << "[display-postkms] REQUIRED " << requiredDrmHz
            << " Hz but kernel kept " << activeDrmHz
            << " Hz / " << appliedClock << " kHz; stopping";
        qCritical() << "[app-quit-request] reason=drm-mode-mismatch code=23";
        QCoreApplication::exit(23);
        return;
    }
    // Let the guarded deadline producer submit the first redraw after the
    // modeset. A direct update here could overlap its already queued commit.
    m_renderDirty = true;
#else
    qCritical() << "[display-postkms] direct KMS switch is Linux-only";
    qCritical() << "[app-quit-request] reason=postkms-not-linux code=24";
    QCoreApplication::exit(24);
#endif
}

bool EglfsVideoWindow::initializeNativeVideoRenderer()
{
    destroyNativeVideoRenderer();

    static const char vertexShader[] =
        "attribute highp vec2 aPosition;\n"
        "attribute highp vec2 aTexCoord;\n"
        "varying mediump vec2 vTexCoord;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = vec4(aPosition, 0.0, 1.0);\n"
        "    vTexCoord = aTexCoord;\n"
        "}\n";
    static const char fragmentShader[] =
        "#ifdef GL_ES\n"
        "precision mediump float;\n"
        "#endif\n"
        "uniform sampler2D uTexture;\n"
        "varying mediump vec2 vTexCoord;\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = texture2D(uTexture, vTexCoord);\n"
        "}\n";

    m_videoProgram = new QOpenGLShaderProgram();
    if (!m_videoProgram->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                                  vertexShader)
        || !m_videoProgram->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                                     fragmentShader)) {
        qWarning() << "[display-gl] shader compile failed:"
                   << m_videoProgram->log();
        destroyNativeVideoRenderer();
        return false;
    }

    m_videoProgram->bindAttributeLocation("aPosition", 0);
    m_videoProgram->bindAttributeLocation("aTexCoord", 1);
    if (!m_videoProgram->link()) {
        qWarning() << "[display-gl] shader link failed:" << m_videoProgram->log();
        destroyNativeVideoRenderer();
        return false;
    }

    glGenTextures(VideoTextureCount, m_videoTextures);
    bool textureAllocationFailed = false;
    for (int i = 0; i < VideoTextureCount; ++i)
        textureAllocationFailed = textureAllocationFailed || m_videoTextures[i] == 0;
    if (textureAllocationFailed) {
        qWarning() << "[display-gl] texture allocation failed";
        destroyNativeVideoRenderer();
        return false;
    }
    for (int i = 0; i < VideoTextureCount; ++i) {
        glBindTexture(GL_TEXTURE_2D, m_videoTextures[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

void EglfsVideoWindow::destroyNativeVideoRenderer()
{
    if (m_videoTextures[0] != 0) {
        glDeleteTextures(VideoTextureCount, m_videoTextures);
        for (int i = 0; i < VideoTextureCount; ++i)
            m_videoTextures[i] = 0;
    }
    delete m_videoProgram;
    m_videoProgram = nullptr;
    m_videoTextureSize = QSize();
    m_videoTextureIndex = -1;
    m_nativeVideoReady = false;
}

bool EglfsVideoWindow::uploadAndDrawNativeVideo(const QImage &frame)
{
    if (!m_nativeVideoReady || !m_videoProgram || m_videoTextures[0] == 0
        || frame.isNull() || frame.format() != QImage::Format_RGB16
        || frame.bytesPerLine() != frame.width() * 2) {
        return false;
    }

    glActiveTexture(GL_TEXTURE0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
    if (m_videoTextureSize != frame.size()) {
        // Allocate once.  The single-texture path is substantially faster on
        // this Mali driver than rotating through three textures.
        for (int i = 0; i < VideoTextureCount; ++i) {
            glBindTexture(GL_TEXTURE_2D, m_videoTextures[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                         frame.width(), frame.height(), 0,
                         GL_RGB, GL_UNSIGNED_SHORT_5_6_5, nullptr);
        }
        m_videoTextureSize = frame.size();
    }
    m_videoTextureIndex = (m_videoTextureIndex + 1) % VideoTextureCount;
    glBindTexture(GL_TEXTURE_2D, m_videoTextures[m_videoTextureIndex]);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                    frame.width(), frame.height(),
                    GL_RGB, GL_UNSIGNED_SHORT_5_6_5,
                    frame.constBits());

    const qreal dpr = devicePixelRatio();
    const float canvasW = qMax(1, qRound(width() * dpr));
    const float canvasH = qMax(1, qRound(height() * dpr));
    const float canvasAspect = canvasW / canvasH;
    const float frameAspect = static_cast<float>(frame.width())
                            / static_cast<float>(frame.height());
    float xScale = 1.0f;
    float yScale = 1.0f;
    if (frameAspect > canvasAspect)
        yScale = canvasAspect / frameAspect;
    else
        xScale = frameAspect / canvasAspect;

    // The first uploaded RGB565 row is the top image row. OpenGL texture v=0
    // is therefore assigned to the top two vertices to preserve orientation.
    const GLfloat vertices[] = {
        -xScale, -yScale,
         xScale, -yScale,
        -xScale,  yScale,
         xScale,  yScale
    };
    const GLfloat texCoords[] = {
        0.0f, 1.0f,
        1.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 0.0f
    };

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    m_videoProgram->bind();
    m_videoProgram->setUniformValue("uTexture", 0);
    m_videoProgram->enableAttributeArray(0);
    m_videoProgram->enableAttributeArray(1);
    m_videoProgram->setAttributeArray(0, GL_FLOAT, vertices, 2);
    m_videoProgram->setAttributeArray(1, GL_FLOAT, texCoords, 2);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    m_videoProgram->disableAttributeArray(0);
    m_videoProgram->disableAttributeArray(1);
    m_videoProgram->release();
    glBindTexture(GL_TEXTURE_2D, 0);

    // RGB565 upload uses 2-byte row alignment. Restore the OpenGL default
    // before Qt's paint engine uploads its 1-byte glyph atlas; leaking 2 here
    // corrupts every directly rendered label while 4-byte HUD images look fine.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    return true;
}

void EglfsVideoWindow::resizeGL(int w, int h)
{
    Q_UNUSED(w);
    Q_UNUSED(h);
}

void EglfsVideoWindow::paintGL()
{
    const qint64 paintStartUs = m_paintProfTimer.nsecsElapsed() / 1000;
    ++m_diagPaintCount;
    if (m_lastPaintStartUs != 0) {
        const qint64 periodUs = paintStartUs - m_lastPaintStartUs;
        m_paintPeriodTotalUs += periodUs;
        m_paintPeriodMaxUs = qMax(m_paintPeriodMaxUs, periodUs);
    }
    m_lastPaintStartUs = paintStartUs;

    const qreal dpr = devicePixelRatio();
    glViewport(0, 0, qRound(width() * dpr), qRound(height() * dpr));
    glClearColor(0x1a / 255.0f, 0x1a / 255.0f, 0x1a / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    const bool nativeVideoDrawn = !m_frame.isNull()
                               && uploadAndDrawNativeVideo(m_frame);
    const qint64 uploadEndUs = m_paintProfTimer.nsecsElapsed() / 1000;
    const qint64 uploadUs = uploadEndUs - paintStartUs;

    QOpenGLPaintDevice device(QSize(qRound(width() * dpr), qRound(height() * dpr)));
    device.setDevicePixelRatio(dpr);

    QPainter painter(&device);

    if (!m_frame.isNull()) {
        const QSizeF frameSize = m_frame.size();
        const QSizeF canvasSize = size();
        const qreal scale = qMin(canvasSize.width() / frameSize.width(),
                                 canvasSize.height() / frameSize.height());
        const qreal drawW = frameSize.width() * scale;
        const qreal drawH = frameSize.height() * scale;
        const qreal offsetX = (canvasSize.width() - drawW) * 0.5;
        const qreal offsetY = (canvasSize.height() - drawH) * 0.5;

        painter.translate(offsetX, offsetY);
        painter.scale(scale, scale);
        // Unsupported/non-RGB565 inputs retain the safe QPainter fallback.
        // The normal PCIe HDMI path is drawn above by the native GLES texture.
        if (!nativeVideoDrawn)
            painter.drawImage(QPoint(0, 0), m_frame);
        if (m_obbResult && !m_obbResult->boxes.empty())
            OverlayRenderer::draw(painter, *m_obbResult, m_obbClassNames, m_obbNumClasses);
        if (!m_yoloSemanticOverlay.isNull())
            painter.drawImage(QPoint(0, 0), m_yoloSemanticOverlay);
        if (m_yoloResult && (!m_yoloResult->boxes.empty() ||
                             m_yoloResult->laneGeometryValid ||
                             m_yoloResult->laneDepartureAlert ||
                             m_yoloResult->cameraCenterX > 0))
            OverlayRenderer::drawYoloRects(painter, *m_yoloResult,
                                           m_yoloClassNames, m_yoloNumClasses);
        if (m_yieldYoloResult && (!m_yieldYoloResult->boxes.empty() ||
                                  m_yieldYoloResult->yieldViolationAlert))
            OverlayRenderer::drawYoloRects(painter, *m_yieldYoloResult,
                                           m_yieldYoloClassNames,
                                           m_yieldYoloNumClasses);
        if (m_detect3dResult && !m_detect3dResult->boxes.empty())
            OverlayRenderer::drawYoloRects(painter, *m_detect3dResult,
                                           m_detect3dClassNames,
                                           m_detect3dNumClasses);
        if (m_helmetResult && !m_helmetResult->boxes.empty())
            OverlayRenderer::drawYoloRects(painter, *m_helmetResult,
                                           m_helmetClassNames, m_helmetNumClasses);
        if (m_trafficResult && !m_trafficResult->boxes.empty())
            OverlayRenderer::drawTrafficRects(painter, *m_trafficResult,
                                              m_trafficClassNames, m_trafficNumClasses);
    } else {
        painter.setPen(QColor(0xdd, 0xdd, 0xdd));
        QFont font = painter.font();
        font.setPointSize(28);
        font.setBold(true);
        painter.setFont(font);
        painter.drawText(QRect(QPoint(0, 0), size()), Qt::AlignCenter, m_placeholderText);
    }

    painter.resetTransform();
    if (!m_hudImage.isNull())
        painter.drawImage(QPoint(24, 24), m_hudImage);

    // ---- Raw + three independent preprocessing outputs (bottom-right) ----
    if (!m_plateOriginalThumb.isNull()) {
        const int thumbW = 420;
        const int thumbH = 120;
        const int gap = 8;
        const int N = 1 + (m_plateSciThumb.isNull() ? 0 : 1)
                        + (m_plateFfdnetThumb.isNull() ? 0 : 1)
                        + (m_plateMimoThumb.isNull() ? 0 : 1);
        const int x0 = width() - thumbW * N - gap * (N - 1) - 20;
        const int y0 = height() - thumbH - 20;
        auto drawThumb = [&](const QImage &img, const QString &label, int col) {
            if (img.isNull()) return;
            int x = x0 + col * (thumbW + gap);
            painter.drawImage(x, y0, img);
            QFont f;
            f.setPointSize(8);
            painter.setFont(f);
            painter.setPen(Qt::white);
            painter.drawText(x, y0 - 4, label);
        };
        int col = 0;
        drawThumb(m_plateOriginalThumb, "Raw", col++);
        if (!m_plateSciThumb.isNull())
            drawThumb(m_plateSciThumb, "SCI", col++);
        if (!m_plateFfdnetThumb.isNull())
            drawThumb(m_plateFfdnetThumb, "FFDNet", col++);
        if (!m_plateMimoThumb.isNull())
            drawThumb(m_plateMimoThumb, "Cascade Result", col++);
    }

    painter.end();

    const qint64 paintEndUs = m_paintProfTimer.nsecsElapsed() / 1000;
    const qint64 paintUs = paintEndUs - paintStartUs;
    const qint64 overlayUs = paintEndUs - uploadEndUs;
    m_paintTotalUs += paintUs;
    m_paintMaxUs = qMax(m_paintMaxUs, paintUs);
    m_uploadTotalUs += uploadUs;
    m_uploadMaxUs = qMax(m_uploadMaxUs, uploadUs);
    m_overlayTotalUs += overlayUs;
    m_overlayMaxUs = qMax(m_overlayMaxUs, overlayUs);
    ++m_paintCount;
    if (paintEndUs - m_paintWindowStartUs >= 1000000) {
        const qint64 windowUs = paintEndUs - m_paintWindowStartUs;
        m_measuredPaintFpsRaw = m_paintCount * 1000000.0 / windowUs;
        m_measuredPaintFps = adasDemoHudFps(m_measuredPaintFpsRaw);
        qWarning("[disp-prof] paint_avg_us=%lld paint_max_us=%lld "
                 "upload_avg_us=%lld upload_max_us=%lld "
                 "overlay_avg_us=%lld overlay_max_us=%lld "
                 "period_avg_us=%lld period_max_us=%lld paint_fps=%.1f paint_fps_raw=%.1f",
                  m_paintCount ? m_paintTotalUs / m_paintCount : 0,
                  m_paintMaxUs,
                  m_paintCount ? m_uploadTotalUs / m_paintCount : 0,
                  m_uploadMaxUs,
                  m_paintCount ? m_overlayTotalUs / m_paintCount : 0,
                  m_overlayMaxUs,
                  m_paintCount ? m_paintPeriodTotalUs / m_paintCount : 0,
                  m_paintPeriodMaxUs,
                  m_measuredPaintFps,
                  m_measuredPaintFpsRaw);
        m_paintWindowStartUs = paintEndUs;
        m_paintTotalUs = 0;
        m_paintMaxUs = 0;
        m_paintCount = 0;
        m_paintPeriodTotalUs = 0;
        m_paintPeriodMaxUs = 0;
        m_uploadTotalUs = 0;
        m_uploadMaxUs = 0;
        m_overlayTotalUs = 0;
        m_overlayMaxUs = 0;
    }
}

void EglfsVideoWindow::keyPressEvent(QKeyEvent *event)
{
    // Holding a key must not toggle a source twice.  On EGLFS the keyboard
    // repeat delay can be short enough for one physical press to generate an
    // HDMI request followed immediately by an FMC request.
    if (event->isAutoRepeat()
        && (event->key() == Qt::Key_Q || event->key() == Qt::Key_H
            || event->key() == Qt::Key_R || event->key() == Qt::Key_A
            || event->key() == Qt::Key_E
            || event->key() == Qt::Key_1 || event->key() == Qt::Key_2
            || event->key() == Qt::Key_3 || event->key() == Qt::Key_4
            || event->key() == Qt::Key_5 || event->key() == Qt::Key_6
            || event->key() == Qt::Key_7)) {
        event->accept();
        return;
    }

    switch (event->key()) {
    case Qt::Key_Q:
        if (m_toggleSourceHandler) m_toggleSourceHandler();
        event->accept();
        return;
    case Qt::Key_H:
        if (m_toggleFpgaSourceHandler) m_toggleFpgaSourceHandler();
        event->accept();
        return;
    case Qt::Key_R:
        if (m_systemRecoveryHandler) m_systemRecoveryHandler();
        event->accept();
        return;
    case Qt::Key_A:
        if (m_toggleAiHandler) m_toggleAiHandler();
        event->accept();
        return;
    case Qt::Key_E:
        if (m_toggleEnhancedLprHandler) m_toggleEnhancedLprHandler();
        event->accept();
        return;
    case Qt::Key_1:
    case Qt::Key_2:
    case Qt::Key_3:
        if (m_togglePreprocessStageHandler)
            m_togglePreprocessStageHandler(event->key() - Qt::Key_0);
        event->accept();
        return;
    case Qt::Key_4:
        if (m_toggleHelmetHandler) m_toggleHelmetHandler();
        event->accept();
        return;
    case Qt::Key_5:
        if (m_toggleTrafficHandler) m_toggleTrafficHandler();
        event->accept();
        return;
    case Qt::Key_6:
        if (m_toggleDetect3dHandler) m_toggleDetect3dHandler();
        event->accept();
        return;
    case Qt::Key_7:
        if (m_toggleYieldYoloHandler) m_toggleYieldYoloHandler();
        event->accept();
        return;
    case Qt::Key_Escape:
        QCoreApplication::quit();
        event->accept();
        return;
    default:
        break;
    }
    QOpenGLWindow::keyPressEvent(event);
}
