#ifndef EGLFS_VIDEO_WINDOW_H
#define EGLFS_VIDEO_WINDOW_H

#include <QImage>
#include <QElapsedTimer>
#include <QOpenGLFunctions>
#include <QOpenGLWindow>
#include <QString>
#include <QStringList>
#include <QTimer>

#include <functional>
#include <QByteArray>
#include <vector>

struct DetectResult;
struct YoloDetectResult;
class QOpenGLShaderProgram;

class EglfsVideoWindow : public QOpenGLWindow, protected QOpenGLFunctions
{
public:
    EglfsVideoWindow();
    ~EglfsVideoWindow() override;

    // Called by the global Qt message handler when the Rockchip EGLFS/KMS
    // backend reports a rejected page flip.  The notification can originate
    // while Qt is inside swapBuffers(), so recovery is always queued.
    static void notifyDrmPageFlipFailure();

    void presentFrame(const QImage &frame);
    void setObbOverlay(const DetectResult &result,
                       const char *const *classNames, int numClasses);
    void clearObbOverlay();
    void setYoloOverlay(const YoloDetectResult &result,
                        const char *const *classNames, int numClasses);
    void clearYoloOverlay();
    void setYieldYoloOverlay(const YoloDetectResult &result,
                             const char *const *classNames, int numClasses);
    void clearYieldYoloOverlay();
    void setDetect3dOverlay(const YoloDetectResult &result,
                            const char *const *classNames, int numClasses);
    void clearDetect3dOverlay();
    void setHelmetOverlay(const YoloDetectResult &result,
                          const char *const *classNames, int numClasses);
    void clearHelmetOverlay();
    void setTrafficOverlay(const YoloDetectResult &result,
                           const char *const *classNames, int numClasses);
    void clearTrafficOverlay();
    void showPlaceholder(const QString &text);
    void setHudLines(const QStringList &lines);
    void setPlateComparison(const QImage &original, const QImage &sci,
                            const QImage &ffdnet, const QImage &mimo);
    void setToggleAiHandler(std::function<void()> handler);
    void setToggleYoloHandler(std::function<void()> handler);
    void setToggleYieldYoloHandler(std::function<void()> handler);
    void setToggleDetect3dHandler(std::function<void()> handler);
    void setToggleHelmetHandler(std::function<void()> handler);
    void setToggleTrafficHandler(std::function<void()> handler);
    void setToggleSourceHandler(std::function<void()> handler);
    void setToggleFpgaSourceHandler(std::function<void()> handler);
    void setTogglePreprocessHandler(std::function<void()> handler);
    void setTogglePreprocessStageHandler(std::function<void(int)> handler);
    void setToggleEnhancedLprHandler(std::function<void()> handler);
    void setSystemRecoveryHandler(std::function<void()> handler);
    double measuredPaintFps() const { return m_measuredPaintFps; }
    bool hasCompletedFirstSwap() const { return m_firstSwapCompleted; }

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    QImage            m_frame;
    QString           m_placeholderText;
    DetectResult     *m_obbResult;
    const char *const *m_obbClassNames;
    int               m_obbNumClasses;
    std::vector<QByteArray> m_obbClassStorage;
    std::vector<const char *> m_obbClassPointers;
    YoloDetectResult *m_yoloResult;
    QImage            m_yoloSemanticOverlay;
    const char *const *m_yoloClassNames;
    int               m_yoloNumClasses;
    std::vector<QByteArray> m_yoloClassStorage;
    std::vector<const char *> m_yoloClassPointers;
    YoloDetectResult *m_yieldYoloResult;
    const char *const *m_yieldYoloClassNames;
    int               m_yieldYoloNumClasses;
    std::vector<QByteArray> m_yieldYoloClassStorage;
    std::vector<const char *> m_yieldYoloClassPointers;
    YoloDetectResult *m_detect3dResult;
    const char *const *m_detect3dClassNames;
    int m_detect3dNumClasses;
    std::vector<QByteArray> m_detect3dClassStorage;
    std::vector<const char *> m_detect3dClassPointers;
    YoloDetectResult *m_helmetResult;
    const char *const *m_helmetClassNames;
    int               m_helmetNumClasses;
    std::vector<QByteArray> m_helmetClassStorage;
    std::vector<const char *> m_helmetClassPointers;
    YoloDetectResult *m_trafficResult;
    const char *const *m_trafficClassNames;
    int               m_trafficNumClasses;
    std::vector<QByteArray> m_trafficClassStorage;
    std::vector<const char *> m_trafficClassPointers;
    QStringList       m_hudLines;
    std::function<void()> m_toggleAiHandler;
    std::function<void()> m_toggleYoloHandler;
    std::function<void()> m_toggleYieldYoloHandler;
    std::function<void()> m_toggleDetect3dHandler;
    std::function<void()> m_toggleHelmetHandler;
    std::function<void()> m_toggleTrafficHandler;
    std::function<void()> m_toggleSourceHandler;
    std::function<void()> m_toggleFpgaSourceHandler;
    std::function<void()> m_togglePreprocessHandler;
    std::function<void(int)> m_togglePreprocessStageHandler;
    std::function<void()> m_toggleEnhancedLprHandler;
    std::function<void()> m_systemRecoveryHandler;
    QImage m_plateOriginal;
    QImage m_plateSci;
    QImage m_plateFfdnet;
    QImage m_plateMimo;
    QImage m_plateOriginalThumb;
    QImage m_plateSciThumb;
    QImage m_plateFfdnetThumb;
    QImage m_plateMimoThumb;

    // Persistent native RGB565 renderer.  QPainter::drawImage() recreated or
    // converted the 1280x720 DMA-backed image every frame and took about
    // 20.5 ms on RK3568.  A fixed GLES texture accepts RGB565 directly and
    // leaves QPainter responsible only for the lightweight overlays/HUD.
    QOpenGLShaderProgram *m_videoProgram = nullptr;
    // One texture is the measured fast path on this Mali-G52: cached RGB565
    // uploads take about 1.7 ms.  A three-texture ring raised the measured
    // upload cost to 13+ ms on this driver.
    static constexpr int  VideoTextureCount = 1;
    GLuint                m_videoTextures[VideoTextureCount] = {0};
    int                   m_videoTextureIndex = -1;
    QSize                 m_videoTextureSize;
    bool                  m_nativeVideoReady = false;
    bool                  m_renderPumpStarted = false;
    // Qt 5's EGLFS/KMS backend can emit frameSwapped when a flip is queued,
    // before its vblank completion.  Never feed that signal directly back
    // into update(): a deadline pacer caps submissions at the active CRTC Hz.
    int                   m_renderPacerHz = 60;
    QTimer                m_renderPaceTimer;
    QTimer                m_renderDiagTimer;
    QElapsedTimer         m_renderPaceClock;
    qint64                m_nextRenderDeadlineNs = 0;
    qint64                m_lastRenderSubmitNs = 0;
    qint64                m_lastRenderStallLogNs = 0;
    bool                  m_renderDirty = true;
    bool                  m_updatePending = false;
    bool                  m_contextDestroying = false;
    bool                  m_firstSwapCompleted = false;
    bool                  m_postKmsAttempted = false;
    QTimer                m_postKmsSettleTimer;

    // ---- DRM/page-flip scheduler diagnostics (1 Hz aggregate) ----
    quint64               m_diagTimerTicks = 0;
    quint64               m_diagSourceFrames = 0;
    quint64               m_diagSourceCoalesced = 0;
    quint64               m_diagSubmitCount = 0;
    quint64               m_diagPaintCount = 0;
    quint64               m_diagSwapCount = 0;
    quint64               m_diagCompletedSubmissions = 0;
    quint64               m_diagPendingBlocked = 0;
    quint64               m_diagUnexpectedSwaps = 0;
    quint64               m_diagStallWarnings = 0;
    qint64                m_diagSubmitToSwapTotalUs = 0;
    qint64                m_diagSubmitToSwapMaxUs = 0;

    // ---- display performance instrumentation (1 Hz aggregate, qWarning) ----
    QElapsedTimer m_paintProfTimer;
    qint64        m_paintWindowStartUs = 0;
    qint64        m_paintTotalUs = 0;
    qint64        m_paintMaxUs = 0;
    int           m_paintCount = 0;
    qint64        m_lastPaintStartUs = 0;
    qint64        m_paintPeriodTotalUs = 0;
    qint64        m_paintPeriodMaxUs = 0;
    qint64        m_uploadTotalUs = 0;
    qint64        m_uploadMaxUs = 0;
    qint64        m_overlayTotalUs = 0;
    qint64        m_overlayMaxUs = 0;
    double        m_measuredPaintFps = 0.0;
    double        m_measuredPaintFpsRaw = 0.0;  // 原始 paint 帧率（日志用，HUD 显示已钳制）

    // HUD pre-rendered image; rebuilt only when the text changes.
    QImage        m_hudImage;

    void renderHudImage();
    void requestAuxiliaryUpdate();
    void configureRenderPacer(int refreshHz);
    bool initializeNativeVideoRenderer();
    bool uploadAndDrawNativeVideo(const QImage &frame);
    void destroyNativeVideoRenderer();
    void applyRequestedDrmModeAfterFirstSwap();
};

#endif // EGLFS_VIDEO_WINDOW_H
