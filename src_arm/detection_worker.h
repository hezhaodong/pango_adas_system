#ifndef DETECTION_WORKER_H
#define DETECTION_WORKER_H

#include <QObject>
#include <QMutex>
#include <QMetaType>
#include <QImage>
#include <QElapsedTimer>
#include <QString>
#include <deque>
#include <map>
#include <string>
#include "obb_detector.h"
#include "lprnet.h"
#include <atomic>
#include "frame_buffer.h"
#include "plate_preprocess_rknn.h"

Q_DECLARE_METATYPE(DetectResult)

enum PlatePreprocessStageMask {
    PlatePreprocessNone   = 0,
    PlatePreprocessSci    = 1 << 0,
    PlatePreprocessFfdnet = 1 << 1,
    PlatePreprocessDeblur = 1 << 2,
    PlatePreprocessAll    = PlatePreprocessSci | PlatePreprocessFfdnet |
                            PlatePreprocessDeblur,
};

class WifiControl;

// ============================================================================
// DetectionWorker — 自驱动异步推理工作线程
//
// 设计要点 (方案一):
//   - 拥有独立的 ObbDetector / LPRNet 上下文, 与 UI 线程零共享
//   - 自驱动循环: 推理完成后立即取下一帧, 不依赖 UI 线程信号驱动
//   - FrameBuffer 非消耗性读取, 不影响显示线程取帧
//   - OBB 完成后立即发布几何框，LPR 完成后再发布车牌文字
//   - UI 直接在最新显示帧上绘制矢量叠加层，避免整帧 ARGB 转换
//   - 预分配所有缓冲区, 消除运行时内存分配
// ============================================================================

class DetectionWorker : public QObject
{
    Q_OBJECT

public:
    explicit DetectionWorker(QObject *parent = nullptr);
    ~DetectionWorker();

    // ---- 模型加载 (由 UI 线程在工作线程启动后调用) ----
    bool loadModel(const char *modelPath, const char *labelPath,
                   const ObbModelParams &params = ObbModelParams());
    bool loadLprModels(const char *blueModelPath,
                       const char *otherModelPath,
                       const char *doubleYellowModelPath);

    // 设置帧缓冲 (必须在 startInference 之前调用)
    void setFrameBuffer(FrameBuffer *fb) { m_frameBuffer = fb; }
    void setWifiControl(WifiControl *control) { m_wifiControl = control; }

    bool isLoaded() const { return m_detector.isLoaded(); }
    bool isLprLoaded() const { return m_lprnetLoaded; }
    bool isSciLoaded() const { return m_sciLoaded; }
    bool isFfdnetLoaded() const { return m_ffdnetLoaded; }
    bool isMimoLoaded() const { return m_mimoLoaded; }
    void releasePreprocessModels();
    bool loadSciModel(const char *modelPath);
    bool loadFfdnetModel(const char *modelPath);
    bool loadMimoModel(const char *modelPath);
    int  numClasses() const { return m_detector.numClasses(); }
    const char *className(int id) const { return m_detector.className(id); }

    // Latest raw crop and the final output of the enabled serial stages.
    // This path is display-only and is deliberately never used as LPR input.
    bool getLatestPlateComparison(QImage &original, QImage &processed,
                            int &frameSeq, int afterFrameSeq = -1) const
    {
        QMutexLocker lock(&m_platePairMutex);
        if (m_lastOriginalPlate.isNull() || m_lastPlatePairSeq <= afterFrameSeq)
            return false;
        original = m_lastOriginalPlate;
        processed = m_lastMimoPlate;
        frameSeq = m_lastPlatePairSeq;
        return true;
    }

    bool getLatestResult(DetectResult &outResult, int &sourceFrameSeq) const
    {
        QMutexLocker lock(&m_resultMutex);
        if (m_stableResult.boxes.empty() && !m_hasDetection) return false;
        outResult = m_stableResult;
        sourceFrameSeq = m_stableFrameSeq;
        return true;
    }

    // Latest plate text produced by running LPR on the enhanced cascade
    // output. Kept strictly separate from the raw recognition pipeline and
    // never merged into tracks / public IDs.
    bool getLatestEnhancedPlate(QString &plateText, int &frameSeq,
                                int afterFrameSeq = -1) const
    {
        QMutexLocker lock(&m_platePairMutex);
        if (m_lastEnhancedPlate.isEmpty() || m_lastEnhancedSeq <= afterFrameSeq)
            return false;
        plateText = m_lastEnhancedPlate;
        frameSeq = m_lastEnhancedSeq;
        return true;
    }

signals:
    // Publish the completed boxes after character-based public IDs are applied.
    void detectionReady(const DetectResult &result, int frameSeq);

    // OBB + LPR processing for the frame is complete.
    void inferenceDone(int frameSeq);

    // 状态消息
    void statusMessage(const QString &msg);
    void preprocessStateChanged(int enabledMask, const QString &message);

public slots:
    // 启动/停止推理循环
    void startInference();
    void stopInference();
    void resetSceneState();
    void configurePreprocess(int enabledMask,
                             const QString &sciModelPath,
                             const QString &ffdnetModelPath,
                             const QString &mimoModelPath);
    void setEnhancedLprEnabled(bool enabled);

private:
    // 推理主循环 (自驱动: 完成后立即调度下一轮)
    void processNextFrame();
    void assignTrackIds(DetectResult &result, int frameSeq);
    void suppressDuplicatePlateBoxes(DetectResult &result);

    struct PlateTrackState {
        int id;
        int classId;
        int lastSeenFrameSeq;
        int lastLprFrameSeq;
        float cx, cy, w, h;
        float vx, vy;
        std::string stablePlate;
        std::string candidatePlate;
        int candidateMatchCount = 0;
        int candidateMismatchCount = 0;
        std::deque<std::string> recentPlateSamples;
        int publicId = -1;
        std::string identityPlate;
        std::string lastSentPlate;
    };

    bool updatePlateConsensus(PlateTrackState &track,
                              const std::string &recognizedPlate);
    bool updatePlateIdentity(PlateTrackState &track,
                             const std::string &recognizedPlate);
    PlateTrackState *findTrack(int trackId);

    // === 帧缓冲 ===
    FrameBuffer  *m_frameBuffer;        // 共享帧缓冲 (非消耗性读取)
    WifiControl  *m_wifiControl;        // 异步回传已确认车牌图像
    uint8_t      *m_frameBuf565;        // RGB565 源帧
    uint8_t      *m_rgb888Buf;          // RGB888 推理输入

    // === 检测器 ===
    ObbDetector   m_detector;
    std::string   m_modelPath;

    // === LPRNet ===
    bool           m_lprnetLoaded;
    lprnet_context_t m_lprnetBlueCtx;
    lprnet_context_t m_lprnetOtherCtx;
    lprnet_context_t m_lprnetDoubleYellowCtx;

    // === Display-only plate preprocessing branches (SCI / FFDNet / MIMO+) ===
    bool                  m_preprocessEnabled;
    int                   m_preprocessMask;
    bool                  m_sciLoaded;
    bool                  m_ffdnetLoaded;
    bool                  m_mimoLoaded;
    PlatePreprocessorRknn *m_sciPreprocessor;
    PlatePreprocessorRknn *m_ffdnetPreprocessor;
    PlatePreprocessorRknn *m_mimoPreprocessor;
    mutable QMutex        m_platePairMutex;
    QImage                m_lastOriginalPlate;
    QImage                m_lastSciPlate;
    QImage                m_lastFfdnetPlate;
    QImage                m_lastMimoPlate;
    int                   m_lastPlatePairSeq;
    std::atomic<bool>     m_enhancedLprEnabled{false};
    QString               m_lastEnhancedPlate;
    int                   m_lastEnhancedSeq = -1;
    QElapsedTimer         m_preprocessThrottleTimer;

    // === 循环控制 ===
    bool m_running;
    int  m_lastProcessedSeq;
    int  m_nextTrackId;
    int  m_nextPublicTrackId;
    std::vector<PlateTrackState> m_tracks;
    std::map<std::string, int> m_publicPlateIds;

    // === 结果缓存 (m_resultMutex 保护, UI 线程安全读取) ===
    mutable QMutex m_resultMutex;
    DetectResult   m_stableResult;
    int            m_stableFrameSeq;
    bool           m_hasDetection;

};

#endif // DETECTION_WORKER_H
