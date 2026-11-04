// 自驱动异步推理工作线程 — 方案一核心实现
//
// 关键设计:
//   - 推理线程直接从 FrameBuffer 非消耗性读取帧 (不影响显示)
//   - RGB565→RGB888 转换在推理线程中完成 (不占用 UI 线程)
//   - OBB 完成后立即发布框，LPR 结束后再补充字符结果
//   - 自驱动循环: 每轮完成立即取下一帧, 最大化推理吞吐
#include "detection_worker.h"
#include "image_converter.h"
#include <QDir>
#include <QImage>
#include <QByteArray>
#include "wifi_control.h"

#include <QTimer>
#include <QDebug>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <opencv2/imgproc.hpp>

namespace {

static bool isPlateClassId(int classId)
{
    // 新 OBB 模型的四类依次为 blue/yellow/green/doubleline_yellow。
    return classId >= 0 && classId <= 3;
}

constexpr int kTrackMaxAgeFrames = 24;
constexpr size_t kPlateVoteWindow = 10;
constexpr int kPlateConfirmVotes = 7;
constexpr int kPlateNewIdentityDifference = 3;
// These outputs are currently used only by the on-screen comparison.  LPR
// still consumes plateImg, so the slow preview must not monopolize the NPU.
constexpr qint64 kPreprocessPreviewIntervalMs = 1000;

static std::vector<std::string> utf8Characters(const std::string &text)
{
    std::vector<std::string> characters;
    for (size_t offset = 0; offset < text.size();) {
        const unsigned char lead = static_cast<unsigned char>(text[offset]);
        size_t length = 1;
        if ((lead & 0xE0u) == 0xC0u) length = 2;
        else if ((lead & 0xF0u) == 0xE0u) length = 3;
        else if ((lead & 0xF8u) == 0xF0u) length = 4;
        if (offset + length > text.size()) length = 1;
        characters.push_back(text.substr(offset, length));
        offset += length;
    }
    return characters;
}

static int plateCharacterDifference(const std::string &lhs,
                                    const std::string &rhs)
{
    const std::vector<std::string> leftChars = utf8Characters(lhs);
    const std::vector<std::string> rightChars = utf8Characters(rhs);
    const size_t common = std::min(leftChars.size(), rightChars.size());
    int difference = static_cast<int>(
        std::max(leftChars.size(), rightChars.size()) - common);
    for (size_t i = 0; i < common; ++i) {
        if (leftChars[i] != rightChars[i]) ++difference;
    }
    return difference;
}

static float axisAlignedIou(float cx1, float cy1, float w1, float h1,
                            float cx2, float cy2, float w2, float h2)
{
    const float left = std::max(cx1 - w1 * 0.5f, cx2 - w2 * 0.5f);
    const float top = std::max(cy1 - h1 * 0.5f, cy2 - h2 * 0.5f);
    const float right = std::min(cx1 + w1 * 0.5f, cx2 + w2 * 0.5f);
    const float bottom = std::min(cy1 + h1 * 0.5f, cy2 + h2 * 0.5f);
    const float intersection = std::max(0.0f, right - left) *
                               std::max(0.0f, bottom - top);
    const float unionArea = w1 * h1 + w2 * h2 - intersection;
    return unionArea > 1.0f ? intersection / unionArea : 0.0f;
}

static std::string lowerAscii(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

static PlateType yoloClassToPlateType(int classId, const char *className)
{
    // 优先按标签名路由，兼容模型导出时类别 ID 顺序变化。
    const std::string label = lowerAscii(className ? className : "");
    if (label.find("double") != std::string::npos ||
        label.find("双层") != std::string::npos) {
        return PLATE_YELLOW_DOUBLE;
    }
    if (label.find("green") != std::string::npos ||
        label.find("新能源") != std::string::npos) {
        return PLATE_GREEN;
    }
    if (label.find("yellow") != std::string::npos ||
        label.find("黄") != std::string::npos) {
        return PLATE_YELLOW_SINGLE;
    }
    if (label.find("blue") != std::string::npos ||
        label.find("蓝") != std::string::npos) {
        return PLATE_BLUE;
    }

    // 标签名不可用时，回退到当前四类模型的固定顺序。
    switch (classId) {
    case 1: return PLATE_YELLOW_SINGLE;
    case 2: return PLATE_GREEN;
    case 3: return PLATE_YELLOW_DOUBLE;
    case 0:
    default:
        return PLATE_BLUE;
    }
}

static bool hasSupportedPlateLabels(const ObbDetector &detector)
{
    if (detector.numClasses() != 4) return false;

    bool hasBlue = false;
    bool hasYellow = false;
    bool hasGreen = false;
    bool hasDoubleYellow = false;
    for (int i = 0; i < detector.numClasses(); ++i) {
        const std::string label = lowerAscii(detector.className(i));
        if (label.find("double") != std::string::npos ||
            label.find("双层") != std::string::npos) {
            hasDoubleYellow = true;
        } else if (label.find("green") != std::string::npos ||
                   label.find("新能源") != std::string::npos) {
            hasGreen = true;
        } else if (label.find("yellow") != std::string::npos ||
                   label.find("黄") != std::string::npos) {
            hasYellow = true;
        } else if (label.find("blue") != std::string::npos ||
                   label.find("蓝") != std::string::npos) {
            hasBlue = true;
        }
    }
    return hasBlue && hasYellow && hasGreen && hasDoubleYellow;
}

static std::string vehicleLabelForPlate(const std::string &plateName,
                                        PlateType plateType)
{
    // 特殊用途字符比 YOLO 的颜色/层数更具体，必须优先判断。
    if (plateName.find("领") != std::string::npos)
        return "（领事馆车辆）";
    if (plateName.find("港") != std::string::npos ||
        plateName.find("澳") != std::string::npos)
        return "（港澳车辆）";
    if (plateName.find("学") != std::string::npos)
        return "（教练车）";
    if (plateName.find("挂") != std::string::npos)
        return "（挂车）";
    if (plateType == PLATE_GREEN || plateType == PLATE_GREEN_LARGE)
        return "（新能源车辆）";
    if (!plateName.empty() && plateType == PLATE_YELLOW_DOUBLE)
        return "（货车或摩托车）";
    return std::string();
}

static lprnet_context_t *selectLprContext(PlateType plateType,
                                          lprnet_context_t *blueCtx,
                                          lprnet_context_t *otherCtx)
{
    if (plateType == PLATE_BLUE && blueCtx && blueCtx->rknn_ctx != 0) {
        return blueCtx;
    }
    return otherCtx;
}

static void normalizePlateObb(const ObbBox &box, float &normW, float &normH, float &normAngle)
{
    normW = box.w;
    normH = box.h;
    normAngle = box.angle;

    // 某些 OBB 会把长边写进 h，导致车牌被按竖版拉正。
    // 这里统一把长边归一到 w，同时把角度补偿 90 度。
    if (normW < normH) {
        std::swap(normW, normH);
        normAngle -= static_cast<float>(CV_PI * 0.5);
    }
}

static bool rectifyPlateFromObb(const cv::Mat &bgrFrame, const ObbBox &box, cv::Mat &plateOut)
{
    if (bgrFrame.empty()) {
        return false;
    }

    float rectW = 0.0f;
    float rectH = 0.0f;
    float rectAngle = 0.0f;
    normalizePlateObb(box, rectW, rectH, rectAngle);
    // DEBUG: disable aspect-ratio reject for sample collection

    const float halfW = rectW * 0.5f;
    const float halfH = rectH * 0.5f;
    const float cosA = std::cos(rectAngle);
    const float sinA = std::sin(rectAngle);

    cv::Point2f srcVertices[4];
    // 必须和参考版/后处理保持同一套点序: 左上、左下、右下、右上。
    // 否则某些角度下 warpPerspective 会把车牌左右翻转，表现为“有框但没字”。
    srcVertices[0] = cv::Point2f(box.cx + (-halfW) * cosA - (-halfH) * sinA,
                                 box.cy + (-halfW) * sinA + (-halfH) * cosA);
    srcVertices[1] = cv::Point2f(box.cx + (-halfW) * cosA - ( halfH) * sinA,
                                 box.cy + (-halfW) * sinA + ( halfH) * cosA);
    srcVertices[2] = cv::Point2f(box.cx + ( halfW) * cosA - ( halfH) * sinA,
                                 box.cy + ( halfW) * sinA + ( halfH) * cosA);
    srcVertices[3] = cv::Point2f(box.cx + ( halfW) * cosA - (-halfH) * sinA,
                                 box.cy + ( halfW) * sinA + (-halfH) * cosA);

    const int dstW = std::max(1, static_cast<int>(std::round(rectW)));
    const int dstH = std::max(1, static_cast<int>(std::round(rectH)));
    cv::Point2f dstVertices[4] = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, static_cast<float>(dstH - 1)),
        cv::Point2f(static_cast<float>(dstW - 1), static_cast<float>(dstH - 1)),
        cv::Point2f(static_cast<float>(dstW - 1), 0.0f)
    };

    const cv::Mat warpMat = cv::getPerspectiveTransform(srcVertices, dstVertices);
    cv::warpPerspective(bgrFrame, plateOut, warpMat, cv::Size(dstW, dstH),
                        cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    return !plateOut.empty();
}

static float rotatedIntersectionOverMinArea(const ObbBox &a, const ObbBox &b)
{
    const float radToDeg = 180.0f / static_cast<float>(CV_PI);
    const cv::RotatedRect rectA(cv::Point2f(a.cx, a.cy),
                                cv::Size2f(std::max(a.w, 1.0f), std::max(a.h, 1.0f)),
                                a.angle * radToDeg);
    const cv::RotatedRect rectB(cv::Point2f(b.cx, b.cy),
                                cv::Size2f(std::max(b.w, 1.0f), std::max(b.h, 1.0f)),
                                b.angle * radToDeg);

    std::vector<cv::Point2f> intersection;
    const int intersectionType = cv::rotatedRectangleIntersection(rectA, rectB, intersection);
    if (intersectionType == cv::INTERSECT_NONE || intersection.empty()) return 0.0f;

    std::vector<cv::Point2f> hull;
    cv::convexHull(intersection, hull);
    const float intersectionArea = static_cast<float>(std::fabs(cv::contourArea(hull)));
    const float minArea = std::max(1.0f, std::min(a.w * a.h, b.w * b.h));
    return intersectionArea / minArea;
}

static bool areDuplicatePlateBoxes(const ObbBox &a, const ObbBox &b)
{
    if (!isPlateClassId(a.classId) || !isPlateClassId(b.classId)) return false;
    const bool sameClass = (a.classId == b.classId);

    // 同一车牌的多类别预测中心应非常接近。使用较小框的短边作为尺度，
    // 避免原先按长边判断时把左右相邻的两块车牌误当成重复框。
    const float shortSideA = std::min(a.w, a.h);
    const float shortSideB = std::min(b.w, b.h);
    // 同类别使用更严格条件，只清理提高原始 NMS 阈值后留下的近乎完全重复框。
    const float centerScale = sameClass ? 0.25f : 0.75f;
    const float centerTolerance = std::max(4.0f,
        std::min(shortSideA, shortSideB) * centerScale);
    const float centerDistance = std::hypot(a.cx - b.cx, a.cy - b.cy);
    if (centerDistance > centerTolerance) return false;

    // 使用交集/较小框面积，允许单层与双层类别给出大小不同但指向同一块车牌的框。
    const float overlapThreshold = sameClass ? 0.92f : 0.80f;
    return rotatedIntersectionOverMinArea(a, b) >= overlapThreshold;
}

static int plateBoxRecognitionRank(const ObbBox &box)
{
    if (box.plateRecognizedThisFrame && !box.plateName.empty()) return 2;
    if (!box.plateName.empty()) return 1;
    return 0;
}

} // namespace

DetectionWorker::DetectionWorker(QObject *parent)
    : QObject(parent)
    , m_frameBuffer(nullptr)
    , m_wifiControl(nullptr)
    , m_frameBuf565(nullptr)
    , m_rgb888Buf(nullptr)
    , m_lprnetLoaded(false)
    , m_preprocessEnabled(false)
    , m_preprocessMask(PlatePreprocessNone)
    , m_sciLoaded(false)
    , m_ffdnetLoaded(false)
    , m_mimoLoaded(false)
    , m_sciPreprocessor(nullptr)
    , m_ffdnetPreprocessor(nullptr)
    , m_mimoPreprocessor(nullptr)
    , m_lastPlatePairSeq(-1)
    , m_running(false)
    , m_lastProcessedSeq(-1)
    , m_nextTrackId(1)
    , m_nextPublicTrackId(1)
    , m_stableFrameSeq(-1)
    , m_hasDetection(false)
{
    memset(&m_lprnetBlueCtx, 0, sizeof(m_lprnetBlueCtx));
    memset(&m_lprnetOtherCtx, 0, sizeof(m_lprnetOtherCtx));
    memset(&m_lprnetDoubleYellowCtx, 0, sizeof(m_lprnetDoubleYellowCtx));
}

DetectionWorker::PlateTrackState *DetectionWorker::findTrack(int trackId)
{
    for (PlateTrackState &track : m_tracks) {
        if (track.id == trackId) return &track;
    }
    return nullptr;
}

bool DetectionWorker::updatePlateConsensus(
    PlateTrackState &track, const std::string &recognizedPlate)
{
    if (recognizedPlate.empty()) return false;

    track.recentPlateSamples.push_back(recognizedPlate);
    while (track.recentPlateSamples.size() > kPlateVoteWindow)
        track.recentPlateSamples.pop_front();

    std::map<std::string, int> frequencies;
    for (const std::string &sample : track.recentPlateSamples)
        ++frequencies[sample];

    const std::string previousDisplay = track.stablePlate.empty()
        ? track.candidatePlate
        : track.stablePlate;
    std::string dominantPlate;
    int dominantCount = 0;
    for (const auto &entry : frequencies) {
        if (entry.second > dominantCount ||
            (entry.second == dominantCount &&
             entry.first == previousDisplay)) {
            dominantPlate = entry.first;
            dominantCount = entry.second;
        }
    }

    track.candidatePlate = dominantPlate;
    track.candidateMatchCount = dominantCount;
    track.candidateMismatchCount =
        static_cast<int>(track.recentPlateSamples.size()) - dominantCount;

    // Wait for a complete ten-result window. A plate is confirmed or replaced
    // only when the exact full string wins at least seven votes.
    if (track.recentPlateSamples.size() < kPlateVoteWindow)
        return false;

    if (dominantPlate == track.stablePlate)
        return false;

    if (dominantCount < kPlateConfirmVotes)
        return false;

    track.stablePlate = dominantPlate;
    return true;
}

bool DetectionWorker::updatePlateIdentity(
    PlateTrackState &track, const std::string &recognizedPlate)
{
    if (recognizedPlate.empty()) return false;

    const bool replacedExistingIdentity = !track.identityPlate.empty();
    const int previousPublicId = track.publicId;
    if (replacedExistingIdentity &&
        plateCharacterDifference(track.identityPlate, recognizedPlate) <
            kPlateNewIdentityDifference) {
        // Zero, one or two changed characters are treated as OCR jitter and
        // remain attached to the current character identity.
        return false;
    }

    track.identityPlate = recognizedPlate;
    const auto assignCharacterId =
        [this, &track, replacedExistingIdentity, previousPublicId]() {
        const auto existingId = m_publicPlateIds.find(track.identityPlate);
        if (existingId != m_publicPlateIds.end() &&
            (!replacedExistingIdentity ||
             existingId->second != previousPublicId)) {
            track.publicId = existingId->second;
            return;
        }

        // Geometry tracks may split and reappear. Reuse a public ID only when
        // the recognized characters differ by at most two positions.
        int bestId = -1;
        int bestDifference = kPlateNewIdentityDifference;
        for (const auto &entry : m_publicPlateIds) {
            if (replacedExistingIdentity &&
                entry.second == previousPublicId) {
                continue;
            }
            const int difference =
                plateCharacterDifference(entry.first, track.identityPlate);
            if (difference < bestDifference) {
                bestDifference = difference;
                bestId = entry.second;
            }
        }
        if (bestId >= 0) {
            track.publicId = bestId;
            m_publicPlateIds.emplace(track.identityPlate, bestId);
            return;
        }

        track.publicId = m_nextPublicTrackId++;
        m_publicPlateIds.emplace(track.identityPlate, track.publicId);
    };
    assignCharacterId();
    return replacedExistingIdentity;
}

void DetectionWorker::assignTrackIds(DetectResult &result, int frameSeq)
{
    m_tracks.erase(std::remove_if(m_tracks.begin(), m_tracks.end(),
                                  [frameSeq](const PlateTrackState &track) {
                                      return frameSeq - track.lastSeenFrameSeq > kTrackMaxAgeFrames;
                                  }),
                   m_tracks.end());

    std::vector<bool> trackUsed(m_tracks.size(), false);
    for (ObbBox &box : result.boxes) {
        if (!isPlateClassId(box.classId)) continue;
        int bestIndex = -1;
        float bestCost = 1e9f;

        for (size_t i = 0; i < m_tracks.size(); ++i) {
            if (trackUsed[i]) continue;
            const PlateTrackState &track = m_tracks[i];
            // 不同车牌类别使用不同轨迹，避免 yellow/doubleline_yellow 互相继承字符缓存。
            if (box.classId != track.classId) continue;
            const int frameDelta = std::max(1, frameSeq - track.lastSeenFrameSeq);
            const float predictedX = track.cx + track.vx * frameDelta;
            const float predictedY = track.cy + track.vy * frameDelta;
            const float distance = std::hypot(box.cx - predictedX, box.cy - predictedY);
            const float referenceSize = std::max(40.0f,
                std::max(std::max(box.w, box.h), std::max(track.w, track.h)));
            const float iou = axisAlignedIou(predictedX, predictedY, track.w, track.h,
                                             box.cx, box.cy, box.w, box.h);

            // 高速车辆允许较大的中心位移；没有交叠时仍限制在约2.5个框尺度内。
            if (iou < 0.02f && distance > referenceSize * 2.5f) continue;

            const float sizeRatio = std::min(box.w * box.h, track.w * track.h) /
                                    std::max(1.0f, std::max(box.w * box.h, track.w * track.h));
            if (sizeRatio < 0.25f) continue;

            const float cost = distance / referenceSize + (1.0f - iou) * 0.55f +
                               (1.0f - sizeRatio) * 0.25f;
            if (cost < bestCost) {
                bestCost = cost;
                bestIndex = static_cast<int>(i);
            }
        }

        if (bestIndex >= 0) {
            PlateTrackState &track = m_tracks[static_cast<size_t>(bestIndex)];
            const int frameDelta = std::max(1, frameSeq - track.lastSeenFrameSeq);
            const float measuredVx = (box.cx - track.cx) / frameDelta;
            const float measuredVy = (box.cy - track.cy) / frameDelta;
            track.vx = track.vx * 0.6f + measuredVx * 0.4f;
            track.vy = track.vy * 0.6f + measuredVy * 0.4f;
            track.cx = box.cx;
            track.cy = box.cy;
            track.w = box.w;
            track.h = box.h;
            track.classId = box.classId;
            track.lastSeenFrameSeq = frameSeq;
            box.trackId = track.id;
            trackUsed[static_cast<size_t>(bestIndex)] = true;
        } else {
            PlateTrackState track;
            track.id = m_nextTrackId++;
            track.classId = box.classId;
            track.lastSeenFrameSeq = frameSeq;
            track.lastLprFrameSeq = -1;
            track.cx = box.cx;
            track.cy = box.cy;
            track.w = box.w;
            track.h = box.h;
            track.vx = 0.0f;
            track.vy = 0.0f;
            box.trackId = track.id;
            m_tracks.push_back(std::move(track));
            trackUsed.push_back(true);
        }
    }
}

void DetectionWorker::suppressDuplicatePlateBoxes(DetectResult &result)
{
    if (result.boxes.size() < 2) return;

    std::vector<size_t> order(result.boxes.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(),
                     [&result](size_t lhs, size_t rhs) {
                         const ObbBox &a = result.boxes[lhs];
                         const ObbBox &b = result.boxes[rhs];
                         const int rankA = plateBoxRecognitionRank(a);
                         const int rankB = plateBoxRecognitionRank(b);
                         if (rankA != rankB) return rankA > rankB;
                         return a.confidence > b.confidence;
                     });

    std::vector<bool> suppressed(result.boxes.size(), false);
    std::vector<int> suppressedTrackIds;
    for (size_t oi = 0; oi < order.size(); ++oi) {
        const size_t keepIndex = order[oi];
        if (suppressed[keepIndex]) continue;

        for (size_t oj = oi + 1; oj < order.size(); ++oj) {
            const size_t dropIndex = order[oj];
            if (suppressed[dropIndex] ||
                !areDuplicatePlateBoxes(result.boxes[keepIndex], result.boxes[dropIndex])) {
                continue;
            }

            const ObbBox &kept = result.boxes[keepIndex];
            const ObbBox &dropped = result.boxes[dropIndex];
            suppressed[dropIndex] = true;
            if (dropped.trackId >= 0) suppressedTrackIds.push_back(dropped.trackId);
            qDebug().nospace()
                << "[PlateDedup] keep cls=" << kept.classId
                << " track=" << kept.trackId
                << " current_lpr=" << kept.plateRecognizedThisFrame
                << " plate=" << QString::fromStdString(kept.plateName)
                << "; drop cls=" << dropped.classId
                << " track=" << dropped.trackId
                << " current_lpr=" << dropped.plateRecognizedThisFrame
                << " plate=" << QString::fromStdString(dropped.plateName);
        }
    }

    std::vector<ObbBox> keptBoxes;
    keptBoxes.reserve(result.boxes.size());
    for (size_t i = 0; i < result.boxes.size(); ++i) {
        if (!suppressed[i]) keptBoxes.push_back(std::move(result.boxes[i]));
    }
    result.boxes = std::move(keptBoxes);

    // 保留轨迹几何连续性，但清掉被压制框的字符缓存，防止下一帧重新污染保留框。
    for (PlateTrackState &track : m_tracks) {
        if (std::find(suppressedTrackIds.begin(), suppressedTrackIds.end(), track.id) !=
            suppressedTrackIds.end()) {
            track.stablePlate.clear();
            track.lastLprFrameSeq = -1;
            track.candidatePlate.clear();
            track.candidateMatchCount = 0;
            track.candidateMismatchCount = 0;
            track.recentPlateSamples.clear();
            track.publicId = -1;
            track.identityPlate.clear();
            track.lastSentPlate.clear();
        }
    }
}

DetectionWorker::~DetectionWorker()
{
    stopInference();

    delete[] m_frameBuf565;
    delete[] m_rgb888Buf;
}

bool DetectionWorker::loadModel(const char *modelPath, const char *labelPath,
                                 const ObbModelParams &params)
{
    bool ok = m_detector.loadModel(modelPath, labelPath, params);
    if (ok && !hasSupportedPlateLabels(m_detector)) {
        qWarning().nospace()
            << "[DetectionWorker] plate label list mismatch: expected exactly "
               "blue/yellow/green/doubleline_yellow, actual classes="
            << m_detector.numClasses();
        m_detector.release();
        ok = false;
    }
    if (ok && modelPath) {
        m_modelPath = modelPath;
        qDebug().nospace()
            << "[DetectionWorker] YOLO loaded: " << modelPath
            << " input=" << m_detector.modelWidth() << "x" << m_detector.modelHeight();
    } else {
        m_modelPath.clear();
    }
    return ok;
}

bool DetectionWorker::loadLprModels(const char *blueModelPath,
                                    const char *otherModelPath,
                                    const char *doubleYellowModelPath)
{
    if (m_lprnetLoaded) return true;

    if (init_lprnet_model(blueModelPath, &m_lprnetBlueCtx) != 0) {
        return false;
    }
    if (init_lprnet_model(otherModelPath, &m_lprnetOtherCtx) != 0) {
        release_lprnet_model(&m_lprnetBlueCtx);
        memset(&m_lprnetBlueCtx, 0, sizeof(m_lprnetBlueCtx));
        return false;
    }

    if (doubleYellowModelPath && doubleYellowModelPath[0] != '\0') {
        if (init_lprnet_model(doubleYellowModelPath, &m_lprnetDoubleYellowCtx) != 0) {
            memset(&m_lprnetDoubleYellowCtx, 0, sizeof(m_lprnetDoubleYellowCtx));
            qWarning().nospace()
                << "[LPRNet] double-yellow model load failed: " << doubleYellowModelPath
                << "; cropped plate will use the normal yellow model";
        }
    }

    m_lprnetLoaded = true;
    return true;
}

// ---- 加载 SCI 车牌增强模型 ----
bool DetectionWorker::loadSciModel(const char *modelPath)
{
    if (m_sciLoaded) return true;
    if (!modelPath || modelPath[0] == '\0') return false;

    m_sciPreprocessor = new PlatePreprocessorRknn(PlatePreprocessKind::Sci);
    std::string error;
    if (!m_sciPreprocessor->init(std::string(modelPath), &error)) {
        qWarning() << "[DetectionWorker] SCI model init failed:" << error.c_str();
        delete m_sciPreprocessor;
        m_sciPreprocessor = nullptr;
        return false;
    }
    m_sciLoaded = true;
    qDebug() << "[DetectionWorker] SCI plate enhancement model loaded";
    return true;
}

// ---- 加载 FFDNet 车牌增强模型 ----
bool DetectionWorker::loadFfdnetModel(const char *modelPath)
{
    if (m_ffdnetLoaded) return true;
    if (!modelPath || modelPath[0] == '\0') return false;

    m_ffdnetPreprocessor = new PlatePreprocessorRknn(PlatePreprocessKind::Sci);
    std::string error;
    if (!m_ffdnetPreprocessor->init(std::string(modelPath), &error)) {
        qWarning() << "[DetectionWorker] FFDNet model init failed:" << error.c_str();
        delete m_ffdnetPreprocessor;
        m_ffdnetPreprocessor = nullptr;
        return false;
    }
    m_ffdnetLoaded = true;
    qDebug() << "[DetectionWorker] FFDNet denoising model loaded";
    return true;
}

bool DetectionWorker::loadMimoModel(const char *modelPath)
{
    if (m_mimoLoaded) return true;
    if (!modelPath || modelPath[0] == '\0') return false;

    m_mimoPreprocessor = new PlatePreprocessorRknn(PlatePreprocessKind::MimoPlus);
    std::string error;
    if (!m_mimoPreprocessor->init(std::string(modelPath), &error)) {
        qWarning() << "[DetectionWorker] MIMO+ teacher init failed:" << error.c_str();
        delete m_mimoPreprocessor;
        m_mimoPreprocessor = nullptr;
        return false;
    }
    m_mimoLoaded = true;
    qDebug() << "[DetectionWorker] MIMO+ display teacher loaded";
    return true;
}

// ---- 启动推理循环 (在工作线程中执行) ----
void DetectionWorker::startInference()
{
    if (!m_detector.isLoaded()) return;
    if (m_running) return;

    // 预分配所有缓冲区 (消除运行时 malloc)
    m_frameBuf565  = new uint8_t[FRAME_SIZE_RGB565];              // RGB565 源帧
    m_rgb888Buf   = new uint8_t[FRAME_WIDTH * FRAME_HEIGHT * 3]; // RGB888 推理输入

    m_lastProcessedSeq = -1;
    m_tracks.clear();
    m_publicPlateIds.clear();
    m_nextTrackId = 1;
    m_nextPublicTrackId = 1;
    m_hasDetection = false;
    m_running = true;

    emit statusMessage(QString("AI 推理已启动 (%1×%2)")
                       .arg(m_detector.modelWidth())
                       .arg(m_detector.modelHeight()));

    // 立即开始第一轮推理
    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}

void DetectionWorker::resetSceneState()
{
    m_tracks.clear();
    m_publicPlateIds.clear();
    m_nextTrackId = 1;
    m_nextPublicTrackId = 1;
    m_lastProcessedSeq = -1;
    {
        QMutexLocker lock(&m_resultMutex);
        m_stableResult = DetectResult();
        m_stableFrameSeq = -1;
        m_hasDetection = false;
    }
    {
        QMutexLocker lock(&m_platePairMutex);
        m_lastOriginalPlate = QImage();
        m_lastSciPlate = QImage();
        m_lastFfdnetPlate = QImage();
        m_lastMimoPlate = QImage();
        m_lastPlatePairSeq = -1;
        m_lastEnhancedPlate.clear();
        m_lastEnhancedSeq = -1;
    }
    m_preprocessThrottleTimer.invalidate();
}

void DetectionWorker::setEnhancedLprEnabled(bool enabled)
{
    m_enhancedLprEnabled.store(enabled, std::memory_order_relaxed);
    qDebug() << "[DetectionWorker] enhanced-cascade LPR"
             << (enabled ? "enabled" : "disabled");
}

void DetectionWorker::configurePreprocess(int enabledMask,
                                          const QString &sciModelPath,
                                          const QString &ffdnetModelPath,
                                          const QString &mimoModelPath)
{
    // This slot runs in the inference thread.  Never destroy an RKNN context
    // from the UI thread while infer() may still be using it.
    enabledMask &= PlatePreprocessAll;
    if (enabledMask == PlatePreprocessNone) {
        m_preprocessEnabled = false;
        m_preprocessMask = PlatePreprocessNone;
        m_preprocessThrottleTimer.invalidate();
        {
            QMutexLocker lock(&m_platePairMutex);
            m_lastOriginalPlate = QImage();
            m_lastSciPlate = QImage();
            m_lastFfdnetPlate = QImage();
            m_lastMimoPlate = QImage();
            m_lastPlatePairSeq = -1;
        }
        // Retain initialized contexts for a fast, race-free re-enable.  They
        // are released by stopInference() in this same worker thread.
        emit preprocessStateChanged(
            PlatePreprocessNone, QStringLiteral("Plate preprocessing disabled"));
        return;
    }

    const QByteArray sciPath = sciModelPath.toLocal8Bit();
    const QByteArray ffdPath = ffdnetModelPath.toLocal8Bit();
    const QByteArray mimoPath = mimoModelPath.toLocal8Bit();
    const bool sciOk = !(enabledMask & PlatePreprocessSci) ||
                       loadSciModel(sciPath.constData());
    const bool ffdOk = !(enabledMask & PlatePreprocessFfdnet) ||
                       loadFfdnetModel(ffdPath.constData());
    const bool mimoOk = !(enabledMask & PlatePreprocessDeblur) ||
                        loadMimoModel(mimoPath.constData());
    int activeMask = enabledMask;
    if (!sciOk) activeMask &= ~PlatePreprocessSci;
    if (!ffdOk) activeMask &= ~PlatePreprocessFfdnet;
    if (!mimoOk) activeMask &= ~PlatePreprocessDeblur;
    m_preprocessMask = activeMask;
    m_preprocessEnabled = activeMask != PlatePreprocessNone;
    m_preprocessThrottleTimer.invalidate();
    {
        QMutexLocker lock(&m_platePairMutex);
        m_lastOriginalPlate = QImage();
        m_lastSciPlate = QImage();
        m_lastFfdnetPlate = QImage();
        m_lastMimoPlate = QImage();
        m_lastPlatePairSeq = -1;
    }

    const QString message = QStringLiteral("Cascade: [1]SCI=%1 [2]FFDNet=%2 [3]Deblur=%3")
        .arg(activeMask & PlatePreprocessSci ? QStringLiteral("ON") : QStringLiteral("OFF"))
        .arg(activeMask & PlatePreprocessFfdnet ? QStringLiteral("ON") : QStringLiteral("OFF"))
        .arg(activeMask & PlatePreprocessDeblur ? QStringLiteral("ON") : QStringLiteral("OFF"));
    emit preprocessStateChanged(activeMask, message);
    emit statusMessage(message);
}

void DetectionWorker::stopInference()
{
    m_running = false;
    m_tracks.clear();
    m_publicPlateIds.clear();
    m_nextTrackId = 1;
    m_nextPublicTrackId = 1;

    // 释放 LPRNet 模型
    if (m_lprnetLoaded) {
        release_lprnet_model(&m_lprnetBlueCtx);
        release_lprnet_model(&m_lprnetOtherCtx);
        if (m_lprnetDoubleYellowCtx.rknn_ctx != 0) {
            release_lprnet_model(&m_lprnetDoubleYellowCtx);
        }
        memset(&m_lprnetBlueCtx, 0, sizeof(m_lprnetBlueCtx));
        memset(&m_lprnetOtherCtx, 0, sizeof(m_lprnetOtherCtx));
        memset(&m_lprnetDoubleYellowCtx, 0, sizeof(m_lprnetDoubleYellowCtx));
        m_lprnetLoaded = false;
    }

    const bool hadPreprocess = m_preprocessEnabled || m_sciLoaded ||
                               m_ffdnetLoaded || m_mimoLoaded;
    releasePreprocessModels();
    if (hadPreprocess) {
        emit preprocessStateChanged(
            PlatePreprocessNone, QStringLiteral("Plate preprocessing stopped with AI"));
    }

    // 释放 YOLO 模型
    m_detector.release();
    m_modelPath.clear();

    delete[] m_frameBuf565;   m_frameBuf565 = nullptr;
    delete[] m_rgb888Buf;    m_rgb888Buf = nullptr;

    {
        QMutexLocker lock(&m_resultMutex);
        m_stableResult = DetectResult();
        m_hasDetection = false;
    }

    emit statusMessage("AI 推理已停止");
}

// ---- 释放 SCI + FFDNet (destructor / manual) ----
void DetectionWorker::releasePreprocessModels()
{
    m_preprocessEnabled = false;
    m_preprocessMask = PlatePreprocessNone;
    if (m_sciPreprocessor) {
        m_sciPreprocessor->release();
        delete m_sciPreprocessor;
        m_sciPreprocessor = nullptr;
        m_sciLoaded = false;
    }
    if (m_ffdnetPreprocessor) {
        m_ffdnetPreprocessor->release();
        delete m_ffdnetPreprocessor;
        m_ffdnetPreprocessor = nullptr;
        m_ffdnetLoaded = false;
    }
    if (m_mimoPreprocessor) {
        m_mimoPreprocessor->release();
        delete m_mimoPreprocessor;
        m_mimoPreprocessor = nullptr;
        m_mimoLoaded = false;
    }
    {
        QMutexLocker lock(&m_platePairMutex);
        m_lastOriginalPlate = QImage();
        m_lastSciPlate = QImage();
        m_lastFfdnetPlate = QImage();
        m_lastMimoPlate = QImage();
        m_lastPlatePairSeq = -1;
    }
    m_preprocessThrottleTimer.invalidate();
}

// ---- 推理主循环 (自驱动) ----
void DetectionWorker::processNextFrame()
{
    if (!m_running || !m_detector.isLoaded() || !m_frameBuffer) return;

    // ---- 1. 从 FrameBuffer 非消耗性读取最新帧 ----
    int frameSeq = -1;
    bool hasFrame = m_frameBuffer->copyLatestFrame(
        m_frameBuf565, frameSeq, m_lastProcessedSeq);
    if (!hasFrame) {
        // 无新帧, 2ms 后重试
        QTimer::singleShot(2, this, [this]() { processNextFrame(); });
        return;
    }
    m_lastProcessedSeq = frameSeq;
    // ---- 2. RGB565 → RGB888 (LUT 加速转换) ----
    ImageConverter::rgb565ToRGB888Fast(m_frameBuf565, m_rgb888Buf,
                                       FRAME_WIDTH, FRAME_HEIGHT);
    // ---- 3. YOLO OBB 检测 ----
    DetectResult result;
    m_detector.detect(m_rgb888Buf, FRAME_WIDTH, FRAME_HEIGHT, result);
    assignTrackIds(result, frameSeq);
    std::map<int, int> publicTrackIds;
    for (const ObbBox &box : result.boxes) {
        if (box.trackId >= 0)
            publicTrackIds[box.trackId] = -1;
    }

    // ---- 4. LPRNet 车牌识别 (逻辑与原版一致) ----
    if (m_lprnetLoaded && !result.boxes.empty()) {
        cv::Mat rgbFrame(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3, m_rgb888Buf);
        cv::Mat bgrFrame;
        cv::cvtColor(rgbFrame, bgrFrame, cv::COLOR_RGB2BGR);

        int preprocessTrackId = -1;
        if (m_preprocessEnabled &&
            (!m_preprocessThrottleTimer.isValid() ||
             m_preprocessThrottleTimer.elapsed() >= kPreprocessPreviewIntervalMs)) {
            float largestArea = 0.0f;
            for (const ObbBox &candidate : result.boxes) {
                if (!isPlateClassId(candidate.classId)) continue;
                const float area = candidate.w * candidate.h;
                if (area > largestArea) {
                    largestArea = area;
                    preprocessTrackId = candidate.trackId;
                }
            }
        }

        bool preprocessAttempted = false;
        for (ObbBox &box : result.boxes) {
            if (!isPlateClassId(box.classId)) continue;

            PlateTrackState *track = findTrack(box.trackId);
            const PlateType plateType = yoloClassToPlateType(
                box.classId, m_detector.className(box.classId));
            float normW = 0.0f;
            float normH = 0.0f;
            float normAngle = 0.0f;
            normalizePlateObb(box, normW, normH, normAngle);
            Q_UNUSED(normAngle);
            const float aspectRatio = normW / std::max(normH, 1.0f);

            // 只有 YOLO 双层黄牌类进入裁剪和专用模型，不再用宽高比猜测。
            // 每个候选都执行本帧 LPR，字符身份用于生成对外 ID。
            cv::Mat plateImg;
            if (!rectifyPlateFromObb(bgrFrame, box, plateImg)) {
                qDebug().nospace()
                    << "[PlateLpr] rectify failed cls=" << box.classId
                    << " angle=" << box.angle
                    << " wh=" << box.w << "x" << box.h;
                continue;
            }

            // Enabled stages form one fixed cascade:
            // raw -> SCI -> FFDNet -> Deblur.  Only the largest plate is
            // processed and at most once per preview interval.  This work is
            // in the AI worker thread, so the 60 Hz display path never waits.
            cv::Mat cascadeInput = plateImg;
            cv::Mat sciOut, ffdnetOut, deblurOut;
            bool anyStageOk = false;
            const bool runPreprocess = m_preprocessEnabled && !preprocessAttempted &&
                                       box.trackId == preprocessTrackId;
            if (runPreprocess) {
                preprocessAttempted = true;
                m_preprocessThrottleTimer.restart();
            }
            if (runPreprocess && (m_preprocessMask & PlatePreprocessSci) &&
                m_sciLoaded && m_sciPreprocessor &&
                m_sciPreprocessor->is_initialized()) {
                if (m_sciPreprocessor->infer(cascadeInput, &sciOut, nullptr, nullptr)) {
                    cascadeInput = sciOut;
                    anyStageOk = true;
                }
            }
            if (runPreprocess && (m_preprocessMask & PlatePreprocessFfdnet) &&
                m_ffdnetLoaded && m_ffdnetPreprocessor &&
                m_ffdnetPreprocessor->is_initialized()) {
                if (m_ffdnetPreprocessor->infer(cascadeInput, &ffdnetOut, nullptr, nullptr)) {
                    cascadeInput = ffdnetOut;
                    anyStageOk = true;
                }
            }
            if (runPreprocess && (m_preprocessMask & PlatePreprocessDeblur) &&
                m_mimoLoaded && m_mimoPreprocessor &&
                m_mimoPreprocessor->is_initialized()) {
                if (m_mimoPreprocessor->infer(cascadeInput, &deblurOut, nullptr, nullptr)) {
                    cascadeInput = deblurOut;
                    anyStageOk = true;
                }
            }
            if (anyStageOk) {
                QMutexLocker lock(&m_platePairMutex);
                m_lastOriginalPlate = QImage(plateImg.data,
                    plateImg.cols, plateImg.rows, plateImg.step,
                    QImage::Format_BGR888).copy();
                m_lastSciPlate = QImage();
                m_lastFfdnetPlate = QImage();
                m_lastMimoPlate = QImage(cascadeInput.data,
                    cascadeInput.cols, cascadeInput.rows, cascadeInput.step,
                    QImage::Format_BGR888).copy();
                m_lastPlatePairSeq = frameSeq;
            }

            lprnet_result lprResult;
            lprnet_context_t *doubleYellowCtx =
                (m_lprnetDoubleYellowCtx.rknn_ctx != 0) ? &m_lprnetDoubleYellowCtx : nullptr;
            lprnet_context_t *lprCtx =
                selectLprContext(plateType, &m_lprnetBlueCtx, &m_lprnetOtherCtx);

            if (lprCtx) {
                if (track) track->lastLprFrameSeq = frameSeq;
                const int lprRet = inference_lprnet_model(lprCtx, plateImg, &lprResult,
                                                          plateType, doubleYellowCtx,
                                                          aspectRatio, nullptr,
                                                          frameSeq, box.trackId);
                if (lprRet == 0 && !lprResult.plate_name.empty()) {
                    box.plateRecognizedThisFrame = true;
                    if (track) {
                        const bool identityReplaced =
                            updatePlateIdentity(*track, lprResult.plate_name);
                        if (identityReplaced) {
                            // Three or more changed characters create a new
                            // identity immediately. Never carry old text
                            // consensus into that public ID.
                            track->stablePlate.clear();
                            track->candidatePlate.clear();
                            track->candidateMatchCount = 0;
                            track->candidateMismatchCount = 0;
                            track->recentPlateSamples.clear();
                            track->lastSentPlate.clear();
                        }
                        updatePlateConsensus(*track, lprResult.plate_name);
                        publicTrackIds[track->id] = track->publicId;
                        box.plateName = track->stablePlate.empty()
                            ? track->candidatePlate
                            : track->stablePlate;

                        // Once 7/10 voting confirms the text, use it as the
                        // identity reference while retaining the same ID.
                        if (track->publicId >= 0 &&
                            !track->stablePlate.empty() &&
                            plateCharacterDifference(track->identityPlate,
                                                     track->stablePlate) <
                                kPlateNewIdentityDifference) {
                            track->identityPlate = track->stablePlate;
                            m_publicPlateIds[track->identityPlate] =
                                track->publicId;
                        }

                        qDebug().nospace()
                            << "[PlateConfirm] internal_track=" << track->id
                            << " public_id=" << publicTrackIds[track->id]
                            << " candidate=" << QString::fromStdString(track->candidatePlate)
                            << " current=" << QString::fromStdString(lprResult.plate_name)
                            << " confirmed=" << QString::fromStdString(track->stablePlate)
                            << " matches=" << track->candidateMatchCount
                            << " mismatches=" << track->candidateMismatchCount;

                        if (track->publicId >= 0 &&
                            !track->stablePlate.empty() &&
                            track->lastSentPlate != track->stablePlate &&
                            m_wifiControl &&
                            m_wifiControl->enqueuePlateImage(
                                plateImg, frameSeq, track->publicId,
                                QString::fromStdString(track->stablePlate))) {
                            track->lastSentPlate = track->stablePlate;
                            track->candidateMismatchCount = 0;
                        }
                    } else {
                        box.plateName = lprResult.plate_name;
                    }
                    box.vehicleLabel = vehicleLabelForPlate(box.plateName, plateType);
                } else {
                    if (track) {
                        box.plateName = track->stablePlate.empty()
                            ? track->candidatePlate
                            : track->stablePlate;
                        box.vehicleLabel = vehicleLabelForPlate(box.plateName, plateType);
                    }
                    qDebug().nospace()
                        << "[PlateLpr] empty result cls=" << box.classId
                        << " type=" << static_cast<int>(plateType)
                        << " aspect=" << aspectRatio
                        << " angle=" << box.angle
                        << " plate=" << plateImg.cols << "x" << plateImg.rows
                        << " ret=" << lprRet;
                }
            }
            // Optional second LPR pass on the enhanced cascade output. This
            // result stays display-only and is never merged into tracks.
            if (m_enhancedLprEnabled.load(std::memory_order_relaxed) &&
                runPreprocess && anyStageOk && lprCtx && !cascadeInput.empty()) {
                lprnet_result enhancedLprResult;
                const int enhancedRet = inference_lprnet_model(
                    lprCtx, cascadeInput, &enhancedLprResult, plateType,
                    doubleYellowCtx, aspectRatio, nullptr, frameSeq, box.trackId);
                if (enhancedRet == 0 && !enhancedLprResult.plate_name.empty()) {
                    QMutexLocker lock(&m_platePairMutex);
                    m_lastEnhancedPlate =
                        QString::fromStdString(enhancedLprResult.plate_name);
                    m_lastEnhancedSeq = frameSeq;
                }
            }
        }
    }

    // LPR 完成后再跨类别去重，优先保留真正能识别出字符的候选框。
suppressDuplicatePlateBoxes(result);
    for (ObbBox &box : result.boxes) {
        const auto publicIdIt = publicTrackIds.find(box.trackId);
        box.trackId = publicIdIt == publicTrackIds.end()
            ? -1
            : publicIdIt->second;
    }

    // ---- 5. 不生成标注帧；UI 直接绘制矢量叠加层 ----
    // The UI paints vectors over its current RGB565 frame. Avoid producing an
    // unused full-size ARGB frame in the inference thread.
    // ---- 6. 写入结果缓存 (QMutex 保护, UI 线程安全读取) ----
    {
        QMutexLocker lock(&m_resultMutex);
        m_stableResult = result;
        m_stableFrameSeq = frameSeq;
        m_hasDetection = true;
    }
    // Publish only after LPR processing, so the first valid plate text reaches
    // the UI immediately and no geometry-only result overwrites it.
    emit detectionReady(result, frameSeq);
    emit inferenceDone(frameSeq);

    // ---- 7. 立即调度下一轮 ----
    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}
