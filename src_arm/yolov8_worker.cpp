// Shared asynchronous worker for YOLOPv2, legacy YOLOv8 and Detect3D.
#include "yolov8_worker.h"
#include "image_converter.h"

#include <QTimer>
#include <QDebug>
#include <algorithm>
#include <cctype>
#include <utility>

namespace {

constexpr qint64 kCrosswalkGateMs = 1000;

std::string lowerClassName(const char *name)
{
    std::string value = name ? name : "";
    if (value.size() >= 3 &&
        static_cast<unsigned char>(value[0]) == 0xEF &&
        static_cast<unsigned char>(value[1]) == 0xBB &&
        static_cast<unsigned char>(value[2]) == 0xBF) {
        value.erase(0, 3);
    }
    while (!value.empty() &&
           std::isspace(static_cast<unsigned char>(value.front()))) {
        value.erase(value.begin());
    }
    while (!value.empty() &&
           std::isspace(static_cast<unsigned char>(value.back()))) {
        value.pop_back();
    }
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    value.erase(std::remove_if(value.begin(), value.end(),
                               [](unsigned char ch) {
                                   return ch == '_' || ch == '-' ||
                                          std::isspace(ch);
                               }),
                value.end());
    return value;
}

bool isPersonClass(const std::string &name)
{
    return name.find("person") != std::string::npos
        || name.find("pedestrian") != std::string::npos
        || name.find("people") != std::string::npos
        || name.find("rider") != std::string::npos
        || name.find("motor") != std::string::npos
        || name.find("行人") != std::string::npos;
}

bool isVehicleClass(const std::string &name)
{
    if (name.find("motor") != std::string::npos)
        return false;

    return name.find("car") != std::string::npos
        || name.find("bus") != std::string::npos
        || name.find("truck") != std::string::npos
        || name.find("vehicle") != std::string::npos
        || name.find("van") != std::string::npos
        || name.find("车辆") != std::string::npos;
}

bool isYieldSubjectClass(const std::string &name)
{
    return isPersonClass(name);
}

bool isCrosswalkClass(const std::string &name)
{
    return name.find("crosswalk") != std::string::npos
        || name.find("zebra") != std::string::npos
        || name.find("crossing") != std::string::npos
        || name.find("斑马") != std::string::npos;
}

bool boxesOverlap(const YoloBox &a, const YoloBox &b)
{
    return std::max(a.left, b.left) < std::min(a.right, b.right)
        && std::max(a.top, b.top) < std::min(a.bottom, b.bottom);
}

} // namespace

YoloV8Worker::YoloV8Worker(QObject *parent)
    : QObject(parent)
    , m_frameBuffer(nullptr)
    , m_frameBuf565(nullptr)
    , m_rgb888Buf(nullptr)
    , m_running(false)
    , m_yieldDetectionEnabled(false)
    , m_lastProcessedSeq(-1)
    , m_stableFrameSeq(-1)
    , m_hasResult(false)
{
}

YoloV8Worker::~YoloV8Worker()
{
    stopInference();
    delete[] m_frameBuf565;
    delete[] m_rgb888Buf;
}

bool YoloV8Worker::loadModel(const char *modelPath, const char *labelPath,
                              const YoloV8Params &params)
{
    m_params = params;
    m_laneDepartureAnalyzer.configure(params);
    return m_detector.loadModel(modelPath, labelPath, params);
}

void YoloV8Worker::startInference()
{
    if (!m_detector.isLoaded() || m_running) return;

    m_frameBuf565  = new uint8_t[FRAME_SIZE_RGB565];
    m_rgb888Buf   = new uint8_t[FRAME_WIDTH * FRAME_HEIGHT * 3];

    m_lastProcessedSeq = -1;
    m_hasResult = false;
    m_violationAlertTimer.invalidate();
    m_crosswalkGateTimer.invalidate();
    m_laneDepartureAnalyzer.reset();
    m_running = true;

    const char *backendName = m_detector.isDetect3d()
        ? "Detect3D" : (m_yieldDetectionEnabled ? "YOLOv8" : "YOLOPv2");
    emit statusMessage(QString("%1 inference started (%2x%3)")
                       .arg(backendName)
                       .arg(m_detector.modelWidth())
                       .arg(m_detector.modelHeight()));

    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}

void YoloV8Worker::resetSceneState()
{
    m_lastProcessedSeq = -1;
    m_violationAlertTimer.invalidate();
    m_crosswalkGateTimer.invalidate();
    m_laneDepartureAnalyzer.reset();
    {
        QMutexLocker lock(&m_resultMutex);
        m_stableResult = YoloDetectResult();
        m_stableFrameSeq = -1;
        m_hasResult = false;
    }
}

void YoloV8Worker::stopInference()
{
    const bool wasDetect3d = m_detector.isDetect3d();
    const bool wasYieldDetector = m_yieldDetectionEnabled;
    m_running = false;
    m_detector.release();
    delete[] m_frameBuf565;   m_frameBuf565 = nullptr;
    delete[] m_rgb888Buf;    m_rgb888Buf = nullptr;

    {
        QMutexLocker lock(&m_resultMutex);
        m_stableResult = YoloDetectResult();
        m_hasResult = false;
    }
    m_violationAlertTimer.invalidate();
    m_crosswalkGateTimer.invalidate();
    m_laneDepartureAnalyzer.reset();
    emit statusMessage(wasDetect3d ? "Detect3D inference stopped"
        : (wasYieldDetector ? "YOLOv8 inference stopped"
                            : "YOLOPv2 inference stopped"));
}

void YoloV8Worker::detectYieldViolation(YoloDetectResult &result)
{
    const auto nameOf = [this](const YoloBox &box) {
        return lowerClassName(m_detector.className(box.classId));
    };

    // Expand every pedestrian/rider/motor box to three times its original
    // width. The expanded box is used both for display and overlap decisions.
    for (YoloBox &roadUser : result.boxes) {
        if (!isYieldSubjectClass(nameOf(roadUser)))
            continue;
        const int originalWidth = std::max(1, roadUser.right - roadUser.left);
        const int centerX = (roadUser.left + roadUser.right) / 2;
        const int expandedWidth = originalWidth * 3;
        const int expandedLeft = centerX - expandedWidth / 2;
        roadUser.left = std::max(0, expandedLeft);
        roadUser.right = std::min(FRAME_WIDTH,
                                  expandedLeft + expandedWidth);
    }

    const bool crosswalkDetected = std::any_of(
        result.boxes.begin(), result.boxes.end(),
        [&nameOf](const YoloBox &box) {
            return isCrosswalkClass(nameOf(box));
        });
    if (crosswalkDetected)
        m_crosswalkGateTimer.restart();

    // Crosswalk detection opens the person/vehicle-overlap decision for one
    // second. The crosswalk box itself is never cached or added to later frames.
    const bool crosswalkGateOpen =
        m_crosswalkGateTimer.isValid() &&
        m_crosswalkGateTimer.elapsed() < kCrosswalkGateMs;
    if (!crosswalkGateOpen) {
        m_violationAlertTimer.invalidate();
        result.yieldViolationAlert = false;
        return;
    }

    // person/rider/motor 与车辆直接重叠时报警。
    for (const YoloBox &roadUser : result.boxes) {
        if (!isYieldSubjectClass(nameOf(roadUser)))
            continue;
        for (YoloBox &vehicle : result.boxes) {
            if (&roadUser != &vehicle
                && isVehicleClass(nameOf(vehicle))
                && boxesOverlap(roadUser, vehicle)) {
                vehicle.yieldViolation = true;
            }
        }
    }

    // 同一斑马线同时与 person/rider/motor 和车辆重叠时报警。
    for (const YoloBox &crosswalk : result.boxes) {
        if (!isCrosswalkClass(nameOf(crosswalk)))
            continue;

        for (const YoloBox &roadUser : result.boxes) {
            if (!isYieldSubjectClass(nameOf(roadUser))
                || !boxesOverlap(roadUser, crosswalk)) {
                continue;
            }
            for (YoloBox &vehicle : result.boxes) {
                if (&roadUser != &vehicle
                    && isVehicleClass(nameOf(vehicle))
                    && boxesOverlap(vehicle, crosswalk)) {
                    vehicle.yieldViolation = true;
                }
            }
        }
    }

    const bool violationDetected = std::any_of(
        result.boxes.begin(), result.boxes.end(),
        [](const YoloBox &box) { return box.yieldViolation; });
    if (violationDetected)
        m_violationAlertTimer.restart();
    result.yieldViolationAlert =
        m_violationAlertTimer.isValid() && m_violationAlertTimer.elapsed() < 3000;

}

void YoloV8Worker::processNextFrame()
{
    if (!m_running || !m_detector.isLoaded() || !m_frameBuffer) return;

    // 1. 取帧
    int frameSeq = -1;
    if (!m_frameBuffer->copyLatestFrame(m_frameBuf565, frameSeq)
        || frameSeq == m_lastProcessedSeq) {
        QTimer::singleShot(2, this, [this]() { processNextFrame(); });
        return;
    }
    m_lastProcessedSeq = frameSeq;

    // 2. RGB565 → RGB888
    ImageConverter::rgb565ToRGB888Fast(m_frameBuf565, m_rgb888Buf,
                                       FRAME_WIDTH, FRAME_HEIGHT);

    // 3. YOLOPv2 detection + drivable-area/lane segmentation.
    YoloDetectResult result;
    if (!m_detector.detect(m_rgb888Buf, FRAME_WIDTH, FRAME_HEIGHT, result)) {
        QTimer::singleShot(0, this, [this]() { processNextFrame(); });
        return;
    }

    if (m_yieldDetectionEnabled) {
        // ARM_final1 behaviour: expand road-user boxes and evaluate vehicle
        // yielding only while a crosswalk has been observed recently.
        detectYieldViolation(result);
    } else {
        m_laneDepartureAnalyzer.update(result);
        // YOLOPv2 and Detect3D do not use the legacy crosswalk gate.
        result.yieldViolationAlert = false;
    }

    // 4. Write the stable result cache.
    {
        QMutexLocker lock(&m_resultMutex);
        m_stableResult = std::move(result);
        m_stableFrameSeq = frameSeq;
        m_hasResult = true;
    }

    emit inferenceDone(frameSeq);

    // 5. Next frame.
    QTimer::singleShot(0, this, [this]() { processNextFrame(); });
}
