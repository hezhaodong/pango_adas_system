#include "video_frame_widget.h"
#include "overlay_renderer.h"
#include "obb_detector.h"
#include "yolov8_detector.h"

#include <QPaintEvent>
#include <QPainter>

VideoFrameWidget::VideoFrameWidget(QWidget *parent)
    : QWidget(parent)
    , m_placeholderText("等待采集...")
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
{
    setAttribute(Qt::WA_OpaquePaintEvent);
}

VideoFrameWidget::~VideoFrameWidget()
{
    delete m_obbResult;
    delete m_yoloResult;
    delete m_yieldYoloResult;
    delete m_detect3dResult;
    delete m_helmetResult;
    delete m_trafficResult;
}

void VideoFrameWidget::presentFrame(const QImage &frame)
{
    m_frame = frame;
    m_placeholderText.clear();
    update();
}

void VideoFrameWidget::setObbOverlay(const DetectResult &result,
                                     const char *const *classNames, int numClasses)
{
    if (!m_obbResult)
        m_obbResult = new DetectResult();
    *m_obbResult = result;
    m_obbClassNames = classNames;
    m_obbNumClasses = numClasses;
    update();
}

void VideoFrameWidget::clearObbOverlay()
{
    if (m_obbResult)
        m_obbResult->boxes.clear();
    m_obbClassNames = nullptr;
    m_obbNumClasses = 0;
    update();
}

void VideoFrameWidget::setYoloOverlay(const YoloDetectResult &result,
                                      const char *const *classNames, int numClasses)
{
    if (!m_yoloResult)
        m_yoloResult = new YoloDetectResult();
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
    m_yoloClassNames = classNames;
    m_yoloNumClasses = numClasses;
    update();
}

void VideoFrameWidget::clearYoloOverlay()
{
    if (m_yoloResult)
        *m_yoloResult = YoloDetectResult();
    m_yoloSemanticOverlay = QImage();
    m_yoloClassNames = nullptr;
    m_yoloNumClasses = 0;
    update();
}

void VideoFrameWidget::setYieldYoloOverlay(
    const YoloDetectResult &result, const char *const *classNames,
    int numClasses)
{
    if (!m_yieldYoloResult)
        m_yieldYoloResult = new YoloDetectResult();
    *m_yieldYoloResult = result;
    m_yieldYoloClassNames = classNames;
    m_yieldYoloNumClasses = numClasses;
    update();
}

void VideoFrameWidget::clearYieldYoloOverlay()
{
    if (m_yieldYoloResult)
        *m_yieldYoloResult = YoloDetectResult();
    m_yieldYoloClassNames = nullptr;
    m_yieldYoloNumClasses = 0;
    update();
}

void VideoFrameWidget::setDetect3dOverlay(const YoloDetectResult &result,
                                          const char *const *classNames,
                                          int numClasses)
{
    if (!m_detect3dResult) m_detect3dResult = new YoloDetectResult();
    *m_detect3dResult = result;
    m_detect3dClassNames = classNames;
    m_detect3dNumClasses = numClasses;
    update();
}

void VideoFrameWidget::clearDetect3dOverlay()
{
    if (m_detect3dResult) *m_detect3dResult = YoloDetectResult();
    m_detect3dClassNames = nullptr;
    m_detect3dNumClasses = 0;
    update();
}

void VideoFrameWidget::setHelmetOverlay(const YoloDetectResult &result,
                                        const char *const *classNames, int numClasses)
{
    if (!m_helmetResult) m_helmetResult = new YoloDetectResult();
    *m_helmetResult = result;
    m_helmetClassNames = classNames;
    m_helmetNumClasses = numClasses;
    update();
}

void VideoFrameWidget::clearHelmetOverlay()
{
    if (m_helmetResult) m_helmetResult->boxes.clear();
    m_helmetClassNames = nullptr;
    m_helmetNumClasses = 0;
    update();
}

void VideoFrameWidget::setTrafficOverlay(const YoloDetectResult &result,
                                         const char *const *classNames,
                                         int numClasses)
{
    if (!m_trafficResult) m_trafficResult = new YoloDetectResult();
    *m_trafficResult = result;
    m_trafficClassNames = classNames;
    m_trafficNumClasses = numClasses;
    update();
}

void VideoFrameWidget::clearTrafficOverlay()
{
    if (m_trafficResult) m_trafficResult->boxes.clear();
    m_trafficClassNames = nullptr;
    m_trafficNumClasses = 0;
    update();
}
void VideoFrameWidget::showPlaceholder(const QString &text)
{
    m_frame = QImage();
    m_placeholderText = text;
    update();
}

void VideoFrameWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), m_frame.isNull() ? QColor(0x8b, 0x00, 0x00) : QColor(0x1a, 0x1a, 0x1a));

    if (!m_frame.isNull()) {
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
        return;
    }

    painter.setPen(QColor(0xff, 0xff, 0xff));
    painter.setFont(font());
    painter.drawText(rect(), Qt::AlignCenter, m_placeholderText);
}
