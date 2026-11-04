#include "overlay_renderer.h"
#include "obb_detector.h"
#include "yolov8_detector.h"
#include <algorithm>
#include <cmath>
#include <QPainterPath>
#include <QPainterPathStroker>

// 15 类调色板
const QColor OverlayRenderer::s_defaultPalette[] = {
    QColor(0,   0,   255),   //  0 蓝
    QColor(0,   255, 0  ),   //  1 绿
    QColor(255, 0,   0  ),   //  2 红
    QColor(255, 255, 0  ),   //  3 黄
    QColor(255, 0,   255),   //  4 品红
    QColor(0,   255, 255),   //  5 青
    QColor(255, 128, 0  ),   //  6 橙
    QColor(128, 0,   255),   //  7 紫
    QColor(0,   128, 255),   //  8 天蓝
    QColor(255, 0,   128),   //  9 粉红
    QColor(128, 255, 0  ),   // 10 黄绿
    QColor(0,   255, 128),   // 11 青绿
    QColor(128, 128, 255),   // 12 淡紫
    QColor(255, 128, 128),   // 13 淡红
    QColor(64,  255, 255),   // 14 淡青
};
const int OverlayRenderer::s_paletteSize = 15;

QColor OverlayRenderer::classColor(int clsId) {
    return s_defaultPalette[clsId % s_paletteSize];
}

void OverlayRenderer::setClassColor(int /*classId*/, const QColor &/*color*/) {
    // 预留接口
}

static void drawTopRightRedAlert(QPainter &p, const QString &text, int top)
{
    QFont font = p.font();
    font.setPixelSize(28);
    font.setBold(true);
    p.setFont(font);
    const QFontMetrics fm(font);
    const int width = fm.horizontalAdvance(text) + 30;
    const int height = fm.height() + 16;
    // All video overlays use the 1280x720 source-frame coordinate system.
    const QRect rect(std::max(20, 1280 - width - 20), top, width, height);
    p.fillRect(rect, QColor(0, 0, 0, 205));
    p.setPen(QPen(QColor(255, 35, 35), 2));
    p.drawRect(rect);
    p.setPen(QColor(255, 45, 45));
    p.drawText(rect.left() + 15, rect.top() + 8 + fm.ascent(), text);
}

static void drawObbBoxes(QPainter &p, const DetectResult &result,
                         const char *const *classNames, int numClasses) {
    if (result.boxes.empty()) return;

    p.setRenderHint(QPainter::Antialiasing, false);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    for (const auto &b : result.boxes) {
        QColor color = OverlayRenderer::classColor(b.classId);
        // 车牌标签按真实车牌底色：黄牌黄底、绿牌绿底、蓝牌蓝底
        QColor labelBg = color;
        if (b.confidence < 0.51f) continue;  // 只显示置信度≥51%的框
        if (b.classId == 1 || b.classId == 3)  // 黄牌(单层/双层) → 黄底
            labelBg = QColor(0xCC, 0x99, 0x00);
        else if (b.classId == 2)                // 绿牌 → 绿底
            labelBg = QColor(0x00, 0x88, 0x33);

        float aCos = cosf(b.angle), aSin = sinf(b.angle);
        float hw = b.w / 2.0f, hh = b.h / 2.0f;
        QPointF corners[5];
        float dx[4] = {-hw,  hw,  hw, -hw};
        float dy[4] = {-hh, -hh,  hh,  hh};
        for (int i = 0; i < 4; i++) {
            corners[i].setX(aCos * dx[i] - aSin * dy[i] + b.cx);
            corners[i].setY(aSin * dx[i] + aCos * dy[i] + b.cy);
        }
        corners[4] = corners[0];

        QPen pen(color, 2);
        p.setPen(pen);
        p.drawPolyline(corners, 5);

        QString className = QStringLiteral("?");
        if (classNames && b.classId >= 0 && b.classId < numClasses &&
            classNames[b.classId] != nullptr) {
            className = QString::fromUtf8(classNames[b.classId]);
        }
        QString label;
        if (b.trackId >= 0)
            label = QStringLiteral("#%1 %2 %3%").arg(b.trackId).arg(className).arg((int)(b.confidence * 100));
        else
            label = QStringLiteral("%1 %2%").arg(className).arg((int)(b.confidence * 100));
        if (!b.plateName.empty())
            label += QStringLiteral(" %1").arg(QString::fromUtf8(b.plateName.data(), (int)b.plateName.size()));
        if (!b.vehicleLabel.empty())
            label += QStringLiteral(" %1").arg(QString::fromUtf8(b.vehicleLabel.data(), (int)b.vehicleLabel.size()));

        QFont font = p.font();
        font.setPixelSize(18);
        font.setBold(true);
        p.setFont(font);
        const QFontMetrics fm(font);
        const int textW = fm.horizontalAdvance(label) + 6;
        const int textH = fm.height() + 2;

        int labelX = (int)corners[0].x();
        int labelY = (int)corners[0].y() - textH;
        if (labelY < 0) labelY = (int)corners[2].y() + 2;

        p.fillRect(labelX, labelY, textW, textH, labelBg);
        p.setPen(Qt::white);
        p.drawText(labelX + 3, labelY + fm.ascent(), label);
    }
}

static void drawYoloBoxes(QPainter &p, const YoloDetectResult &result,
                          const char *const *classNames, int numClasses,
                          bool trafficPalette = false) {
    p.setRenderHint(QPainter::Antialiasing, false);

    if (!trafficPalette && result.hasSegmentation()) {
        p.drawImage(QPoint(0, 0),
                    OverlayRenderer::buildYoloSemanticOverlay(result));
    }

    // cameraCenterX is populated whenever lane analysis actually ran. Keep a
    // visible reference/status even if two reliable lane boundaries cannot be
    // fitted yet; otherwise an invalid fit looks like a missing feature.
    if (!trafficPalette && result.cameraCenterX > 0) {
        const int referenceBottom = result.laneGeometryValid
            ? result.laneGeometryBottomY : 662;
        const int markerY = referenceBottom - 18;
        p.setRenderHint(QPainter::Antialiasing, true);

        // High-contrast camera optical-center marker: black outline, magenta
        // crosshair and an upward triangle remain visible over any road color.
        p.setPen(QPen(QColor(0, 0, 0, 220), 9));
        p.drawLine(QPoint(result.cameraCenterX, markerY - 48),
                   QPoint(result.cameraCenterX, markerY + 18));
        p.drawLine(QPoint(result.cameraCenterX - 18, markerY),
                   QPoint(result.cameraCenterX + 18, markerY));
        p.setPen(QPen(QColor(255, 0, 255), 5));
        p.drawLine(QPoint(result.cameraCenterX, markerY - 48),
                   QPoint(result.cameraCenterX, markerY + 18));
        p.drawLine(QPoint(result.cameraCenterX - 18, markerY),
                   QPoint(result.cameraCenterX + 18, markerY));
        const QPolygon cameraTriangle({
            QPoint(result.cameraCenterX, markerY - 58),
            QPoint(result.cameraCenterX - 11, markerY - 40),
            QPoint(result.cameraCenterX + 11, markerY - 40)
        });
        p.setPen(QPen(Qt::black, 3));
        p.setBrush(QColor(255, 0, 255));
        p.drawPolygon(cameraTriangle);
        p.setBrush(Qt::NoBrush);

    }

    if (!trafficPalette && result.cameraCenterX > 0 &&
        !result.laneGeometryValid) {
        const int candidateTopWidth = result.laneRightTopX - result.laneLeftTopX;
        const int candidateBottomWidth = result.laneRightBottomX - result.laneLeftBottomX;
        const QString stage = result.laneFitStatus == 1
            ? QStringLiteral("左线拟合失败")
            : result.laneFitStatus == 2
                ? QStringLiteral("右线拟合失败")
                : result.laneFitStatus == 3
                    ? QStringLiteral("几何校验失败")
                    : QStringLiteral("等待车道线");
        const QString status = result.laneFitStatus == 3
            ? QStringLiteral("%1 上宽:%2 下宽:%3 左:%4 右:%5")
                .arg(stage)
                .arg(candidateTopWidth)
                .arg(candidateBottomWidth)
                .arg(result.laneLeftSamples)
                .arg(result.laneRightSamples)
            : QStringLiteral("%1  左:%2 右:%3")
            .arg(stage)
            .arg(result.laneLeftSamples)
            .arg(result.laneRightSamples);
        QFont statusFont = p.font();
        statusFont.setPixelSize(18);
        statusFont.setBold(true);
        p.setFont(statusFont);
        const QFontMetrics fm(statusFont);
        const QRect statusRect(20, 620,
                               fm.horizontalAdvance(status) + 16,
                               fm.height() + 8);
        p.fillRect(statusRect, QColor(0, 0, 0, 185));
        p.setPen(QColor(255, 220, 0));
        p.drawText(statusRect.left() + 8,
                   statusRect.top() + 4 + fm.ascent(), status);
    }

    if (!trafficPalette && result.laneGeometryValid) {
        const QPoint leftTop(result.laneLeftTopX, result.laneGeometryTopY);
        const QPoint leftBottom(result.laneLeftBottomX, result.laneGeometryBottomY);
        const QPoint rightTop(result.laneRightTopX, result.laneGeometryTopY);
        const QPoint rightBottom(result.laneRightBottomX, result.laneGeometryBottomY);
        const int centerTop = (result.laneLeftTopX + result.laneRightTopX) / 2;
        p.setPen(QPen(QColor(255, 220, 0), 4));
        p.drawLine(leftTop, leftBottom);
        p.drawLine(rightTop, rightBottom);

        p.setPen(QPen(QColor(0, 255, 255), 3, Qt::DashLine));
        p.drawLine(QPoint(centerTop, result.laneGeometryTopY),
                   QPoint(result.laneCenterX, result.laneGeometryBottomY));

        // Horizontal correction arrow: always points from the camera optical
        // center toward the fitted lane center. Red denotes an active alarm.
        const int arrowY = result.laneGeometryBottomY - 82;
        const int arrowStartX = result.cameraCenterX;
        const int arrowEndX = result.laneCenterX;
        const QColor arrowColor = result.laneDepartureAlert
            ? QColor(255, 35, 35) : QColor(255, 150, 0);
        p.setPen(QPen(QColor(0, 0, 0, 210), 10));
        p.drawLine(QPoint(arrowStartX, arrowY), QPoint(arrowEndX, arrowY));
        p.setPen(QPen(arrowColor, 6));
        p.drawLine(QPoint(arrowStartX, arrowY), QPoint(arrowEndX, arrowY));
        const int direction = arrowEndX >= arrowStartX ? 1 : -1;
        const QPolygon arrowHead({
            QPoint(arrowEndX, arrowY),
            QPoint(arrowEndX - direction * 18, arrowY - 11),
            QPoint(arrowEndX - direction * 18, arrowY + 11)
        });
        p.setPen(QPen(Qt::black, 2));
        p.setBrush(arrowColor);
        p.drawPolygon(arrowHead);
        p.setBrush(Qt::NoBrush);

        const QString correctionText = std::abs(arrowEndX - arrowStartX) < 6
            ? QStringLiteral("车辆已居中")
            : direction < 0
                ? QStringLiteral("向左靠近车道中心")
                : QStringLiteral("向右靠近车道中心");
        QFont correctionFont = p.font();
        correctionFont.setPixelSize(17);
        correctionFont.setBold(true);
        p.setFont(correctionFont);
        const QFontMetrics correctionFm(correctionFont);
        const int correctionCenterX = (arrowStartX + arrowEndX) / 2;
        const int correctionX = correctionCenterX -
                                correctionFm.horizontalAdvance(correctionText) / 2;
        const QRect correctionRect(correctionX - 5,
                                   arrowY - correctionFm.height() - 9,
                                   correctionFm.horizontalAdvance(correctionText) + 10,
                                   correctionFm.height() + 5);
        p.fillRect(correctionRect, QColor(0, 0, 0, 185));
        p.setPen(arrowColor);
        p.drawText(correctionX, correctionRect.top() +
                   correctionFm.ascent() + 2, correctionText);

        p.setPen(QColor(0, 255, 255));
        p.setBrush(QColor(0, 255, 255));
        p.drawEllipse(QPoint(result.laneCenterX, arrowY), 7, 7);
        p.setBrush(Qt::NoBrush);

        const QString roadText = result.roadDirectionConfidence <= 0.0f
            ? QStringLiteral("前方道路：判断中")
            : result.roadDirection < 0
            ? QStringLiteral("前方道路：向左")
            : result.roadDirection > 0
                ? QStringLiteral("前方道路：向右")
                : QStringLiteral("前方道路：直行");
        QFont roadFont = p.font();
        roadFont.setPixelSize(34);
        roadFont.setBold(true);
        p.setFont(roadFont);
        const QFontMetrics roadFm(roadFont);
        const int iconSize = 96;
        const int roadBoxWidth = iconSize + roadFm.horizontalAdvance(roadText) + 68;
        // Keep clear of the multi-line HDMI/FPS/feature HUD drawn at (24, 24).
        const QRect roadRect(20, 230, roadBoxWidth, iconSize + 24);
        p.fillRect(roadRect, QColor(0, 0, 0, 205));

        // Compact blue road-direction icon based on the supplied reference
        // images. It is drawn in code so it remains available on the ARM board.
        const QRectF iconRect(roadRect.left() + 12, roadRect.top() + 12,
                              iconSize, iconSize);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(QPen(QColor(235, 245, 255), 4));
        p.setBrush(result.roadDirectionConfidence > 0.0f
                   ? QColor(0, 76, 190) : QColor(90, 100, 115));
        p.drawEllipse(iconRect);

        if (result.roadDirectionConfidence <= 0.0f) {
            QFont iconFont = roadFont;
            iconFont.setPixelSize(54);
            iconFont.setBold(true);
            p.setFont(iconFont);
            p.setPen(Qt::white);
            p.drawText(iconRect, Qt::AlignCenter, QStringLiteral("?"));
            p.setFont(roadFont);
        } else {
            // Build one filled arrow silhouette. The old implementation drew
            // a round-capped thick line and then added a triangle, which made
            // the bend look like a white blob on the real HDMI display.
            QPainterPath centerLine;
            QPolygonF head;
            const qreal cx = iconRect.center().x();
            const qreal top = iconRect.top();
            const qreal bottom = iconRect.bottom();
            if (result.roadDirection < 0) {
                const QPointF neck(iconRect.left() + 39.0, top + 31.0);
                centerLine.moveTo(cx + 10.0, bottom - 17.0);
                centerLine.cubicTo(cx + 10.0, iconRect.center().y() + 8.0,
                                   cx - 5.0, top + 31.0,
                                   neck.x(), neck.y());
                head << QPointF(iconRect.left() + 12.0, top + 31.0)
                     << QPointF(iconRect.left() + 40.0, top + 14.0)
                     << QPointF(iconRect.left() + 40.0, top + 48.0);
            } else if (result.roadDirection > 0) {
                const QPointF neck(iconRect.right() - 39.0, top + 31.0);
                centerLine.moveTo(cx - 10.0, bottom - 17.0);
                centerLine.cubicTo(cx - 10.0, iconRect.center().y() + 8.0,
                                   cx + 5.0, top + 31.0,
                                   neck.x(), neck.y());
                head << QPointF(iconRect.right() - 12.0, top + 31.0)
                     << QPointF(iconRect.right() - 40.0, top + 14.0)
                     << QPointF(iconRect.right() - 40.0, top + 48.0);
            } else {
                centerLine.moveTo(cx, bottom - 16.0);
                centerLine.lineTo(cx, top + 39.0);
                head << QPointF(cx, top + 12.0)
                     << QPointF(cx - 18.0, top + 40.0)
                     << QPointF(cx + 18.0, top + 40.0);
            }

            QPainterPathStroker stroker;
            stroker.setWidth(15.0);
            stroker.setCapStyle(Qt::FlatCap);
            stroker.setJoinStyle(Qt::RoundJoin);
            const QPainterPath arrowBody = stroker.createStroke(centerLine);
            p.setPen(Qt::NoPen);
            p.setBrush(Qt::white);
            p.drawPath(arrowBody);
            p.drawPolygon(head);
            p.setBrush(Qt::NoBrush);
        }

        p.setPen(result.roadDirectionConfidence <= 0.0f
                 ? QColor(220, 220, 220)
                 : result.roadDirection == 0
                 ? QColor(60, 255, 120) : QColor(255, 185, 0));
        const int textX = roadRect.left() + iconSize + 34;
        const int textY = roadRect.center().y() - roadFm.height() / 2 +
                          roadFm.ascent();
        p.drawText(textX, textY, roadText);
        const QString offsetText = QStringLiteral("车道偏移 %1%")
            .arg(result.laneOffsetRatio * 100.0f, 0, 'f', 1);
        QFont infoFont = p.font();
        infoFont.setPixelSize(18);
        infoFont.setBold(true);
        p.setFont(infoFont);
        const QFontMetrics infoMetrics(infoFont);
        const QRect infoRect(20, result.laneGeometryBottomY - 42,
                             infoMetrics.horizontalAdvance(offsetText) + 16,
                             infoMetrics.height() + 8);
        p.fillRect(infoRect, QColor(0, 0, 0, 175));
        p.setPen(Qt::white);
        p.drawText(infoRect.left() + 8,
                   infoRect.top() + 4 + infoMetrics.ascent(), offsetText);
    }

    for (const auto &b : result.boxes) {
        QColor color;
        if (b.yieldViolation) {
            color = QColor(220, 0, 0);
        } else if (trafficPalette && b.classId == 0) {
            color = QColor(0, 220, 255);
        } else if (trafficPalette && b.classId == 1) {
            color = QColor(255, 32, 32);
        } else if (trafficPalette && b.classId == 2) {
            color = QColor(32, 220, 64);
        } else {
            color = OverlayRenderer::classColor(b.classId);
        }
        QRect rect(b.left, b.top, b.right - b.left, b.bottom - b.top);
        if (rect.width() <= 0 || rect.height() <= 0) continue;

        QPen pen(color, b.yieldViolation ? 4 : 2);
        p.setPen(pen);
        p.drawRect(rect);

        if (b.has3dProjection) {
            static const int edges[12][2] = {
                {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4},
                {0,4},{1,5},{2,6},{3,7}
            };
            p.setRenderHint(QPainter::Antialiasing, true);
            p.setPen(QPen(QColor(0, 255, 255), 3));
            for (const auto &edge : edges) {
                const int a = edge[0] * 2;
                const int c = edge[1] * 2;
                p.drawLine(QPointF(b.projectedCorners[a], b.projectedCorners[a + 1]),
                           QPointF(b.projectedCorners[c], b.projectedCorners[c + 1]));
            }
            p.setRenderHint(QPainter::Antialiasing, false);
        }

        QString className = QStringLiteral("?");
        if (classNames && b.classId >= 0 && b.classId < numClasses &&
            classNames[b.classId] != nullptr) {
            // Label files are UTF-8. Copy to an owned QString explicitly.
            className = QString::fromUtf8(classNames[b.classId]);
        }
        QString label = b.yieldViolation
            ? QStringLiteral("\u672A\u793C\u8BA9\u884C\u4EBA")
            : QStringLiteral("%1 %2%").arg(className).arg((int)(b.confidence * 100));
        if (b.depthMeters > 0.0f && std::isfinite(b.depthMeters))
            label += QStringLiteral("  %1m").arg(b.depthMeters, 0, 'f', 1);

        QFont font = p.font();
        font.setPixelSize(18);
        font.setBold(true);
        p.setFont(font);
        const QFontMetrics fm(font);
        const int tw = fm.horizontalAdvance(label) + 6;
        const int th = fm.height() + 2;
        int lx = rect.left();
        int ly = rect.top() - th;
        if (ly < 0) ly = rect.bottom() + 2;

        p.fillRect(lx, ly, tw, th, color);
        p.setPen(Qt::white);
        p.drawText(lx + 3, ly + fm.ascent(), label);
    }

    if (!trafficPalette && result.helmetNoHelmetAlert)
        drawTopRightRedAlert(
            p, QStringLiteral("警告：骑行者未戴头盔"), 20);
    if (trafficPalette && result.redLightCrosswalkPedestrianAlert)
        drawTopRightRedAlert(
            p, QStringLiteral("警告：有行人闯红灯"), 78);
    if (result.yieldViolationAlert)
        drawTopRightRedAlert(
            p, QStringLiteral("警告：有车辆未礼让行人"), 136);


    if (!trafficPalette && result.laneDepartureAlert) {
        const QString warning = result.laneDepartureDirection < 0
            ? QStringLiteral("车道偏离：向左")
            : QStringLiteral("车道偏离：向右");
        QFont warningFont = p.font();
        warningFont.setPixelSize(30);
        warningFont.setBold(true);
        p.setFont(warningFont);
        const QFontMetrics fm(warningFont);
        const QRect warningRect(20, result.yieldViolationAlert ? 78 : 20,
                                fm.horizontalAdvance(warning) + 28,
                                fm.height() + 16);
        p.fillRect(warningRect, QColor(220, 0, 0, 235));
        p.setPen(Qt::white);
        p.drawText(warningRect.left() + 14,
                   warningRect.top() + 8 + fm.ascent(), warning);
    }
}

QImage OverlayRenderer::buildYoloSemanticOverlay(
    const YoloDetectResult &result)
{
    if (!result.hasSegmentation()) return QImage();

    QImage semantic(result.segmentationWidth, result.segmentationHeight,
                    QImage::Format_ARGB32);
    semantic.fill(Qt::transparent);
    for (int y = 0; y < result.segmentationHeight; ++y) {
        QRgb *row = reinterpret_cast<QRgb *>(semantic.scanLine(y));
        const size_t offset = static_cast<size_t>(y) *
                              result.segmentationWidth;
        for (int x = 0; x < result.segmentationWidth; ++x) {
            const size_t index = offset + static_cast<size_t>(x);
            // Restore the original thick red lane-mask visualization. Lane
            // pixels take priority over the green drivable-area overlay.
            if (result.laneMask[index] != 0)
                row[x] = qRgba(255, 20, 20, 125);
            else if (result.drivableMask[index] != 0)
                row[x] = qRgba(0, 220, 0, 80);
        }
    }
    return semantic;
}

// ---- 在 QImage 上绘制 OBB + 标签 ----
void OverlayRenderer::draw(QImage &image, const DetectResult &result,
                            const char *const *classNames, int numClasses) {
    QPainter p(&image);
    drawObbBoxes(p, result, classNames, numClasses);
    p.end();
}

void OverlayRenderer::draw(QPainter &painter, const DetectResult &result,
                           const char *const *classNames, int numClasses) {
    drawObbBoxes(painter, result, classNames, numClasses);
}

// ---- 绘制标准 YOLOv8 轴对齐矩形框 ----
void OverlayRenderer::drawYoloRects(QImage &image, const YoloDetectResult &result,
                                     const char *const *classNames, int numClasses) {
    QPainter p(&image);
    drawYoloBoxes(p, result, classNames, numClasses);
    p.end();
}

void OverlayRenderer::drawYoloRects(QPainter &painter, const YoloDetectResult &result,
                                    const char *const *classNames, int numClasses) {
    drawYoloBoxes(painter, result, classNames, numClasses);
}

void OverlayRenderer::drawTrafficRects(QImage &image,
                                       const YoloDetectResult &result,
                                       const char *const *classNames,
                                       int numClasses) {
    QPainter p(&image);
    drawYoloBoxes(p, result, classNames, numClasses, true);
    p.end();
}

void OverlayRenderer::drawTrafficRects(QPainter &painter,
                                       const YoloDetectResult &result,
                                       const char *const *classNames,
                                       int numClasses) {
    drawYoloBoxes(painter, result, classNames, numClasses, true);
}
