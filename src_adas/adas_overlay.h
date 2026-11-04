/* adas_overlay.h — ADAS 画面叠加（新增，唯一依赖 Qt 的文件） */
#ifndef ADAS_OVERLAY_H
#define ADAS_OVERLAY_H

#include "adas_bridge.h"

class QPainter;
class QSizeF;

/* 在"已被调用方缩放/平移成帧坐标系"的 painter 上绘制叠加层。
 * 调用点：EglfsVideoWindow::paintGL() 里画完其它叠加之后。 */
void adas_paint_overlay(QPainter &painter, const QSizeF &frameSize,
                        const AdasOverlayState &state);

#endif /* ADAS_OVERLAY_H */
