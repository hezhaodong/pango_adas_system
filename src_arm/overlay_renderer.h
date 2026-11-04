#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include <QImage>
#include <QPainter>
#include <vector>

struct ObbBox;
struct DetectResult;
struct YoloBox;
struct YoloDetectResult;

class OverlayRenderer
{
public:
    // 绘制 OBB 旋转框 + 标签
    static void draw(QImage &image, const DetectResult &result,
                     const char *const *classNames, int numClasses);
    static void draw(QPainter &painter, const DetectResult &result,
                     const char *const *classNames, int numClasses);

    // Draw boxes plus optional YOLOPv2 drivable/lane masks.
    static void drawYoloRects(QImage &image, const YoloDetectResult &result,
                              const char *const *classNames, int numClasses);
    static void drawYoloRects(QPainter &painter, const YoloDetectResult &result,
                              const char *const *classNames, int numClasses);
    // Build the expensive full-frame semantic layer once per inference result.
    // Display widgets cache this image and reuse it for subsequent paint calls.
    static QImage buildYoloSemanticOverlay(const YoloDetectResult &result);

    // Traffic classes use semantic colors: crosswalk=cyan, red=red, green=green.
    static void drawTrafficRects(QImage &image, const YoloDetectResult &result,
                                 const char *const *classNames, int numClasses);
    static void drawTrafficRects(QPainter &painter,
                                 const YoloDetectResult &result,
                                 const char *const *classNames, int numClasses);

    static QColor classColor(int classId);
    static void setClassColor(int classId, const QColor &color);

private:
    static const QColor s_defaultPalette[];
    static const int    s_paletteSize;
};

#endif // OVERLAY_RENDERER_H
