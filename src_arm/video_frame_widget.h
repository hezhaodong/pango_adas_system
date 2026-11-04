#ifndef VIDEO_FRAME_WIDGET_H
#define VIDEO_FRAME_WIDGET_H

#include <QImage>
#include <QString>
#include <QWidget>

struct DetectResult;
struct YoloDetectResult;

class VideoFrameWidget : public QWidget
{
public:
    explicit VideoFrameWidget(QWidget *parent = nullptr);
    ~VideoFrameWidget() override;

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

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QImage           m_frame;
    QString          m_placeholderText;
    DetectResult    *m_obbResult;
    const char *const *m_obbClassNames;
    int              m_obbNumClasses;
    YoloDetectResult *m_yoloResult;
    QImage            m_yoloSemanticOverlay;
    const char *const *m_yoloClassNames;
    int              m_yoloNumClasses;
    YoloDetectResult *m_yieldYoloResult;
    const char *const *m_yieldYoloClassNames;
    int              m_yieldYoloNumClasses;
    YoloDetectResult *m_detect3dResult;
    const char *const *m_detect3dClassNames;
    int              m_detect3dNumClasses;
    YoloDetectResult *m_helmetResult;
    const char *const *m_helmetClassNames;
    int              m_helmetNumClasses;
    YoloDetectResult *m_trafficResult;
    const char *const *m_trafficClassNames;
    int              m_trafficNumClasses;
};

#endif // VIDEO_FRAME_WIDGET_H
