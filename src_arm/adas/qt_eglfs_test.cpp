/* qt_eglfs_test.cpp — 最小 Qt EGLFS 测试: 往屏幕刷纯色
 *
 * 目的: 把问题范围从"上位机 app"缩小到"Qt EGLFS 能不能上屏"。
 *   能刷出颜色 -> Qt 显示正常, 问题在原 app 的窗口/刷新逻辑
 *   刷不出来   -> Qt EGLFS 在这块板子上就不工作, 应改用裸 DRM 方案
 *
 * 编译(板上): g++ -O2 -fPIC -o qt_eglfs_test qt_eglfs_test.cpp \
 *              $(pkg-config --cflags --libs Qt5Widgets)
 * 运行:      QT_QPA_PLATFORM=eglfs QT_QPA_EGLFS_INTEGRATION=eglfs_kms \
 *              QT_QPA_EGLFS_KMS_CONFIG=/userdata/kms.json ./qt_eglfs_test
 */
#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QScreen>
#include <cstdio>

class TestWidget : public QWidget {
public:
    TestWidget() {
        setWindowTitle("Qt EGLFS Test");
        m_tick = 0;
        // 每 500ms 换一次颜色, 便于肉眼确认在刷新
        QTimer *t = new QTimer(this);
        connect(t, &QTimer::timeout, this, [this]() {
            m_tick++;
            printf("tick=%d visible=%d size=%dx%d\n", m_tick,
                   isVisible() ? 1 : 0, width(), height());
            fflush(stdout);
            update();
            if (m_tick >= 20) qApp->quit();
        });
        t->start(500);
        printf("[TestWidget] constructed, screen=%dx%d\n",
               screen() ? screen()->geometry().width() : -1,
               screen() ? screen()->geometry().height() : -1);
        fflush(stdout);
    }

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        // 四色彩条 + 大号数字, 便于确认画面确实在变
        static const QColor cols[4] = { Qt::red, Qt::green, Qt::blue, Qt::yellow };
        int w = width(), h = height();
        for (int i = 0; i < 4; i++) {
            p.fillRect(i * w / 4, 0, w / 4 + 1, h, cols[(i + m_tick) % 4]);
        }
        p.setPen(Qt::white);
        QFont f = p.font();
        f.setPointSize(200);
        f.setBold(true);
        p.setFont(f);
        p.drawText(rect(), Qt::AlignCenter, QString::number(m_tick));
        printf("[paintEvent] painted %dx%d tick=%d\n", w, h, m_tick);
        fflush(stdout);
    }

    void showEvent(QShowEvent *) override {
        printf("[showEvent] window shown %dx%d\n", width(), height());
        fflush(stdout);
    }

private:
    int m_tick;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    printf("[main] platform=%s\n", qPrintable(app.platformName()));
    fflush(stdout);

    TestWidget w;
    w.resize(1440, 900);
    w.show();

    int rc = app.exec();
    printf("[main] exited rc=%d\n", rc);
    return rc;
}
