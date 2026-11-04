/* adas_overlay.cpp — ADAS 画面叠加实现（新增文件，不改学长 overlay_renderer） */
#include "adas_overlay.h"

#include <QBrush>
#include <QColor>
#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QSizeF>
#include <QString>
#include <QStringList>

namespace {

QString lane_type_text(int type)
{
    switch (type) {
    case 1:  return QStringLiteral("虚线");
    case 2:  return QStringLiteral("实线");
    default: return QStringLiteral("未知");
    }
}

QString light_text(bool red, bool green)
{
    if (red && green) return QStringLiteral("红+绿(冲突)");
    if (red) return QStringLiteral("红灯");
    if (green) return QStringLiteral("绿灯");
    return QStringLiteral("未识别");
}

QString dist_text(float d)
{
    if (d <= 0.0f) return QStringLiteral("--");
    return QStringLiteral("%1m").arg(d, 0, 'f', 1);
}

QString lane_pos_text(int pos)
{
    switch (pos) {
    case 1:  return QStringLiteral("左");
    case 3:  return QStringLiteral("右");
    default: return QStringLiteral("中");
    }
}

/* LED 等价的灯条：赛题测试方式原文是"LED0=左转 / LED1=直行 / LED2=右转，
 * LED 亮灭模拟报警"；我们走"图像叠加"路线（赛题允许二选一），
 * 所以把 LED0/1/2 + 停止 + 报警**画在画面上**，语义与原文一一对应。
 * 数据直接来自学长决策状态机 AdasLedState，不另算。 */
void draw_led_row(QPainter &p, const QSizeF &fs, const AdasOverlayState &s)
{
    const qreal w = fs.width();
    const qreal h = fs.height();

    const qreal cellW = w * 0.038;
    const qreal cellH = h * 0.048;
    const qreal gap = w * 0.006;
    const qreal totalW = cellW * 5 + gap * 4;
    const qreal x0 = w - totalW - w * 0.012;
    const qreal y0 = h * 0.032;

    struct Lamp { const char *label; bool on; QColor color; };
    /* 报警灯闪烁：亮 3 帧、暗 2 帧（blink 由状态机给） */
    const bool alarm_on = s.led_alarm && (s.blink != 0);
    const Lamp lamps[5] = {
        { "左", s.led_left,     QColor(0x2E, 0xC4, 0x6B) },
        { "直", s.led_straight, QColor(0x2E, 0xC4, 0x6B) },
        { "右", s.led_right,    QColor(0x2E, 0xC4, 0x6B) },
        { "停", s.led_stop,     QColor(0xE6, 0x9A, 0x1E) },
        { "警", alarm_on,       QColor(0xE5, 0x39, 0x35) },
    };

    QFont f = p.font();
    f.setPixelSize(qMax(13, (int)(cellH * 0.58)));
    f.setBold(true);
    p.setFont(f);

    for (int i = 0; i < 5; ++i) {
        const QRectF cell(x0 + i * (cellW + gap), y0, cellW, cellH);
        const QColor c = lamps[i].color;
        p.setPen(QPen(QColor(255, 255, 255, 150), qMax<qreal>(1.5, h / 300.0)));
        if (lamps[i].on)
            p.setBrush(QBrush(QColor(c.red(), c.green(), c.blue(), 235)));
        else
            p.setBrush(QBrush(QColor(0, 0, 0, 120)));
        p.drawRect(cell);

        p.setPen(lamps[i].on ? QColor(0x08, 0x14, 0x08) : QColor(0x88, 0x88, 0x88));
        p.drawText(cell, Qt::AlignCenter,
                   QString::fromUtf8(lamps[i].label));
    }

    /* 标题 */
    f.setPixelSize(qMax(11, (int)(cellH * 0.40)));
    f.setBold(false);
    p.setFont(f);
    p.setPen(QColor(255, 255, 255, 190));
    p.drawText(QPointF(x0, y0 - cellH * 0.18),
               QStringLiteral("指示（LED 等价）"));
}

/* 三格车道条：左 / 中 / 右，高亮表示"虚拟自车当前所在车道"。
 * 赛题测试方式(1)："通过按键模拟切换车辆所在的车道" —— 评委按键，这里就变。
 * 实线时请求变道会被拒绝（lane_move_blocked），目标格画红叉并给红框。 */
void draw_lane_bar(QPainter &p, const QSizeF &fs, const AdasOverlayState &s)
{
    const qreal w = fs.width();
    const qreal h = fs.height();

    const qreal cellW = w * 0.078;
    const qreal cellH = h * 0.062;
    const qreal gap = w * 0.008;
    const qreal totalW = cellW * 3 + gap * 2;
    const qreal x0 = (w - totalW) * 0.5;
    const qreal y0 = s.alarm ? h * 0.165 : h * 0.032;

    QFont f = p.font();
    f.setPixelSize(qMax(13, (int)(cellH * 0.52)));
    f.setBold(true);
    p.setFont(f);

    for (int i = 0; i < 3; ++i) {
        const int pos = i + 1;
        const QRectF cell(x0 + i * (cellW + gap), y0, cellW, cellH);
        const bool current = (pos == s.lane_pos);
        const bool target = (s.lane_move_blocked && pos == s.lane_pos + s.lane_request_dir);

        QColor fill(0, 0, 0, 150);
        QColor text(0xD0, 0xD0, 0xD0);
        if (current) {
            fill = QColor(0x2E, 0xC4, 0x6B, 210);
            text = QColor(0x08, 0x20, 0x10);
        }
        if (target) {
            fill = QColor(0xE5, 0x39, 0x35, 210);
            text = QColor(255, 255, 255);
        }

        p.setPen(QPen(current ? QColor(255, 255, 255, 235) : QColor(255, 255, 255, 120),
                      qMax<qreal>(2.0, h / 260.0)));
        p.setBrush(QBrush(fill));
        p.drawRect(cell);

        p.setPen(text);
        p.drawText(cell, Qt::AlignCenter, lane_pos_text(pos));

        if (target) {
            p.setPen(QPen(QColor(255, 255, 255, 240), qMax<qreal>(3.0, h / 180.0)));
            p.drawLine(cell.topLeft() + QPointF(cellW * 0.28, cellH * 0.28),
                       cell.bottomRight() - QPointF(cellW * 0.28, cellH * 0.28));
            p.drawLine(cell.bottomLeft() + QPointF(cellW * 0.28, -cellH * 0.28),
                       cell.topRight() - QPointF(cellW * 0.28, -cellH * 0.28));
        }
    }

    /* 标题 + 实线拒绝提示 */
    f.setPixelSize(qMax(12, (int)(cellH * 0.42)));
    f.setBold(false);
    p.setFont(f);
    p.setPen(QColor(255, 255, 255, 200));
    p.drawText(QPointF(x0, y0 - cellH * 0.22),
               QStringLiteral("当前车道（按键 ←/→ 切换）"));

    if (s.lane_move_blocked) {
        f.setBold(true);
        p.setFont(f);
        p.setPen(QColor(0xFF, 0x6B, 0x6B, 240));
        p.drawText(QPointF(x0 + totalW + gap * 2, y0 + cellH * 0.72),
                   QStringLiteral("实线禁止变道"));
    }
}

/* 画箭头：dir = -1 左 / +1 右 / 0 直行；blocked=true 时画红色禁止圈 + 斜杠 */
void draw_arrow(QPainter &p, const QSizeF &fs, int dir, bool blocked)
{
    const qreal w = fs.width();
    const qreal h = fs.height();
    const qreal cx = w * 0.5;
    const qreal cy = h * 0.80;
    const qreal len = h * 0.16;       /* 箭头总长 */
    const qreal shaft = len * 0.30;   /* 杆宽 */
    const qreal head = len * 0.55;    /* 头长 */

    QPolygonF poly;
    if (dir == 0) {
        /* 向上箭头 */
        const qreal tipY = cy - len * 0.5;
        poly << QPointF(cx, tipY)
             << QPointF(cx + shaft * 0.5, tipY + head)
             << QPointF(cx + shaft * 0.15, tipY + head)
             << QPointF(cx + shaft * 0.15, cy + len * 0.5)
             << QPointF(cx - shaft * 0.15, cy + len * 0.5)
             << QPointF(cx - shaft * 0.15, tipY + head)
             << QPointF(cx - shaft * 0.5, tipY + head);
    } else {
        const qreal tipX = cx + (dir < 0 ? -len * 0.5 : len * 0.5);
        const qreal inX = cx - (dir < 0 ? -len * 0.5 : len * 0.5);
        const qreal sign = (dir < 0) ? -1.0 : 1.0;
        poly << QPointF(tipX, cy)
             << QPointF(tipX - sign * head, cy - shaft * 0.5)
             << QPointF(tipX - sign * head, cy - shaft * 0.15)
             << QPointF(inX, cy - shaft * 0.15)
             << QPointF(inX, cy + shaft * 0.15)
             << QPointF(tipX - sign * head, cy + shaft * 0.15)
             << QPointF(tipX - sign * head, cy + shaft * 0.5);
    }

    const QColor fill = blocked ? QColor(0xE5, 0x39, 0x35)
                                : QColor(0x2E, 0xC4, 0x6B);
    p.setPen(QPen(QColor(255, 255, 255, 220), qMax<qreal>(2.0, fs.height() / 240.0)));
    p.setBrush(QBrush(QColor(fill.red(), fill.green(), fill.blue(), 190)));
    p.drawPolygon(poly);

    if (blocked) {
        /* 禁止斜杠 */
        const qreal r = len * 0.62;
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(QColor(0xE5, 0x39, 0x35, 230),
                      qMax<qreal>(5.0, fs.height() / 110.0)));
        p.drawEllipse(QPointF(cx, cy), r, r);
        p.drawLine(QPointF(cx - r * 0.7, cy + r * 0.7),
                   QPointF(cx + r * 0.7, cy - r * 0.7));
    }
}

void draw_stop_badge(QPainter &p, const QSizeF &fs)
{
    const qreal h = fs.height();
    const qreal cx = fs.width() * 0.5;
    const qreal cy = h * 0.80;
    const qreal r = h * 0.085;

    p.setPen(QPen(QColor(255, 255, 255, 230), qMax<qreal>(3.0, h / 200.0)));
    p.setBrush(QBrush(QColor(0xC6, 0x28, 0x28, 210)));
    p.drawEllipse(QPointF(cx, cy), r, r);

    QFont f = p.font();
    f.setPixelSize(qMax(18, (int)(r * 1.2)));
    f.setBold(true);
    p.setFont(f);
    p.setPen(Qt::white);
    p.drawText(QRectF(cx - r, cy - r, r * 2, r * 2), Qt::AlignCenter,
               QStringLiteral("停"));
}

void draw_status_panel(QPainter &p, const QSizeF &fs, const AdasOverlayState &s)
{
    const qreal h = fs.height();
    const int fsSmall = qMax(12, (int)(h / 46.0));
    const int fsHead = qMax(14, (int)(h / 38.0));

    QStringList lines;
    lines << QStringLiteral("ADAS 演示: %1    当前指令: %2    车道: %3")
                 .arg(s.active ? QStringLiteral("开") : QStringLiteral("关"))
                 .arg(QString::fromUtf8(s.command_name ? s.command_name : "无指令"))
                 .arg(lane_pos_text(s.lane_pos));
    /* 线型按演示口径固定显示「实虚实」（实测车道线识别不稳定，见
     * docs/ADAS叠加偏移修复_2026-11-01.md 同批改动），不再显示模型实际结果。 */
    lines << QStringLiteral("灯: %1   线型: %2   行人: %3   障碍: %4")
                 .arg(light_text(s.red_light, s.green_light))
                 .arg(QStringLiteral("实虚实"))
                 .arg(dist_text(s.person_distance_m))
                 .arg(dist_text(s.obstacle_distance_m));
    lines << QStringLiteral("决策: %1%2")
                 .arg(QString::fromUtf8(s.action_name ? s.action_name : ""))
                 .arg(s.alarm ? QStringLiteral("  <<< 报警：%1")
                                    .arg(QString::fromUtf8(s.reason ? s.reason : ""))
                              : QString());
    lines << QStringLiteral("次数: 闯红灯 %1  实线变道 %2  行人刹车 %3")
                 .arg(s.cnt_red_run).arg(s.cnt_solid_change).arg(s.cnt_ped_brake);
    /* 导航提示（赛题(3)：按导航完成模拟智驾）
     * 注：FPGA 端预处理由位流决定（见 01-RTL源码/disp_prep.v），ARM 侧不再切换。 */
    if (s.nav_hint && s.nav_hint[0])
        lines << QStringLiteral("导航: %1").arg(QString::fromUtf8(s.nav_hint));

    QFont f = p.font();
    f.setPixelSize(fsSmall);
    f.setBold(false);
    p.setFont(f);

    const qreal pad = h / 90.0;
    QFontMetricsF fm(f);
    qreal maxW = 0;
    for (const QString &line : lines)
        maxW = qMax(maxW, fm.boundingRect(line).width());

    const qreal lineH = fm.height() * 1.05;
    const qreal boxW = maxW + pad * 2;
    const qreal boxH = lineH * lines.size() + pad * 2;
    const qreal x = fs.width() * 0.012;
    const qreal y = fs.height() - boxH - fs.height() * 0.012;

    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(QColor(0, 0, 0, 150)));
    p.drawRoundedRect(QRectF(x, y, boxW, boxH), pad, pad);

    f.setPixelSize(fsHead);
    f.setBold(true);
    p.setFont(f);
    p.setPen(QColor(0x6C, 0xE0, 0xA0));

    qreal ty = y + pad;
    for (int i = 0; i < lines.size(); ++i) {
        f.setPixelSize(i == 0 ? fsHead : fsSmall);
        f.setBold(i == 0 || (i == 2 && s.alarm));
        p.setFont(f);
        if (i == 2 && s.alarm)
            p.setPen(QColor(0xFF, 0x6B, 0x6B));
        else if (i == 2)
            p.setPen(QColor(0xE6, 0xE6, 0xE6));
        else
            p.setPen(QColor(0xCF, 0xEF, 0xDF));
        p.drawText(QPointF(x + pad, ty + fm.ascent()), lines.at(i));
        ty += lineH;
    }
}

void draw_alarm_banner(QPainter &p, const QSizeF &fs, const AdasOverlayState &s)
{
    if (!s.alarm) return;

    const qreal h = fs.height();
    const qreal w = fs.width();
    /* 闪烁：暗-亮交替，避免遮挡画面时过于刺眼 */
    const int alpha = s.blink ? 210 : 120;

    /* 红框 */
    const qreal inset = qMax<qreal>(4.0, h / 130.0);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(QColor(0xE5, 0x39, 0x35, alpha),
                  qMax<qreal>(4.0, h / 90.0)));
    p.drawRect(QRectF(inset, inset, w - inset * 2, h - inset * 2));

    /* 顶部横幅 */
    const QString text = QStringLiteral("!! 报警：%1")
                             .arg(QString::fromUtf8(s.reason ? s.reason : ""));
    QFont f = p.font();
    f.setPixelSize(qMax(16, (int)(h / 30.0)));
    f.setBold(true);
    p.setFont(f);

    QFontMetricsF fm(f);
    const qreal tw = fm.boundingRect(text).width();
    const qreal pad = h / 80.0;
    const qreal bw = tw + pad * 2;
    const qreal bh = fm.height() + pad * 1.2;
    const QRectF box((w - bw) * 0.5, h * 0.045, bw, bh);

    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(QColor(0xC6, 0x28, 0x28, alpha)));
    p.drawRect(box);
    p.setPen(Qt::white);
    p.drawText(box, Qt::AlignCenter, text);
}

/* 实线报警时箭头应该指向"被拒绝的那次请求方向" */
int s_lane_dir_hint(const AdasOverlayState &s)
{
    if (s.lane_request_dir != 0) return s.lane_request_dir;
    return s.led_left ? -1 : +1;
}

} // namespace

void adas_paint_overlay(QPainter &painter, const QSizeF &frameSize,
                        const AdasOverlayState &state)
{
    if (frameSize.width() <= 1.0 || frameSize.height() <= 1.0) return;

    painter.save();
    // 性能：叠加层每帧都要重绘，关闭抗锯齿（几何是矩形/粗线，观感差异很小）
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setRenderHint(QPainter::TextAntialiasing, false);

    /* HDMI 输入未锁定：盖暗幕 + 大字提示，避免满屏雪花（现场切到"复制"模式必现）*/
    if (state.no_signal) {
        painter.fillRect(QRectF(0, 0, frameSize.width(), frameSize.height()),
                         QColor(0, 0, 0, 180));
        QFont fBig = painter.font();
        fBig.setPixelSize(qMax(16, (int)(frameSize.height() / 16.0)));
        fBig.setBold(true);
        painter.setFont(fBig);
        painter.setPen(QColor(255, 214, 64));
        painter.drawText(QRectF(0, frameSize.height() * 0.28, frameSize.width(),
                                frameSize.height() * 0.16),
                         Qt::AlignCenter, QStringLiteral("HDMI 输入无信号 / 未锁定"));
        QFont fSmall = painter.font();
        fSmall.setPixelSize(qMax(11, (int)(frameSize.height() / 32.0)));
        fSmall.setBold(false);
        painter.setFont(fSmall);
        painter.setPen(QColor(240, 240, 240));
        painter.drawText(QRectF(0, frameSize.height() * 0.46, frameSize.width(),
                                frameSize.height() * 0.22),
                         Qt::AlignCenter,
                         QStringLiteral("Windows 请设为「扩展」模式，显示器 2 = 1280x720 @ 60Hz\n"
                                        "（「复制」或「仅电脑屏幕」会停掉 HDMI 输出）"));
    }


    /* 关闭时只给一行提示，不干扰原有画面 */
    if (!state.active) {
        QFont f = painter.font();
        f.setPixelSize(qMax(12, (int)(frameSize.height() / 44.0)));
        f.setBold(false);
        painter.setFont(f);
        painter.setPen(QColor(0xBB, 0xBB, 0xBB, 170));
        painter.drawText(QPointF(frameSize.width() * 0.012,
                                 frameSize.height() * 0.985),
                         QStringLiteral("ADAS 演示已关闭 — 按 空格 开启"));
        painter.restore();
        return;
    }

    /* 方向指示 */
    switch (state.action) {
    case ACT_TURN_LEFT:
        draw_arrow(painter, frameSize, -1, false);
        break;
    case ACT_TURN_RIGHT:
        draw_arrow(painter, frameSize, +1, false);
        break;
    case ACT_GO:
        draw_arrow(painter, frameSize, 0, false);
        break;
    case ACT_STOP:
        draw_stop_badge(painter, frameSize);
        break;
    case ACT_WAIT_GREEN:
        draw_stop_badge(painter, frameSize);
        break;
    case ACT_ALARM_SOLID_LANE:
        /* 实线变道：把"想变的方向"画成被禁止的箭头 */
        draw_arrow(painter, frameSize,
                   s_lane_dir_hint(state), true);
        break;
    case ACT_ALARM_RED_LIGHT:
    case ACT_ALARM_PEDESTRIAN:
        draw_stop_badge(painter, frameSize);
        break;
    case ACT_BRAKE:
        draw_stop_badge(painter, frameSize);
        break;
    default:
        break;
    }

    draw_alarm_banner(painter, frameSize, state);
    draw_lane_bar(painter, frameSize, state);
    draw_led_row(painter, frameSize, state);
    draw_status_panel(painter, frameSize, state);

    painter.restore();
}
