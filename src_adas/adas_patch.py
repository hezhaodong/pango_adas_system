#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""adas_patch.py — 对学长 ARM 工程做“最小插入式”改动（新增 ADAS 增量功能）

用法:
    python3 adas_patch.py [工程目录]      # 默认当前目录

特性:
    * 只做“插入”，不删除/不重写学长任何原有语句
    * 首次运行把原文件备份为 <file>.adas_bak
    * 幂等：已打过的补丁会跳过（用标记字符串检测）
    * 任何锚点找不到就报错退出，绝不“猜着改”
"""
import io
import os
import re
import sys

MARK = "==== ADAS 增量"

# ---------------------------------------------------------------- 工具函数
def read(path):
    """读入并统一成 LF；返回 (文本, 原换行符)"""
    with io.open(path, "rb") as f:
        raw = f.read()
    eol = "\r\n" if b"\r\n" in raw else "\n"
    text = raw.decode("utf-8").replace("\r\n", "\n")
    return text, eol


def write(path, text, eol):
    data = text.replace("\n", eol).encode("utf-8")
    with io.open(path, "wb") as f:
        f.write(data)


def backup_once(path):
    bak = path + ".adas_bak"
    if not os.path.exists(bak):
        with io.open(path, "rb") as src, io.open(bak, "wb") as dst:
            dst.write(src.read())


def patch_file(path, pairs, required=True):
    if not os.path.exists(path):
        if required:
            raise SystemExit("缺少文件: %s" % path)
        return False
    text, eol = read(path)
    if MARK in text:
        print("  [skip] %s 已打过补丁" % os.path.basename(path))
        return False
    original = text
    for old, new in pairs:
        count = text.count(old)
        if count != 1:
            raise SystemExit("锚点匹配 %d 次（期望 1 次）于 %s:\n%r"
                             % (count, os.path.basename(path), old[:120]))
        text = text.replace(old, new, 1)
    backup_once(path)
    write(path, text, eol)
    print("  [ok]   %s (+%d 处)" % (os.path.basename(path), len(pairs)))
    assert text != original
    return True


# ---------------------------------------------------------------- 补丁内容
SOURCES_ANCHOR = "    wifi_control.cpp\n"
SOURCES_NEW = """    wifi_control.cpp \\
    adas_bridge.cpp \\
    adas_lane_type.cpp \\
    adas_glue.cpp \\
    adas_overlay.cpp

# ==== ADAS 增量（新增文件，见 08-ADAS新增源码/README.md）====
"""

HEADERS_ANCHOR = "    wifi_control.h\n"
HEADERS_NEW = """    wifi_control.h \\
    adas_decision.h \\
    adas_bridge.h \\
    adas_lane_type.h \\
    adas_glue.h \\
    adas_overlay.h
"""

EGL_H_INCLUDES_ANCHOR = "#include <functional>\n#include <QByteArray>\n#include <vector>\n"
EGL_H_INCLUDES_NEW = EGL_H_INCLUDES_ANCHOR + "\n// " + MARK + "（新增）\n#include \"adas_bridge.h\"\n"

EGL_H_METHOD_ANCHOR = "    void setSystemRecoveryHandler(std::function<void()> handler);\n"
EGL_H_METHOD_NEW = EGL_H_METHOD_ANCHOR + """
    // ==== ADAS 增量（新增）====
    void setAdasOverlay(const AdasOverlayState &state);
    void clearAdasOverlay();
    void setAdasCommandHandler(std::function<void(int)> handler);
    void setAdasRollHandler(std::function<void()> handler)
    { m_adasRollHandler = handler; }
"""

EGL_H_MEMBER_ANCHOR = "    std::function<void()> m_systemRecoveryHandler;\n"
EGL_H_MEMBER_NEW = EGL_H_MEMBER_ANCHOR + """
    // ==== ADAS 增量（新增）====
    AdasOverlayState m_adasOverlay;
    bool m_adasOverlayValid = false;
    QImage m_adasLayer;
    bool m_adasLayerDirty = true;
    int m_adasLayerTick = 0;
    unsigned m_adasLastSig = 0xFFFFFFFFu;
    std::function<void(int)> m_adasCommandHandler;
    std::function<void()> m_adasRollHandler;
"""

EGL_CPP_INCLUDE_ANCHOR = "#include <QPainter>\n"
EGL_CPP_INCLUDE_NEW = EGL_CPP_INCLUDE_ANCHOR + "\n// " + MARK + "（新增）\n#include \"adas_overlay.h\"\n"

EGL_CPP_PAINTGL_ANCHOR = "void EglfsVideoWindow::paintGL()\n"
EGL_CPP_PAINTGL_NEW = """// ==== ADAS 增量（新增）====
void EglfsVideoWindow::setAdasOverlay(const AdasOverlayState &state)
{
    // 只在"会改变画面内容"的字段变化时重建缓存层，避免每帧重排文字/几何
    const unsigned sig =
        (state.active ? 1u : 0u) | (state.alarm ? 2u : 0u) |
        (state.blink ? 4u : 0u) | ((unsigned)state.lane_pos << 3) |
        ((unsigned)state.action << 8) | (0u) |
        ((unsigned)state.disp_mode << 24) | (state.no_signal ? 0x80000000u : 0u);
    if (sig != m_adasLastSig || state.reason != m_adasOverlay.reason ||
        state.command_name != m_adasOverlay.command_name ||
        state.action_name != m_adasOverlay.action_name ||
        state.nav_hint != m_adasOverlay.nav_hint ||
        state.lane_type != m_adasOverlay.lane_type ||
        state.red_light != m_adasOverlay.red_light ||
        state.green_light != m_adasOverlay.green_light ||
        state.cnt_red_run != m_adasOverlay.cnt_red_run ||
        state.cnt_solid_change != m_adasOverlay.cnt_solid_change ||
        state.cnt_ped_brake != m_adasOverlay.cnt_ped_brake) {
        m_adasLastSig = sig;
        m_adasLayerDirty = true;
    }
    m_adasOverlay = state;
    m_adasOverlayValid = true;
}

void EglfsVideoWindow::clearAdasOverlay()
{
    m_adasOverlayValid = false;
}

void EglfsVideoWindow::setAdasCommandHandler(std::function<void(int)> handler)
{
    m_adasCommandHandler = handler;
}

""" + EGL_CPP_PAINTGL_ANCHOR

EGL_CPP_TRAFFIC_ANCHOR = """        if (m_trafficResult && !m_trafficResult->boxes.empty())
            OverlayRenderer::drawTrafficRects(painter, *m_trafficResult,
                                              m_trafficClassNames, m_trafficNumClasses);
"""
EGL_CPP_TRAFFIC_NEW = EGL_CPP_TRAFFIC_ANCHOR + """        // ==== ADAS 增量（新增）：决策叠加（缓存层 + blit，避免每帧重排文字）====
        if (m_adasOverlayValid) {
            const QSize layerSize(qMax(1, qRound(frameSize.width())),
                                  qMax(1, qRound(frameSize.height())));
            if (m_adasLayer.size() != layerSize)
                m_adasLayer = QImage(layerSize, QImage::Format_ARGB32_Premultiplied);
            if (m_adasLayerDirty || ++m_adasLayerTick >= 24) {
                m_adasLayer.fill(Qt::transparent);
                QPainter layerPainter(&m_adasLayer);
                adas_paint_overlay(layerPainter, frameSize, m_adasOverlay);
                layerPainter.end();
                m_adasLayerDirty = false;
                m_adasLayerTick = 0;
            }
            painter.drawImage(QPointF(0, 0), m_adasLayer);
        }
"""

EGL_CPP_SWITCH_ANCHOR = "    switch (event->key()) {\n    case Qt::Key_Q:\n"
EGL_CPP_SWITCH_NEW = """    switch (event->key()) {
    // ==== ADAS 增量（新增）：方向键 = 驾驶指令，空格 = 开关 ====
    case Qt::Key_Up:
        if (m_adasCommandHandler) m_adasCommandHandler(CMD_GO);
        event->accept();
        return;
    case Qt::Key_Down:
        if (m_adasCommandHandler) m_adasCommandHandler(CMD_STOP);
        event->accept();
        return;
    case Qt::Key_Left:
        if (m_adasCommandHandler) m_adasCommandHandler(CMD_LANE_CHANGE_L);
        event->accept();
        return;
    case Qt::Key_Right:
        if (m_adasCommandHandler) m_adasCommandHandler(CMD_LANE_CHANGE_R);
        event->accept();
        return;
    case Qt::Key_Space:
        if (m_adasCommandHandler) m_adasCommandHandler(-1);
        event->accept();
        return;
    case Qt::Key_M:
        if (m_adasRollHandler) m_adasRollHandler();
        event->accept();
        return;
    case Qt::Key_Q:
"""

MW_H_INCLUDE_ANCHOR = '#include "wifi_control.h"\n'
MW_H_INCLUDE_NEW = MW_H_INCLUDE_ANCHOR + '#include "adas_glue.h"\n'

MW_H_METHOD_ANCHOR = "    void refreshDedicatedDisplayHud();\n"
MW_H_METHOD_NEW = MW_H_METHOD_ANCHOR + """
    // ==== ADAS 增量（新增）====
    void adasPushOverlay();
    void adasHandleKey(int code);
    void adasCheckSignal();
    void adasRollFrame(const QImage &in);
    void adasToggleRoll();

    AdasGlue m_adasGlue;
    QTimer *m_adasSigTimer = nullptr;
    QImage m_adasRollBuf;
    int m_adasRollPx = 0; // 采集端已在 frame_capture.cpp 纠正 256px；此处为显示端应急兜底，M 键可开
"""

MW_CPP_WIRING_ANCHOR = "        m_eglfsVideoWindow->setSystemRecoveryHandler([this]() { restartRkPciePipeline(); });\n"
MW_CPP_WIRING_NEW = MW_CPP_WIRING_ANCHOR + """        // ==== ADAS 增量（新增）：按键 → 决策 ====
        m_eglfsVideoWindow->setAdasCommandHandler([this](int code) { adasHandleKey(code); });
        m_eglfsVideoWindow->setAdasRollHandler([this]() { adasToggleRoll(); });
        // HDMI 失锁看护：0x110(VS 周期) 不在已知值时显示"无信号"提示
        m_adasSigTimer = new QTimer(this);
        connect(m_adasSigTimer, &QTimer::timeout, this, &MainWindow::adasCheckSignal);
        m_adasSigTimer->start(500);
"""

MW_CPP_YOLO_ANCHOR = """void MainWindow::setDisplayYoloOverlay(const YoloDetectResult &result,
                                       const char *const *classNames, int numClasses)
{
"""
MW_CPP_YOLO_NEW = MW_CPP_YOLO_ANCHOR + """    // ==== ADAS 增量（新增）：车道线 -> 决策 + 重绘叠加 ====
    m_adasGlue.onLane(result);
    adasPushOverlay();
"""

MW_CPP_TRAFFIC_ANCHOR = """void MainWindow::setDisplayTrafficOverlay(const YoloDetectResult &result,
                                          const char *const *classNames,
                                          int numClasses)
{
"""
MW_CPP_TRAFFIC_NEW = MW_CPP_TRAFFIC_ANCHOR + """    // ==== ADAS 增量（新增）：红/绿灯、斑马线 -> 决策 ====
    m_adasGlue.onTraffic(result);
"""

MW_CPP_YIELD_ANCHOR = """void MainWindow::setDisplayYieldYoloOverlay(
    const YoloDetectResult &result, const char *const *classNames,
    int numClasses)
{
"""
MW_CPP_YIELD_NEW = MW_CPP_YIELD_ANCHOR + """    // ==== ADAS 增量（新增）：行人/车辆 -> 决策 ====
    m_adasGlue.onPedestrianAndVehicles(result, classNames, numClasses);
"""

MW_CPP_ONUPDATE_ANCHOR = """    if (hasNewDisplayFrame) {
        // Inference callbacks submit overlays only when a result changes.  Do
        // not recopy both result vectors and request extra GL updates.
        presentDisplayFrame(m_cachedDisplayImage);
"""
MW_CPP_ONUPDATE_NEW = """    if (hasNewDisplayFrame) {
        // ==== ADAS 增量（新增）：每个显示帧推进一步决策（闪烁动画也靠它）====
        adasPushOverlay();
        // Inference callbacks submit overlays only when a result changes.  Do
        // not recopy both result vectors and request extra GL updates.
        presentDisplayFrame(m_cachedDisplayImage);
"""

MW_CPP_FUNCS_ANCHOR = "void MainWindow::presentDisplayFrame(const QImage &frame)\n"
MW_CPD_ANCHOR = """void MainWindow::presentDisplayFrame(const QImage &frame)
{
    if (m_eglfsVideoWindow) {
"""
MW_CPD_NEW = """void MainWindow::presentDisplayFrame(const QImage &frame)
{
    // ==== ADAS 增量（新增）：HDMI 采集水平偏移兜底（本板固定 256px 回卷）====
    if (m_adasRollPx > 0 && frame.width() == FRAME_WIDTH &&
        frame.height() == FRAME_HEIGHT && !frame.isNull()) {
        adasRollFrame(frame);
        if (m_eglfsVideoWindow) {
            m_eglfsVideoWindow->presentFrame(m_adasRollBuf);
        } else if (m_videoWidget) {
            m_videoWidget->presentFrame(m_adasRollBuf);
        }
        return;
    }
    if (m_eglfsVideoWindow) {
"""
MW_CPP_FUNCS_NEW = """// ==== ADAS 增量（新增）====
void MainWindow::adasHandleKey(int code)
{
    if (!m_adasGlue.handleKey(code)) return;
    adasPushOverlay();
}

void MainWindow::adasPushOverlay()
{
    m_adasGlue.setFrame(m_lastDisplaySrc, FRAME_WIDTH, FRAME_HEIGHT);
    const AdasOverlayState state = m_adasGlue.refresh();
    if (m_eglfsVideoWindow)
        m_eglfsVideoWindow->setAdasOverlay(state);
}

void MainWindow::adasRollFrame(const QImage &in)
{
    // HDMI 采集固定水平偏移兜底：out[x] = in[x + m_adasRollPx]（行内回卷）
    if (m_adasRollBuf.size() != in.size() || m_adasRollBuf.format() != in.format())
        m_adasRollBuf = QImage(in.size(), in.format());
    const int w = in.width();
    const int bpp = (in.depth() + 7) / 8;
    const int rowBytes = w * bpp;
    const int shiftBytes = (m_adasRollPx % (w > 0 ? w : 1)) * bpp;
    for (int y = 0; y < in.height(); ++y) {
        const uchar *src = in.constScanLine(y);
        uchar *dst = m_adasRollBuf.scanLine(y);
        if (shiftBytes > 0 && shiftBytes < rowBytes) {
            memcpy(dst, src + shiftBytes, rowBytes - shiftBytes);
            memcpy(dst + (rowBytes - shiftBytes), src, shiftBytes);
        } else {
            memcpy(dst, src, rowBytes);
        }
    }
}

void MainWindow::adasToggleRoll()
{
    m_adasRollPx = (m_adasRollPx == 0) ? 256 : 0;
    m_adasRollBuf = QImage();
    qWarning("[adas-roll] roll_px=%d", m_adasRollPx);
}

void MainWindow::adasCheckSignal()
{
    // 0x110 = dbg_vs_period；0x12E1FC=720p60, 0x25C3F8=1080p60, 0x1B74F0=1280x1024
    if (!m_device || !m_device->isOpened()) {
        m_adasGlue.setNoSignal(false);
        return;
    }
    const unsigned int vs = m_device->pioReadReg(0x110);
    const bool ok = (vs == 0x0012E1FCu || vs == 0x0025C3F8u || vs == 0x001B74F0u);
    m_adasGlue.setNoSignal(!ok);
}

""" + MW_CPP_FUNCS_ANCHOR


FC_HELPER_ANCHOR = "static uint32_t sampledFrameSignature(const uint8_t *frame)\n"
FC_HELPER_NEW = """// ==== ADAS 增量（新增）：HDMI 采集 256px 行内回卷纠偏 ====
// 本板 HDMI 采集的线性帧起点比真实画面右移 256px（跨行连续、行内回卷）。
// 在“提交到共享 FrameBuffer 之前”按行左回卷，使显示链路与推理链路看到同一张
// 正确画面，从而消除“视频已纠正、检测框/车道线仍按原始坐标绘制”的偏移。
static void adasRotateRgb565RowsLeft(uint8_t *frame, int shiftPixels)
{
    if (!frame) return;
    shiftPixels %= FRAME_WIDTH;
    if (shiftPixels <= 0) return;
    const size_t rowBytes = static_cast<size_t>(FRAME_WIDTH) * 2u;
    const size_t shiftBytes = static_cast<size_t>(shiftPixels) * 2u;
    std::vector<uint8_t> scratch(shiftBytes);
    for (int y = 0; y < FRAME_HEIGHT; ++y) {
        uint8_t *row = frame + static_cast<size_t>(y) * rowBytes;
        std::memcpy(scratch.data(), row, shiftBytes);
        std::memmove(row, row + shiftBytes, rowBytes - shiftBytes);
        std::memcpy(row + rowBytes - shiftBytes, scratch.data(), shiftBytes);
    }
}

""" + FC_HELPER_ANCHOR

FC_CALL_ANCHOR = "        SeamEstimate seam;\n"
FC_CALL_NEW = """        // ==== ADAS 增量（新增）：HDMI 采集 256px 行内回卷（显示/推理共用同一纠正）====
        if (activeFrameSource == FpgaInputSource::HDMI) {
            const char *rollEnv = std::getenv("HDMI_ROLL_PX");
            const int hdmiRollPx = rollEnv ? std::atoi(rollEnv) : 256;
            adasRotateRgb565RowsLeft(dst, hdmiRollPx);
        }

""" + FC_CALL_ANCHOR

FC_FAST_ANCHOR = "                __sync_synchronize();\n                m_buffer->commitFrame(m_frameSeq);\n"
FC_FAST_NEW = """                // ==== ADAS 增量（新增）：HDMI 快通道（RGA/宽拷贝）commit 前回卷 ====
                // 该分支是 copy_mode=5 的常用路径，必须在此纠正，否则显示端与推理端
                // 读到的仍是原始右移 256px 的帧。
                if (activeFrameSource == FpgaInputSource::HDMI) {
                    const char *rollEnvFast = std::getenv("HDMI_ROLL_PX");
                    const int hdmiRollPxFast = rollEnvFast ? std::atoi(rollEnvFast) : 256;
                    adasRotateRgb565RowsLeft(dst, hdmiRollPxFast);
                }
""" + FC_FAST_ANCHOR


def force_display_roll_zero(path):
    # 已打过补丁的工程里，把显示端默认回卷从 256 改成 0（采集端已纠正，避免二次回卷）
    if not os.path.exists(path):
        return False
    text, eol = read(path)
    new = text.replace("int m_adasRollPx = 256;", "int m_adasRollPx = 0;")
    if new == text:
        print("  [skip] %s 显示端回卷已为 0" % os.path.basename(path))
        return False
    write(path, new, eol)
    print("  [ok]   %s 显示端回卷默认 256 -> 0" % os.path.basename(path))
    return True


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else "."
    root = os.path.abspath(root)
    if not os.path.isdir(root):
        raise SystemExit("不是目录: %s" % root)
    print("工程目录: %s" % root)

    patch_file(os.path.join(root, "camera_display.pro"),
               [(SOURCES_ANCHOR, SOURCES_NEW),
                (HEADERS_ANCHOR, HEADERS_NEW)])

    patch_file(os.path.join(root, "eglfs_video_window.h"),
               [(EGL_H_INCLUDES_ANCHOR, EGL_H_INCLUDES_NEW),
                (EGL_H_METHOD_ANCHOR, EGL_H_METHOD_NEW),
                (EGL_H_MEMBER_ANCHOR, EGL_H_MEMBER_NEW)])

    patch_file(os.path.join(root, "eglfs_video_window.cpp"),
               [(EGL_CPP_INCLUDE_ANCHOR, EGL_CPP_INCLUDE_NEW),
                (EGL_CPP_PAINTGL_ANCHOR, EGL_CPP_PAINTGL_NEW),
                (EGL_CPP_TRAFFIC_ANCHOR, EGL_CPP_TRAFFIC_NEW),
                (EGL_CPP_SWITCH_ANCHOR, EGL_CPP_SWITCH_NEW)])

    patch_file(os.path.join(root, "mainwindow.h"),
               [(MW_H_INCLUDE_ANCHOR, MW_H_INCLUDE_NEW),
                (MW_H_METHOD_ANCHOR, MW_H_METHOD_NEW)])

    patch_file(os.path.join(root, "mainwindow.cpp"),
               [(MW_CPP_WIRING_ANCHOR, MW_CPP_WIRING_NEW),
                (MW_CPP_YOLO_ANCHOR, MW_CPP_YOLO_NEW),
                (MW_CPP_TRAFFIC_ANCHOR, MW_CPP_TRAFFIC_NEW),
                (MW_CPP_YIELD_ANCHOR, MW_CPP_YIELD_NEW),
                (MW_CPP_ONUPDATE_ANCHOR, MW_CPP_ONUPDATE_NEW),
                (MW_CPP_FUNCS_ANCHOR, MW_CPP_FUNCS_NEW),
               (MW_CPD_ANCHOR, MW_CPD_NEW),
                ("#include <unistd.h>\n", "#include <unistd.h>\n#include <string.h>\n") ])

    patch_file(os.path.join(root, "frame_capture.cpp"),
               [(FC_HELPER_ANCHOR, FC_HELPER_NEW),
                (FC_CALL_ANCHOR, FC_CALL_NEW),
                (FC_FAST_ANCHOR, FC_FAST_NEW)])

    force_display_roll_zero(os.path.join(root, "mainwindow.h"))

    print("补丁完成。下一步： qmake camera_display.pro -o Makefile && make -j4")


if __name__ == "__main__":
    main()
