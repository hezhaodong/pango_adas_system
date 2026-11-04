/* adas_glue.h — 工程数据 → ADAS 感知 的翻译层（新增）
 *
 * 只做数据搬运与几何估算，不含任何决策规则：
 *   - 红/绿灯、斑马线：来自交通灯模型（classId 0=Zebra_Cross 1=R_Signal 2=G_Signal）
 *   - 行人/车辆：来自礼让模型（yolov8_traffic），距离用"框高 + 焦距"粗估
 *   - 车道：来自 YOLOPv2 的 laneGeometry*；虚实线由 adas_lane_type 扫帧判定
 *   - 按键：方向键 → AdasCommand；空格 → 开关
 */
#ifndef ADAS_GLUE_H
#define ADAS_GLUE_H

#include "adas_bridge.h"
#include "adas_lane_type.h"
#include "yolov8_detector.h"

#include <stdint.h>

class AdasGlue
{
public:
    AdasGlue();

    /* code >= 0：按 AdasCommand 处理；code == -1：切换 ADAS 演示模式
     * 返回 true 表示已消费（按键被本模块接管） */
    bool handleKey(int code);

    /* 交通灯 + 斑马线（traffic_signal_crosswalk 模型） */
    void onTraffic(const YoloDetectResult &result);

    /* 行人 + 车辆（yolov8_traffic 礼让模型），names/numClasses 可为空 */
    void onPedestrianAndVehicles(const YoloDetectResult &result,
                                 const char *const *names, int numClasses);

    /* 车道线几何（YOLOPv2） */
    void onLane(const YoloDetectResult &result);

    /* 原始 RGB565 帧，供虚实线判别 */
    void setFrame(const uint8_t *rgb565, int width, int height);

    /* 重新跑一次决策，产出叠加状态 */
    AdasOverlayState refresh();

    /* FPGA 预处理显示模式（0 原图 1 灰度 2 边缘 3 二值），由按键切换后同步 */
    void setDispMode(int mode) { m_bridge.setDispMode(mode); }

    /* HDMI 输入失锁看护（MainWindow 定时读 0x110 后调用）*/
    void setNoSignal(bool b) { m_bridge.setNoSignal(b); }
    int dispMode() const { return m_bridge.state().disp_mode; }

    /* 自动灯色响应开关（默认开） */
    void setAutoDrive(bool on) { m_bridge.setAutoDrive(on); }

    bool enabled() const { return m_bridge.enabled(); }
    AdasOverlayState state() const { return m_bridge.state(); }

private:
    void updateLaneType();

    AdasBridge m_bridge;
    AdasFrameInputs m_in;

    const uint8_t *m_frame = nullptr;
    int m_frameWidth = 0;
    int m_frameHeight = 0;

    bool m_hasLaneGeom = false;
    AdasLaneGeometry m_geom = {0, 0, 0, 0, 0, 0, 0};
    int m_laneType = ADAS_LANE_UNKNOWN;
    AdasLaneFilter m_laneFilter = {{0, 0, 0, 0, 0}, 0, 0};   /* v2 多帧投票（5 帧滑窗） */
    float m_laneConf = 0.0f;
    int m_laneTick = 0;
    uint32_t m_frameCounter = 0;
};

#endif /* ADAS_GLUE_H */
