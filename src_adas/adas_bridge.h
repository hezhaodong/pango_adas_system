/* adas_bridge.h — 感知输入 → 学长决策状态机 → 叠加状态（新增，纯 C++）
 *
 * 这里不写任何决策规则：规则全部来自学长 07-学长ARM源码/adas/adas_decision.h
 * （原样复制的 adas_decision.h）。本文件只负责：
 *   1) 维护"最新一帧感知"（AdasFrameInputs）
 *   2) 维护"驾驶员按键指令"（AdasCommand）
 *   3) 每次 step() 调一次 adas_step()，并把结果整理成画面叠加需要的结构
 */
#ifndef ADAS_BRIDGE_H
#define ADAS_BRIDGE_H

/* 学长 adas_decision.h 用 `goto emit;` / `emit:` 作跳转标签，
 * 而 Qt 头文件把 `emit` 定义成关键字宏（展开为空）⇒ 在已包含 Qt 头的
 * 编译单元里会变成 `goto ;` / `:` 而编译失败。这里先撤掉该宏再包含。 */
#ifdef emit
#undef emit
#endif
#include "adas_decision.h"

#include <stdint.h>

/* 一帧感知结果（由 adas_glue 从 YoloDetectResult 翻译而来） */
struct AdasFrameInputs {
    bool red_light = false;
    bool green_light = false;
    bool crosswalk = false;
    bool person_ahead = false;
    bool obstacle_ahead = false;
    float person_distance_m = -1.0f;
    float obstacle_distance_m = -1.0f;
    bool lane_valid = false;
    bool lane_solid = false;
    bool lane_dashed = false;
    uint32_t frame_id = 0;
};

/* 画面叠加所需的状态（不含 Qt 类型） */
struct AdasOverlayState {
    bool active = false;      /* ADAS 演示模式是否开启 */
    bool no_signal = false;   /* HDMI 输入未锁定（无信号/模式不对）*/
    bool alarm = false;
    int action = ACT_NONE;    /* AdasAction */
    bool led_left = false;
    bool led_right = false;
    bool led_straight = false;
    bool led_stop = false;
    bool led_alarm = false;
    bool led_ok = false;
    unsigned char blink = 0;
    const char *reason = "";       /* 触发原因（学长原文，UTF-8） */
    const char *action_name = "";  /* adas_action_name() */
    const char *command_name = "";
    int lane_type = 0;        /* ADAS_LANE_* */
    float lane_confidence = 0.0f;
    /* 赛题测试方式(1)："通过按键模拟切换车辆所在的车道"
     * lane_pos: 1=左车道 2=中间车道 3=右车道（虚拟自车所在车道，与画面内容无关） */
    int lane_pos = 2;
    int lane_request_dir = 0;      /* 本次按键请求的方向：-1 向左 +1 向右 0 无 */
    bool lane_move_blocked = false;/* 实线时请求变道被拒绝（画面要报红） */
    bool has_perception = false;
    bool red_light = false;
    bool green_light = false;
    float person_distance_m = -1.0f;
    float obstacle_distance_m = -1.0f;
    unsigned cnt_red_run = 0;
    unsigned cnt_solid_change = 0;
    unsigned cnt_ped_brake = 0;
    /* FPGA 预处理显示模式：0=原图 1=灰度 2=边缘 3=二值（V 键循环） */
    int disp_mode = 0;
    /* 导航提示（yolopv2 roadDirection，仅用于显示，不参与决策） */
    const char *nav_hint = "";
};

class AdasBridge
{
public:
    AdasBridge();

    void reset();

    bool enabled() const { return m_enabled; }
    void setEnabled(bool on) { m_enabled = on; }

    /* HDMI 输入是否失锁（由 MainWindow 定时读 FPGA 0x110 决定）*/
    void setNoSignal(bool b) { m_no_signal = b; }
    bool noSignal() const { return m_no_signal; }

    void setCommand(AdasCommand cmd);                 /* 按键立即生效 */
    AdasCommand command() const { return m_sm.last_cmd; }
    AdasDriveState driveState() const { return m_sm.drive; }

    void setLaneType(int type, float conf);

    /* 虚拟自车所在车道：1=左 2=中 3=右（按键切换，实线时禁止移动） */
    int lanePos() const { return m_lane_pos; }
    void setLanePos(int pos);

    /* 自动灯色响应（赛题(1)：红灯亮 → 车辆停止；绿灯亮 → 车辆开始行驶）
     * 仅在驾驶员没有下达指令（CMD_NONE）时生效，人工指令优先。 */
    void setAutoDrive(bool on) { m_auto_drive = on; }
    bool autoDrive() const { return m_auto_drive; }

    /* FPGA 预处理显示模式（由 MainWindow 按键写入 0x20C 后同步过来） */
    void setDispMode(int m) { m_disp_mode = m; }

    /* 导航方向（yolopv2 roadDirection：<0 左 >0 右 0 直），仅显示 */
    void setNavHint(int dir, float conf);

    void updatePerception(const AdasFrameInputs &in);

    /* 跑一次学长状态机，返回可直接交给叠加层用的状态 */
    AdasOverlayState step();

    AdasOverlayState state() const { return m_last; }

private:
    AdasStateMachine m_sm;
    AdasPerception m_p;
    AdasOverlayState m_last;
    bool m_enabled = false;
    bool m_has_perception = false;
    int m_lane_type = 0;
    float m_lane_conf = 0.0f;
    int m_lane_pos = 2;            /* 虚拟自车车道：默认中间车道 */
    int m_move_dir = 0;            /* 最近一次变道请求方向 */
    bool m_move_blocked = false;   /* 最近一次请求是否被实线拒绝 */
    bool m_auto_drive = true;      /* 无人工指令时按灯色自动停车/起步 */
    bool m_stop_by_light = false;  /* 本次停车是"因红灯"而不是人工停车 */
    int m_alarm_hold = 0;          /* 报警锁存帧数（避免一帧闪过评委看不见） */
    int m_hold_action = ACT_NONE;
    const char *m_hold_reason = "";
    int m_disp_mode = 0;
    int m_nav_dir = 0;
    bool m_no_signal = false;
    float m_nav_conf = 0.0f;

    void applyLaneMove(AdasCommand cmd);
};

const char *adas_command_name(AdasCommand cmd);

#endif /* ADAS_BRIDGE_H */
