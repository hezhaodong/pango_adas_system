/* adas_bridge.cpp — 桥接实现（新增文件） */
#include "adas_bridge.h"
#include "adas_lane_type.h"

#include <string.h>

AdasBridge::AdasBridge()
{
    reset();
}

void AdasBridge::reset()
{
    memset(&m_p, 0, sizeof(m_p));
    memset(&m_last, 0, sizeof(m_last));
    adas_init(&m_sm);
    m_enabled = false;
    m_has_perception = false;
    m_lane_type = 0;
    m_lane_conf = 0.0f;
    m_lane_pos = 2;
    m_move_dir = 0;
    m_move_blocked = false;
    m_auto_drive = true;
    m_stop_by_light = false;
    m_alarm_hold = 0;
    m_hold_action = ACT_NONE;
    m_hold_reason = "";
    m_disp_mode = 0;
    m_nav_dir = 0;
    m_nav_conf = 0.0f;
    m_last.command_name = adas_command_name(CMD_NONE);
    m_last.action_name = adas_action_name(ACT_NONE);
    m_last.reason = "";
    m_last.lane_pos = 2;
}

void AdasBridge::setCommand(AdasCommand cmd)
{
    adas_set_command(&m_sm, cmd);
    applyLaneMove(cmd);
}

void AdasBridge::setLanePos(int pos)
{
    if (pos < 1) pos = 1;
    if (pos > 3) pos = 3;
    m_lane_pos = pos;
}

/* 赛题测试方式(1)：按键模拟切换车道。
 * 规则：虚线/未知 → 允许移动（未知时无法证明是实线，不阻断演示）；
 *       实线      → 不移动，并标记 blocked（画面报红，决策状态机同时报警）。 */
void AdasBridge::applyLaneMove(AdasCommand cmd)
{
    int dir = 0;
    if (cmd == CMD_LANE_CHANGE_L || cmd == CMD_LEFT)
        dir = -1;
    else if (cmd == CMD_LANE_CHANGE_R || cmd == CMD_RIGHT)
        dir = +1;

    m_move_dir = dir;
    m_move_blocked = false;

    if (dir == 0)
        return;

    /* 演示口径（2026-11-02）：变道指令一律直接生效，不再因"实线"拒绝或报警。
     * 目标车道超出 1~3 时保持当前车道（不做额外提示）。 */
    const int next = m_lane_pos + dir;
    if (next >= 1 && next <= 3)
        m_lane_pos = next;
}

void AdasBridge::setLaneType(int type, float conf)
{
    m_lane_type = type;
    m_lane_conf = conf;
}

void AdasBridge::updatePerception(const AdasFrameInputs &in)
{
    /* 学长结构体的字段名是 C 风格，逐项拷贝（不改变语义） */
    m_p.red_light = in.red_light;
    m_p.green_light = in.green_light;
    m_p.crosswalk = in.crosswalk;
    m_p.person_ahead = in.person_ahead;
    m_p.obstacle_ahead = in.obstacle_ahead;
    m_p.person_distance_m = in.person_distance_m;
    m_p.obstacle_distance_m = in.obstacle_distance_m;
    m_p.lane_valid = in.lane_valid;
    m_p.lane_solid = in.lane_solid;
    m_p.lane_dashed = in.lane_dashed;
    m_p.frame_id = in.frame_id;
    m_has_perception = true;
}

AdasOverlayState AdasBridge::step()
{
    AdasOverlayState out;

    if (!m_enabled) {
        /* 关闭时不跑状态机，避免计数被"看不见的帧"累加 */
        out.active = false;
        out.action = ACT_NONE;
        out.action_name = adas_action_name(ACT_NONE);
        out.reason = "";
        out.command_name = adas_command_name(m_sm.last_cmd);
        out.has_perception = m_has_perception;
        out.lane_type = m_lane_type;
        out.lane_confidence = m_lane_conf;
        out.lane_pos = m_lane_pos;
        out.lane_request_dir = m_move_dir;
        out.lane_move_blocked = m_move_blocked;
        out.disp_mode = m_disp_mode;
    out.no_signal = m_no_signal;
        out.nav_hint = "";
        m_last = out;
        return m_last;
    }

    /* ------------------------------------------------------------------
     * 自动灯色响应（赛题(1)）：红灯亮 → 车辆停止；绿灯亮 → 车辆开始行驶。
     * 只在驾驶员没有下达指令时接管；驾驶员一旦按键，就以人的指令为准
     * （此时"红灯 + 前进"才会触发闯红灯报警）。
     * ---------------------------------------------------------------- */
    if (m_auto_drive) {
        const bool red = m_p.red_light;
        const bool green = m_p.green_light && !m_p.red_light;
        if (m_sm.last_cmd == CMD_NONE) {
            if (red) {
                adas_set_command(&m_sm, CMD_STOP);
                m_stop_by_light = true;
            } else if (green) {
                adas_set_command(&m_sm, CMD_GO);
                m_stop_by_light = false;
            }
        } else if (m_stop_by_light && green) {
            /* 因红灯停下的车，绿灯亮后自动起步 */
            adas_set_command(&m_sm, CMD_GO);
            m_stop_by_light = false;
        }
    }

    const AdasDecision d = adas_step(&m_sm, &m_p);

    /* 红灯闯行报警后自动把车停下，并记住"这次停车是灯触发的"，绿灯后自动起步。
     * 行人/障碍刹车不改写驾驶员指令（面板要显示"驾驶员按了前进，但系统刹车"）。 */
    if (d.action == ACT_ALARM_RED_LIGHT) {
        m_stop_by_light = true;
        adas_set_command(&m_sm, CMD_STOP);
    }

    /* 报警锁存 ~1.2 秒（60fps 下 72 帧）：红灯闯行这类报警只持续一帧，
     * 不锁存的话评委根本看不见。 */
    if (d.alarm_active) {
        m_alarm_hold = 72;
        m_hold_action = (int)d.action;
        m_hold_reason = d.reason ? d.reason : "";
    } else if (m_alarm_hold > 0) {
        --m_alarm_hold;
    }

    out.active = true;
    out.alarm = d.alarm_active || m_alarm_hold > 0;
    out.action = (d.alarm_active || m_alarm_hold == 0) ? (int)d.action : m_hold_action;
    out.led_left = d.led.led_left;
    out.led_right = d.led.led_right;
    out.led_straight = d.led.led_straight;
    out.led_stop = d.led.led_stop;
    out.led_alarm = d.led.led_alarm || m_alarm_hold > 0;
    out.led_ok = d.led.led_ok && m_alarm_hold == 0;
    out.blink = d.led.alarm_blink ? 1 : 0;
    out.reason = d.alarm_active ? (d.reason ? d.reason : "")
                                : (m_alarm_hold > 0 ? m_hold_reason : "");
    out.action_name = adas_action_name((AdasAction)out.action);
    out.command_name = adas_command_name(m_sm.last_cmd);
    out.lane_type = m_lane_type;
    out.lane_confidence = m_lane_conf;
    out.lane_pos = m_lane_pos;
    out.lane_request_dir = m_move_dir;
    out.lane_move_blocked = m_move_blocked;
    out.has_perception = m_has_perception;
    out.red_light = m_p.red_light;
    out.green_light = m_p.green_light;
    out.person_distance_m = m_p.person_distance_m;
    out.obstacle_distance_m = m_p.obstacle_distance_m;
    out.cnt_red_run = m_sm.cnt_red_run;
    out.cnt_solid_change = m_sm.cnt_solid_change;
    out.cnt_ped_brake = m_sm.cnt_ped_brake;
    out.disp_mode = m_disp_mode;
    out.no_signal = m_no_signal;

    /* 导航提示：yolopv2 的道路朝向（只有置信度够高才显示） */
    if (m_nav_conf >= 0.5f && !m_p.red_light) {
        if (m_nav_dir < 0)      out.nav_hint = "左";
        else if (m_nav_dir > 0) out.nav_hint = "右";
        else                    out.nav_hint = "直";
    } else {
        out.nav_hint = "";
    }

    m_last = out;
    return m_last;
}

void AdasBridge::setNavHint(int dir, float conf)
{
    m_nav_dir = dir;
    m_nav_conf = conf;
}

const char *adas_command_name(AdasCommand cmd)
{
    switch (cmd) {
    case CMD_LEFT:          return "左转";
    case CMD_RIGHT:         return "右转";
    case CMD_STRAIGHT:      return "直行";
    case CMD_LANE_CHANGE_L: return "强制左变道";
    case CMD_LANE_CHANGE_R: return "强制右变道";
    case CMD_STOP:          return "停止";
    case CMD_GO:            return "前进";
    case CMD_NONE:
    default:                return "无指令";
    }
}
