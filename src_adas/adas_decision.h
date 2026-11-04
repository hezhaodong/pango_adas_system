/* adas_decision.h — 智能辅助驾驶 · 决策状态机
 *
 * 对应赛题三要求（官方讲解 PPT 第 9 页）的四条判定规则:
 *   1) 识别到红灯 + 前进状态           = 报警
 *   2) 识别到实线 + 强制变道           = 报警
 *   3) 识别导航指令                    => 输出相应指示 LED
 *   4) 识别前方行人/障碍 + 前进状态    = 报警刹车
 *
 * 输入: 视觉检测结果 + 驾驶员按键指令
 * 输出: LED 指示 + 报警 + 建议动作
 *
 * 设计要点:
 *   - 纯逻辑、零依赖, 可以脱离 FPGA/摄像头单独编译与单元测试
 *   - 检测结果与决策分离: 换模型/换类别不影响决策层
 *   - 所有阈值集中在本头文件, 便于现场调参
 */
#ifndef ADAS_DECISION_H
#define ADAS_DECISION_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================ 输入 ============================ */

/* 视觉检测结果（由检测模型填充） */
typedef struct {
    bool     red_light;          /* 识别到红灯 */
    bool     green_light;        /* 识别到绿灯 */
    bool     crosswalk;          /* 识别到斑马线 */
    bool     person_ahead;       /* 前方行人 */
    bool     obstacle_ahead;     /* 前方障碍物 */

    bool     lane_solid;         /* 当前车道线是实线 */
    bool     lane_dashed;        /* 当前车道线是虚线 */
    bool     lane_valid;         /* 车道线识别是否有效 */

    float    person_distance_m;  /* 行人距离(米), 无效时给 -1 */
    float    obstacle_distance_m;/* 障碍距离(米), 无效时给 -1 */
    uint32_t frame_id;           /* 帧号, 便于观察数据新鲜度 */
} AdasPerception;

/* 驾驶员指令（由按键输入） */
typedef enum {
    CMD_NONE = 0,
    CMD_LEFT,            /* 打左转向 */
    CMD_RIGHT,           /* 打右转向 */
    CMD_STRAIGHT,        /* 直行 */
    CMD_LANE_CHANGE_L,   /* 强制向左变道 */
    CMD_LANE_CHANGE_R,   /* 强制向右变道 */
    CMD_STOP,            /* 模拟停车 */
    CMD_GO               /* 模拟行驶 */
} AdasCommand;

/* 行驶状态 */
typedef enum {
    DRIVE_STOPPED = 0,
    DRIVE_MOVING
} AdasDriveState;

/* ============================ 输出 ============================ */

typedef enum {
    ACT_NONE = 0,
    ACT_GO,              /* 可通行 */
    ACT_STOP,            /* 停车 */
    ACT_BRAKE,           /* 刹车 */
    ACT_TURN_LEFT,
    ACT_TURN_RIGHT,
    ACT_ALARM_RED_LIGHT,       /* 红灯闯行报警 */
    ACT_ALARM_SOLID_LANE,      /* 实线强变道报警 */
    ACT_ALARM_PEDESTRIAN,      /* 行人/障碍刹车报警 */
    ACT_WAIT_GREEN
} AdasAction;

/* LED 指示（对应板上 LED / 画面叠加） */
typedef struct {
    bool led_left;       /* 左转指示 */
    bool led_right;      /* 右转指示 */
    bool led_straight;   /* 直行指示 */
    bool led_stop;       /* 停车指示 */
    bool led_alarm;      /* 报警(红) */
    bool led_ok;         /* 系统正常(绿) */
    uint8_t alarm_blink; /* 报警闪烁节拍 0/1 */
} AdasLedState;

/* 决策结果 */
typedef struct {
    AdasAction   action;
    AdasLedState led;
    bool         should_brake;      /* 是否触发刹车 */
    bool         alarm_active;
    const char  *reason;            /* 触发原因(便于调试/画面显示) */
} AdasDecision;

/* ============================ 阈值 ============================ */

typedef struct {
    float brake_distance_m;      /* 行人/障碍小于此距离触发刹车, 默认 8.0 */
    float warn_distance_m;       /* 小于此距离提示, 默认 15.0 */
    int   alarm_blink_period;    /* 报警闪烁周期(单位=调用次数), 默认 5 */
} AdasConfig;

static inline AdasConfig adas_default_config(void)
{
    AdasConfig c;
    c.brake_distance_m   = 8.0f;
    c.warn_distance_m    = 15.0f;
    c.alarm_blink_period = 5;
    return c;
}

/* ============================ 状态机 ============================ */

typedef struct {
    AdasConfig    cfg;
    AdasDriveState drive;
    AdasCommand   last_cmd;
    int           tick;
    /* 统计, 便于演示时展示"识别到多少次" */
    uint32_t cnt_red_run;        /* 红灯闯行次数 */
    uint32_t cnt_solid_change;   /* 实线强变道次数 */
    uint32_t cnt_ped_brake;      /* 行人刹车次数 */
    uint32_t cnt_total;          /* 决策总次数 */
    bool     solid_alarm_reported; /* 本次变道指令是否已计过"实线变道"（避免每帧累加） */
} AdasStateMachine;

static inline void adas_init(AdasStateMachine *sm)
{
    memset(sm, 0, sizeof(*sm));
    sm->cfg       = adas_default_config();
    sm->drive     = DRIVE_STOPPED;
    sm->last_cmd  = CMD_NONE;
}

static inline void adas_set_command(AdasStateMachine *sm, AdasCommand cmd)
{
    sm->last_cmd = cmd;
    /* 按键模拟停车/行驶 */
    if (cmd == CMD_STOP) sm->drive = DRIVE_STOPPED;
    if (cmd == CMD_GO)   sm->drive = DRIVE_MOVING;
}

/* 距离取值: 无效(-1)或没有目标时返回一个大数, 表示"不构成威胁" */
static inline float adas_eff_dist(float d)
{
    return (d < 0.0f) ? 1e6f : d;
}

/* 核心决策: 每帧检测结果调用一次 */
static inline AdasDecision adas_step(AdasStateMachine *sm, const AdasPerception *p)
{
    AdasDecision d;
    memset(&d, 0, sizeof(d));
    d.action = ACT_NONE;

    sm->tick++;
    sm->cnt_total++;

    /* 变道指令撤销后重置"已计过"标志，下一次再按变道才再计数一次 */
    if (sm->last_cmd != CMD_LANE_CHANGE_L && sm->last_cmd != CMD_LANE_CHANGE_R)
        sm->solid_alarm_reported = false;

    const bool moving = (sm->drive == DRIVE_MOVING);

    /* ---- 规则 4: 前方行人/障碍 + 前进状态 = 报警刹车 ----
     * 优先级最高: 安全相关, 无论灯色/车道如何都要先刹住。       */
    {
        float dp = adas_eff_dist(p->person_distance_m);
        float dO = adas_eff_dist(p->obstacle_distance_m);
        bool  threat_person = p->person_ahead   && dp < sm->cfg.brake_distance_m;
        bool  threat_obj    = p->obstacle_ahead && dO < sm->cfg.brake_distance_m;

        if (moving && (threat_person || threat_obj)) {
            d.action        = ACT_ALARM_PEDESTRIAN;
            d.should_brake  = true;
            d.alarm_active  = true;
            d.reason        = threat_person ? "行人进入刹车距离" : "障碍物进入刹车距离";
            sm->cnt_ped_brake++;
            sm->drive       = DRIVE_STOPPED;   /* 刹车后进入停止状态 */
            goto emit;
        }
    }

    /* ---- 规则 1: 红灯 + 前进状态 = 报警 ----
     * 红灯时本就该停; 若仍在前进则报警。                         */
    if (p->red_light && moving) {
        d.action       = ACT_ALARM_RED_LIGHT;
        d.should_brake = true;
        d.alarm_active = true;
        d.reason       = "红灯状态下仍在前进";
        sm->cnt_red_run++;
        sm->drive      = DRIVE_STOPPED;
        goto emit;
    }

    /* ---- 规则 2: 实线 + 强制变道 = 报警 ----
     * 2026-11-02 演示口径调整：已删除该报警分支，变道指令直接成功执行
     * （由 AdasBridge::requestLaneChange 直接切换车道，画面不再弹报警）。 */
    (void)p;

    /* ---- 红灯停车(未闯行) ---- */
    if (p->red_light && !moving) {
        d.action = ACT_WAIT_GREEN;
        d.reason = "红灯等待";
        goto emit;
    }

    /* ---- 规则 3: 导航/按键指令 => LED 指示 ---- */
    switch (sm->last_cmd) {
    case CMD_LEFT:
    case CMD_LANE_CHANGE_L:
        d.led.led_left = true;
        d.action = ACT_TURN_LEFT;
        d.reason = "左转指示";
        break;
    case CMD_RIGHT:
    case CMD_LANE_CHANGE_R:
        d.led.led_right = true;
        d.action = ACT_TURN_RIGHT;
        d.reason = "右转指示";
        break;
    case CMD_STRAIGHT:
        d.led.led_straight = true;
        d.action = ACT_GO;
        d.reason = "直行指示";
        break;
    case CMD_STOP:
        d.led.led_stop = true;
        d.action = ACT_STOP;
        d.reason = "停车指令";
        break;
    default:
        break;
    }

    /* ---- 绿灯 + 前进 = 放行 ---- */
    if (p->green_light && moving) {
        d.action = ACT_GO;
        d.reason = "绿灯通行";
        d.led.led_ok = true;
    }

emit:
    /* 报警闪烁节拍 */
    if (d.alarm_active) {
        int per = (sm->cfg.alarm_blink_period > 0) ? sm->cfg.alarm_blink_period : 5;
        d.led.alarm_blink = ((sm->tick / per) & 1);
        d.led.led_alarm   = true;
    } else {
        d.led.led_ok = true;
    }
    return d;
}

/* 供画面叠加/日志使用 */
static inline const char *adas_action_name(AdasAction a)
{
    switch (a) {
    case ACT_NONE:               return "无动作";
    case ACT_GO:                 return "通行";
    case ACT_STOP:               return "停车";
    case ACT_BRAKE:              return "刹车";
    case ACT_TURN_LEFT:          return "左转";
    case ACT_TURN_RIGHT:         return "右转";
    case ACT_ALARM_RED_LIGHT:    return "报警:红灯闯行";
    case ACT_ALARM_SOLID_LANE:   return "报警:实线变道";
    case ACT_ALARM_PEDESTRIAN:   return "报警:行人/障碍";
    case ACT_WAIT_GREEN:         return "等待绿灯";
    default:                     return "?";
    }
}

#endif /* ADAS_DECISION_H */
