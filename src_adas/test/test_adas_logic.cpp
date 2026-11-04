/* test_adas_logic.cpp — ADAS 增量逻辑的 PC 侧自测（不需要板子/Qt/RKNN）
 *
 * 覆盖：
 *   1) 学长决策状态机四条规则（经由 AdasBridge 调用，验证我们的桥接没走样）
 *   2) ADAS 关闭时不得累加计数
 *   3) 虚实线判别（合成 RGB565 帧：实线 / 虚线 / 无车道线）
 *
 * 编译（MinGW g++ 或板端 g++ 均可）：
 *   g++ -std=c++17 -O1 -I. test/test_adas_logic.cpp adas_bridge.cpp \
 *       adas_lane_type.cpp -o test_adas_logic
 *   ./test_adas_logic
 */
#include "adas_bridge.h"
#include "adas_lane_type.h"

#include <cstdio>
#include <cstring>
#include <vector>

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        if (cond) {                                                      \
            ++g_pass;                                                    \
            std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__);            \
        } else {                                                         \
            ++g_fail;                                                    \
            std::printf("  [FAIL] " fmt "  (%s:%d)\n", ##__VA_ARGS__,    \
                        __FILE__, __LINE__);                             \
        }                                                                \
    } while (0)

static const char *action_name_of(const AdasOverlayState &st)
{
    return st.action_name ? st.action_name : "?";
}

/* ---------------------------------------------------------------- 决策规则 */
static void test_rule_pedestrian_brake()
{
    std::printf("[规则4] 行人进入刹车距离 + 前进 => 报警刹车\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setCommand(CMD_GO);            /* 驾驶员：前进 */

    AdasFrameInputs in;
    in.person_ahead = true;
    in.person_distance_m = 6.0f;     /* < brake_distance_m(8.0) */
    in.frame_id = 1;
    b.updatePerception(in);

    const AdasOverlayState st = b.step();
    CHECK(st.active, "ADAS 已激活");
    CHECK(st.action == ACT_ALARM_PEDESTRIAN, "动作为行人刹车报警（实际 %s）",
          action_name_of(st));
    CHECK(st.alarm, "报警标志置位");
    CHECK(st.cnt_ped_brake == 1, "行人刹车计数=1（实际 %u）", st.cnt_ped_brake);
    CHECK(strcmp(st.command_name, "前进") == 0, "指令名=前进（实际 %s）",
          st.command_name);
}

static void test_rule_red_light()
{
    std::printf("[规则1] 红灯 + 前进 => 报警\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setCommand(CMD_GO);

    AdasFrameInputs in;
    in.red_light = true;
    b.updatePerception(in);

    const AdasOverlayState st = b.step();
    CHECK(st.action == ACT_ALARM_RED_LIGHT, "动作为红灯闯行报警（实际 %s）",
          action_name_of(st));
    CHECK(st.alarm, "报警标志置位");
    CHECK(st.cnt_red_run == 1, "闯红灯计数=1（实际 %u）", st.cnt_red_run);
    CHECK(st.red_light, "叠加状态带红灯标记");
}

static void test_rule_solid_lane_change()
{
    std::printf("[规则2] 实线 + 强制变道 => 报警\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setLaneType(ADAS_LANE_SOLID, 0.92f);
    b.setCommand(CMD_LANE_CHANGE_L);

    AdasFrameInputs in;
    in.lane_valid = true;
    in.lane_solid = true;
    in.lane_dashed = false;
    b.updatePerception(in);

    const AdasOverlayState st = b.step();
    CHECK(st.action == ACT_ALARM_SOLID_LANE, "动作为实线变道报警（实际 %s）",
          action_name_of(st));
    CHECK(st.alarm, "报警标志置位");
    CHECK(st.led_left, "左侧转向指示仍点亮");
    CHECK(st.cnt_solid_change == 1, "实线变道计数=1（实际 %u）",
          st.cnt_solid_change);
    CHECK(st.lane_type == ADAS_LANE_SOLID, "叠加状态带实线标记");
}

static void test_rule_green_light_go()
{
    std::printf("[规则3/放行] 绿灯 + 前进 => 通行\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setCommand(CMD_GO);

    AdasFrameInputs in;
    in.green_light = true;
    b.updatePerception(in);

    const AdasOverlayState st = b.step();
    CHECK(st.action == ACT_GO, "动作为通行（实际 %s）", action_name_of(st));
    CHECK(st.led_ok, "正常指示灯亮");
    CHECK(!st.alarm, "不报警");
}

static void test_red_light_wait()
{
    std::printf("[红灯等待] 红灯 + 未前进 => 等待绿灯\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    /* 不给 CMD_GO：状态机初始为 DRIVE_STOPPED */

    AdasFrameInputs in;
    in.red_light = true;
    b.updatePerception(in);

    const AdasOverlayState st = b.step();
    CHECK(st.action == ACT_WAIT_GREEN, "动作为等待绿灯（实际 %s）",
          action_name_of(st));
    CHECK(!st.alarm, "不报警");
}

static void test_disabled_no_count()
{
    std::printf("[开关] ADAS 关闭时不得累加任何计数\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(false);

    AdasFrameInputs in;
    in.red_light = true;
    in.person_ahead = true;
    in.person_distance_m = 3.0f;
    b.updatePerception(in);
    b.setCommand(CMD_GO);

    for (int i = 0; i < 10; ++i)
        b.step();

    const AdasOverlayState st = b.state();
    CHECK(!st.active, "关闭状态 active=false");
    CHECK(st.action == ACT_NONE, "关闭状态 action=无动作（实际 %s）",
          action_name_of(st));
    CHECK(st.cnt_red_run == 0 && st.cnt_ped_brake == 0,
          "关闭状态下计数保持 0（red=%u ped=%u）", st.cnt_red_run,
          st.cnt_ped_brake);

    /* 打开后应当立刻按规则动作 */
    b.setEnabled(true);
    const AdasOverlayState on = b.step();
    CHECK(on.active, "打开后 active=true");
    CHECK(on.action == ACT_ALARM_PEDESTRIAN,
          "打开后按规则4报警（实际 %s）", action_name_of(on));
}

/* ---------------------------------------------------------------- 虚实线 */
static void draw_vertical_line(std::vector<uint16_t> &frame, int w, int x,
                               int y0, int y1, int half_w, uint16_t color,
                               int on_rows, int off_rows)
{
    int k = 0;
    for (int y = y0; y <= y1; ++y, ++k) {
        if (off_rows > 0) {
            const int period = on_rows + off_rows;
            if ((k % period) >= on_rows) continue;   /* 虚线缝隙 */
        }
        for (int dx = -half_w; dx <= half_w; ++dx) {
            const int xx = x + dx;
            if (xx >= 0 && xx < w) frame[y * w + xx] = color;
        }
    }
}

static void test_lane_solid()
{
    std::printf("[虚实线] 连续白线 => 实线\n");
    const int W = 320, H = 180;
    std::vector<uint16_t> f(W * H, 0x0000);
    draw_vertical_line(f, W, 100, 20, 160, 1, 0xFFFF, 0, 0);

    AdasLaneGeometry g;
    std::memset(&g, 0, sizeof(g));
    g.valid = 1;
    g.top_y = 20;
    g.bottom_y = 160;
    g.left_top_x = 100;
    g.left_bottom_x = 100;
    g.right_top_x = 220;
    g.right_bottom_x = 220;

    int side = 0;
    float conf = 0.0f;
    const int type = adas_classify_lane_type(
        reinterpret_cast<const uint8_t *>(f.data()), W, H, &g, &side, &conf);
    CHECK(type == ADAS_LANE_SOLID, "判为实线（实际 %d, conf=%.2f）", type, conf);
    CHECK(side == -1, "使用左侧车道线（实际 side=%d）", side);
    CHECK(conf >= 0.85f, "置信度 >=0.85（实际 %.2f）", conf);
}

static void test_lane_dashed()
{
    std::printf("[虚实线] 断续白线 => 虚线\n");
    const int W = 320, H = 180;
    std::vector<uint16_t> f(W * H, 0x0000);
    /* 6 行有线 + 10 行空隙 */
    draw_vertical_line(f, W, 100, 20, 160, 1, 0xFFFF, 6, 10);

    AdasLaneGeometry g;
    std::memset(&g, 0, sizeof(g));
    g.valid = 1;
    g.top_y = 20;
    g.bottom_y = 160;
    g.left_top_x = 100;
    g.left_bottom_x = 100;
    g.right_top_x = 220;
    g.right_bottom_x = 220;

    int side = 0;
    float conf = 0.0f;
    const int type = adas_classify_lane_type(
        reinterpret_cast<const uint8_t *>(f.data()), W, H, &g, &side, &conf);
    CHECK(type == ADAS_LANE_DASHED, "判为虚线（实际 %d, conf=%.2f）", type, conf);
    CHECK(conf > 0.0f, "置信度 >0（实际 %.2f）", conf);
}

static void test_lane_none()
{
    std::printf("[虚实线] 无车道线 => UNKNOWN\n");
    const int W = 320, H = 180;
    std::vector<uint16_t> f(W * H, 0x0000);

    AdasLaneGeometry g;
    std::memset(&g, 0, sizeof(g));   /* valid=0 */
    int side = 0;
    float conf = 0.0f;
    const int type = adas_classify_lane_type(
        reinterpret_cast<const uint8_t *>(f.data()), W, H, &g, &side, &conf);
    CHECK(type == ADAS_LANE_UNKNOWN, "无有效车道线时返回 UNKNOWN（实际 %d)",
          type);
}

/* ------------------------------------------------- 车道切换（赛题测试方式(1)） */
static void test_lane_move_dashed()
{
    std::printf("[车道切换] 虚线/未知时按键 ←/→ 切换虚拟自车车道\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setLaneType(ADAS_LANE_DASHED, 0.9f);

    CHECK(b.lanePos() == 2, "初始在中间车道（实际 %d）", b.lanePos());

    b.setCommand(CMD_LANE_CHANGE_L);
    CHECK(b.lanePos() == 1, "按 ← 后到左车道（实际 %d）", b.lanePos());
    CHECK(!b.step().lane_move_blocked, "虚线变道未被拒绝");

    b.setCommand(CMD_LANE_CHANGE_R);
    b.setCommand(CMD_LANE_CHANGE_R);
    CHECK(b.lanePos() == 3, "按两次 → 后到右车道（实际 %d）", b.lanePos());

    b.setCommand(CMD_LANE_CHANGE_R);
    CHECK(b.lanePos() == 3, "最右车道再按 → 不越界（实际 %d）", b.lanePos());

    const AdasOverlayState st = b.step();
    CHECK(st.lane_pos == 3, "叠加状态里的车道号为 3（实际 %d）", st.lane_pos);
    CHECK(st.lane_request_dir == 1, "记录的请求方向为 +1（实际 %d）",
          st.lane_request_dir);
}

static void test_lane_move_solid_blocked()
{
    std::printf("[车道切换] 实线上按 ← => 车道不动 + 报警\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setLaneType(ADAS_LANE_SOLID, 0.9f);

    AdasFrameInputs in;
    in.lane_valid = true;
    in.lane_solid = true;
    b.updatePerception(in);

    b.setCommand(CMD_LANE_CHANGE_L);
    CHECK(b.lanePos() == 2, "实线上车道不移动（实际 %d）", b.lanePos());

    const AdasOverlayState st = b.step();
    CHECK(st.lane_move_blocked, "标记为“被实线拒绝”");
    CHECK(st.action == ACT_ALARM_SOLID_LANE, "动作为实线变道报警（实际 %s）",
          action_name_of(st));
    CHECK(st.alarm, "报警标志置位");
    CHECK(st.cnt_solid_change == 1, "实线变道计数=1（实际 %u）", st.cnt_solid_change);
}

static void test_lane_move_unknown_allowed()
{
    std::printf("[车道切换] 线型未知时允许移动（否则现场没法演示）\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setLaneType(ADAS_LANE_UNKNOWN, 0.0f);

    b.setCommand(CMD_LANE_CHANGE_R);
    CHECK(b.lanePos() == 3, "未知线型按 → 仍可切到右车道（实际 %d）", b.lanePos());
    CHECK(!b.step().lane_move_blocked, "未知线型不算被拒绝");
}

/* ------------------------------------------------- 赛题(1)：自动灯色响应 */
static void test_auto_red_stop()
{
    std::printf("[自动灯色] 无人按键时红灯亮 → 自动停车\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setAutoDrive(true);
    AdasFrameInputs in;
    in.red_light = true;
    b.updatePerception(in);

    AdasOverlayState st = b.step();
    CHECK(st.command_name && std::strcmp(st.command_name, "停止") == 0,
          "红灯自动停车（实际指令 %s）", st.command_name ? st.command_name : "?");
}

static void test_auto_green_go_after_red()
{
    std::printf("[自动灯色] 因红灯停下的车，绿灯亮 → 自动起步\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setAutoDrive(true);

    AdasFrameInputs in;
    in.red_light = true;
    b.updatePerception(in);
    b.step();                       /* 红灯 → 停车 */

    in = AdasFrameInputs();
    in.green_light = true;
    b.updatePerception(in);
    AdasOverlayState st = b.step();
    CHECK(st.command_name && std::strcmp(st.command_name, "前进") == 0,
          "绿灯后自动前进（实际指令 %s）", st.command_name ? st.command_name : "?");
}

static void test_manual_override_red()
{
    std::printf("[自动灯色] 红灯时驾驶员强制按 ↑ → 仍然报警（不被自动逻辑吞掉）\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    b.setAutoDrive(true);

    AdasFrameInputs in;
    in.red_light = true;
    b.updatePerception(in);
    b.setCommand(CMD_GO);           /* 驾驶员强制前进 */
    AdasOverlayState st = b.step();
    CHECK(st.alarm, "红灯强制前进必须报警");
    CHECK(st.action == ACT_ALARM_RED_LIGHT, "动作=闯红灯报警（实际 %s）", action_name_of(st));
}

static void test_alarm_hold()
{
    std::printf("[报警锁存] 报警消失后仍保持约 1.2 秒，评委能看见\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);

    AdasFrameInputs in;
    in.red_light = true;
    b.updatePerception(in);
    b.setCommand(CMD_GO);
    AdasOverlayState st = b.step();
    CHECK(st.alarm, "第一帧报警");

    /* 灯灭了，再跑 30 帧（0.5s）仍应保持报警 */
    in = AdasFrameInputs();
    b.updatePerception(in);
    int held = 0;
    for (int i = 0; i < 30; ++i) {
        st = b.step();
        if (st.alarm) ++held;
    }
    CHECK(held == 30, "灯灭后 0.5s 内报警仍显示（实际保持 %d 帧）", held);

    /* 再跑 60 帧（累计 1.5s）应已释放 */
    for (int i = 0; i < 60; ++i)
        st = b.step();
    CHECK(!st.alarm, "1.5s 后报警释放");
}

static void test_disp_mode_passthrough()
{
    std::printf("[预处理模式] 0x20C 四模式透传到叠加状态\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);
    for (int m = 0; m < 4; ++m) {
        b.setDispMode(m);
        AdasOverlayState st = b.step();
        CHECK(st.disp_mode == m, "模式 %d 透传（实际 %d）", m, st.disp_mode);
    }
    /* 关闭时也要透传（关闭时面板仍显示当前模式） */
    b.setEnabled(false);
    b.setDispMode(2);
    CHECK(b.step().disp_mode == 2, "关闭时模式仍透传");
}

static void test_nav_hint()
{
    std::printf("[导航] 高置信度道路朝向 → 导航提示；红灯时不提示\n");
    AdasBridge b;
    b.reset();
    b.setEnabled(true);

    b.setNavHint(1, 0.9f);          /* 道路向右 */
    AdasOverlayState st = b.step();
    CHECK(st.nav_hint && st.nav_hint[0], "有导航提示（实际 '%s'）", st.nav_hint ? st.nav_hint : "");

    AdasFrameInputs in;
    in.red_light = true;
    b.updatePerception(in);
    st = b.step();
    CHECK(!st.nav_hint || !st.nav_hint[0], "红灯时不出导航提示");
}


/* ---------------- v2 新增：多帧投票 / 亮度自适应 ---------------- */
static void test_lane_filter_vote()
{
    std::printf("[test] 虚实线多帧投票\n");
    AdasLaneFilter f;
    adas_lane_filter_reset(&f);
    CHECK(adas_lane_filter_push(&f, ADAS_LANE_SOLID) == ADAS_LANE_UNKNOWN, "1 帧不急着出结论");
    CHECK(adas_lane_filter_push(&f, ADAS_LANE_SOLID) == ADAS_LANE_SOLID, "2 票同类提前采纳（降低延迟）");
    CHECK(adas_lane_filter_push(&f, ADAS_LANE_DASHED) == ADAS_LANE_SOLID, "混入 1 帧虚线不改结论");
    CHECK(adas_lane_filter_push(&f, ADAS_LANE_DASHED) == ADAS_LANE_SOLID, "混入 2 帧虚线仍不改结论");
    CHECK(adas_lane_filter_push(&f, ADAS_LANE_DASHED) == ADAS_LANE_DASHED, "虚线反超（3:2）→ 改判虚线");
    CHECK(adas_lane_filter_push(&f, ADAS_LANE_DASHED) == ADAS_LANE_DASHED, "累计 4 帧虚线 → 稳定采纳");

    adas_lane_filter_reset(&f);
    for (int i = 0; i < 5; ++i) adas_lane_filter_push(&f, ADAS_LANE_UNKNOWN);
    CHECK(adas_lane_filter_push(&f, ADAS_LANE_SOLID) == ADAS_LANE_UNKNOWN, "全是 UNKNOWN 时单帧不误判");
}

/* 夜间暗线：把合成帧整体压暗（车道像素灰阶 ~70），验证亮度自适应还能判出实线 */
static void test_lane_solid_dark_adaptive()
{
    std::printf("[test] 夜间暗线（亮度自适应）\n");
    const int W = 320, H = 240;
    std::vector<uint16_t> f(W * H, 0x0000);
    for (int y = 0; y < H; ++y)
        for (int x = 120; x < 124; ++x)
            f[y * W + x] = 0x4200; /* 暗灰（R=8,G=16,B=0 量级） */
    AdasLaneGeometry g = {1, 20, H - 1, 122, 122, 300, 300};
    int side = 0; float conf = 0.0f;
    const int t = adas_classify_lane_type(reinterpret_cast<const uint8_t *>(f.data()),
                                          W, H, &g, &side, &conf);
    CHECK(t == ADAS_LANE_SOLID || t == ADAS_LANE_UNKNOWN, "暗线不误判成虚线（实际 %d）", t);
    CHECK(!(t == ADAS_LANE_DASHED), "暗线不会被判成虚线");
}

/* ---------------------------------------------------------------- main */
int main()
{
    std::printf("==== ADAS 增量逻辑 PC 自测 ====\n");
    test_rule_pedestrian_brake();
    test_rule_red_light();
    test_rule_solid_lane_change();
    test_rule_green_light_go();
    test_red_light_wait();
    test_disabled_no_count();
    test_lane_solid();
    test_lane_dashed();
    test_lane_none();
    test_lane_move_dashed();
    test_lane_move_solid_blocked();
    test_lane_move_unknown_allowed();
    test_auto_red_stop();
    test_auto_green_go_after_red();
    test_manual_override_red();
    test_alarm_hold();
    test_disp_mode_passthrough();
    test_nav_hint();
    test_lane_filter_vote();
    test_lane_solid_dark_adaptive();

    std::printf("\n==== 结果: %d 通过, %d 失败 ====\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
