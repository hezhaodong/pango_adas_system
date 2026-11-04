/* adas_glue.cpp — 工程数据 → ADAS 感知（新增文件） */
#include "adas_glue.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

/* 估算焦距（像素）：1280 宽、约 60° 水平视场 ≈ 1109 px；取整便于现场调 */
constexpr float kFocalPx = 1100.0f;

/* 典型目标真实高度（米），用于"框高反推距离" */
constexpr float kPersonHeightM = 1.70f;
constexpr float kCarHeightM = 1.50f;
constexpr float kTruckHeightM = 3.00f;

/* 超过这个距离的行人不算"前方行人"威胁（学长阈值 warn=15m） */
constexpr float kPersonMaxInterestM = 15.0f;

bool name_contains(const char *name, const char *needle)
{
    if (!name || !needle) return false;
    return strstr(name, needle) != nullptr;
}

/* 只把"离得够近"的目标标记为前方行人/障碍；距离未知时按框高粗估 */
float estimate_distance_m(const YoloBox &box, float real_height_m)
{
    if (box.depthMeters > 0.1f) return box.depthMeters;
    const int h = box.bottom - box.top;
    if (h <= 0) return -1.0f;
    return kFocalPx * real_height_m / (float)h;
}

} // namespace

AdasGlue::AdasGlue()
{
    memset(&m_in, 0, sizeof(m_in));
    m_in.person_distance_m = -1.0f;
    m_in.obstacle_distance_m = -1.0f;
}

bool AdasGlue::handleKey(int code)
{
    if (code == -1) {
        m_bridge.setEnabled(!m_bridge.enabled());
        return true;
    }
    if (code < 0 || code > CMD_GO) return false;
    m_bridge.setCommand((AdasCommand)code);
    return true;
}

void AdasGlue::onTraffic(const YoloDetectResult &result)
{
    bool red = false;
    bool green = false;
    bool crosswalk = false;
    for (const YoloBox &box : result.boxes) {
        if (box.confidence < 0.35f) continue;
        if (box.classId == 1) red = true;
        else if (box.classId == 2) green = true;
        else if (box.classId == 0) crosswalk = true;
    }
    m_in.red_light = red;
    m_in.green_light = green;
    m_in.crosswalk = crosswalk;
    m_in.frame_id = ++m_frameCounter;
    m_bridge.updatePerception(m_in);
}

void AdasGlue::onPedestrianAndVehicles(const YoloDetectResult &result,
                                       const char *const *names, int numClasses)
{
    bool person = false;
    float personDist = -1.0f;
    bool obstacle = false;
    float obstacleDist = -1.0f;

    for (const YoloBox &box : result.boxes) {
        if (box.confidence < 0.35f) continue;
        const char *name = nullptr;
        if (names && box.classId >= 0 && box.classId < numClasses)
            name = names[box.classId];

        const bool is_person = name ? name_contains(name, "person")
                                    : (box.classId == 0);
        bool is_vehicle = false;
        if (name) {
            is_vehicle = name_contains(name, "car")
                      || name_contains(name, "truck")
                      || name_contains(name, "bus")
                      || name_contains(name, "motor")
                      || name_contains(name, "bike")
                      || name_contains(name, "vehicle");
        } else {
            is_vehicle = (box.classId == 2 || box.classId == 3
                          || box.classId == 5 || box.classId == 7);
        }

        if (is_person) {
            const float d = estimate_distance_m(box, kPersonHeightM);
            if (d > 0.0f && d < kPersonMaxInterestM && (!person || d < personDist)) {
                person = true;
                personDist = d;
            }
        } else if (is_vehicle) {
            const float realH = (name && name_contains(name, "truck")) ? kTruckHeightM
                                                                       : kCarHeightM;
            const float d = estimate_distance_m(box, realH);
            if (d > 0.0f && d < kPersonMaxInterestM && (!obstacle || d < obstacleDist)) {
                obstacle = true;
                obstacleDist = d;
            }
        }
    }

    m_in.person_ahead = person;
    m_in.person_distance_m = person ? personDist : -1.0f;
    m_in.obstacle_ahead = obstacle;
    m_in.obstacle_distance_m = obstacle ? obstacleDist : -1.0f;
    m_in.frame_id = ++m_frameCounter;
    m_bridge.updatePerception(m_in);
}

void AdasGlue::onLane(const YoloDetectResult &result)
{
    m_hasLaneGeom = result.laneGeometryValid;
    if (m_hasLaneGeom) {
        m_geom.valid = 1;
        m_geom.top_y = result.laneGeometryTopY;
        m_geom.bottom_y = result.laneGeometryBottomY;
        m_geom.left_top_x = result.laneLeftTopX;
        m_geom.left_bottom_x = result.laneLeftBottomX;
        m_geom.right_top_x = result.laneRightTopX;
        m_geom.right_bottom_x = result.laneRightBottomX;
    } else {
        m_geom.valid = 0;
        m_laneType = ADAS_LANE_UNKNOWN;
        m_laneConf = 0.0f;
        adas_lane_filter_reset(&m_laneFilter);
    }
    m_in.lane_valid = m_hasLaneGeom;
    /* 导航提示（赛题(3)）：yolopv2 给出的道路朝向，只用于显示 */
    if (result.roadDirectionConfidence >= 0.5f)
        m_bridge.setNavHint(result.roadDirection, result.roadDirectionConfidence);
    else
        m_bridge.setNavHint(0, 0.0f);
    m_bridge.updatePerception(m_in);
}

void AdasGlue::setFrame(const uint8_t *rgb565, int width, int height)
{
    m_frame = rgb565;
    m_frameWidth = width;
    m_frameHeight = height;
}

void AdasGlue::updateLaneType()
{
    /* 2026-11-02 演示口径调整：变道报警已删除，这里恢复真实的线型识别
     * （HUD 的"线型"文案仍是固定显示「实虚实」，不随识别结果变化）。
     */
    if (!m_hasLaneGeom || !m_frame || m_frameWidth <= 0 || m_frameHeight <= 0) {
        m_in.lane_valid = false;
        m_in.lane_solid = false;
        m_in.lane_dashed = false;
        m_bridge.setLaneType(ADAS_LANE_UNKNOWN, 0.0f);
        return;
    }

    /* 虚实线扫帧比较贵，每 3 次刷新算一次即可（约 10~20 Hz） */
    if ((m_laneTick++ % 3) == 0) {
        int side = 0;
        float conf = 0.0f;
        const int raw = adas_classify_lane_type(m_frame, m_frameWidth, m_frameHeight,
                                                &m_geom, &side, &conf);
        /* v2：单帧结论先过 5 帧投票，抑制误检/闪断 */
        m_laneType = adas_lane_filter_push(&m_laneFilter, raw);
        m_laneConf = (m_laneType == raw) ? conf : conf * 0.6f;
        m_bridge.setLaneType(m_laneType, m_laneConf);
    }

    m_in.lane_valid = (m_laneType != ADAS_LANE_UNKNOWN);
    m_in.lane_solid = (m_laneType == ADAS_LANE_SOLID);
    m_in.lane_dashed = (m_laneType == ADAS_LANE_DASHED);
}

AdasOverlayState AdasGlue::refresh()
{
    updateLaneType();
    m_bridge.updatePerception(m_in);
    return m_bridge.step();
}
