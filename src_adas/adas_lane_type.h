/* adas_lane_type.h — 虚实线判别（新增，纯 C++，无 Qt/RKNN 依赖）
 *
 * v1 思路：不用新模型。沿着现有车道线输出的左右两条线（top/bottom 坐标插值）
 *         在 RGB565 帧里找白/黄色车道像素，统计"命中/断续"的规律：
 *           断续 ≥2 段 + 命中有间隙        → 虚线
 *           基本连续（命中率 ≥85%，无间隙） → 实线
 * v2 增强（2026-10-22）：
 *   ① 阈值自适应：先算搜索带内的平均亮度，白/黄判据按亮度缩放
 *      （解决夜间偏暗车道线全漏、白天过曝全亮误判）；
 *   ② 多帧投票：AdasLaneFilter 保留最近 5 帧结论，≥3 帧一致才输出
 *      （解决单帧误检/闪断，切换延迟约 5 次判定）。
 */
#ifndef ADAS_LANE_TYPE_H
#define ADAS_LANE_TYPE_H

#include <stdint.h>

enum {
    ADAS_LANE_UNKNOWN = 0,
    ADAS_LANE_DASHED  = 1,
    ADAS_LANE_SOLID   = 2
};

/* 车道线几何：来自 YoloDetectResult 的 laneGeometry* 字段 */
typedef struct {
    int valid;
    int top_y, bottom_y;
    int left_top_x, left_bottom_x;
    int right_top_x, right_bottom_x;
} AdasLaneGeometry;

/* 多帧投票器（v2 新增，纯逻辑，可单测） */
typedef struct {
    int history[5];
    int count;
    int idx;
} AdasLaneFilter;

#ifdef __cplusplus
extern "C" {
#endif

/* rgb565: 帧首地址（width*height*2 字节）；返回 ADAS_LANE_*
 * out_side: -1=用左线判定 +1=用右线判定 0=未判定
 * out_confidence: 0..1
 */
int adas_classify_lane_type(const uint8_t *rgb565, int width, int height,
                            const AdasLaneGeometry *g,
                            int *out_side, float *out_confidence);

void adas_lane_filter_reset(AdasLaneFilter *f);
/* 推入一帧原始结论，返回投票后的结论（>=3/5 一致才采纳，否则 UNKNOWN） */
int  adas_lane_filter_push(AdasLaneFilter *f, int raw_type);

#ifdef __cplusplus
}
#endif

#endif /* ADAS_LANE_TYPE_H */
