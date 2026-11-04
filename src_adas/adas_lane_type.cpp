/* adas_lane_type.cpp — 虚实线判别实现（新增文件，不改学长代码）
 * v2（2026-10-22）：亮度自适应阈值 + 多帧投票
 */
#include "adas_lane_type.h"

#include <stddef.h>
#include <string.h>

namespace {

constexpr int kMaxSamples = 200;      /* 每条线最多采样点 */
constexpr int kSearchHalfPx = 18;     /* 沿插值 x 左右搜索半径 */
constexpr int kMinGapSamples = 3;     /* 断续间隙至少这么长才算"虚线缝" */
constexpr float kMinHitRatio = 0.20f; /* 低于此命中率认为线不可靠 */

/* 亮度自适应系数：白线阈值 = clamp(mean*kWhiteGain, kWhiteLo, kWhiteHi) */
constexpr float kWhiteGain = 1.55f;
constexpr int   kWhiteLo   = 118;
constexpr int   kWhiteHi   = 224;
constexpr float kYellowGain = 1.30f;
constexpr int   kYellowLo   = 118;
constexpr int   kYellowHi   = 240;

inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

inline void rgb565_to_rgb(uint16_t p, int *r, int *g, int *b)
{
    const int r5 = (p >> 11) & 0x1F;
    const int g6 = (p >> 5) & 0x3F;
    const int b5 = p & 0x1F;
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}

inline int luma565(uint16_t p)
{
    int r, g, b;
    rgb565_to_rgb(p, &r, &g, &b);
    return (77 * r + 150 * g + 29 * b) >> 8;
}

/* 白色车道线：亮且低饱和；黄色车道线：亮且 R 明显大于 B。
 * 阈值由调用方按带内平均亮度给出（v2 自适应）。 */
inline bool is_lane_pixel(uint16_t p, int white_min, int yellow_min)
{
    int r, g, b;
    rgb565_to_rgb(p, &r, &g, &b);
    const int maxc = r > g ? (r > b ? r : b) : (g > b ? g : b);
    const int minc = r < g ? (r < b ? r : b) : (g < b ? g : b);
    const bool white = (minc >= white_min) && ((maxc - minc) <= 45);
    const bool yellow = (r >= yellow_min) && (g >= 110) && (b <= 130) && ((r - b) >= 55);
    return white || yellow;
}

struct SideResult {
    int type;
    float confidence;
};

SideResult classify_side(const uint8_t *rgb565, int width, int height,
                         int top_y, int bottom_y, int top_x, int bottom_x)
{
    SideResult out;
    out.type = ADAS_LANE_UNKNOWN;
    out.confidence = 0.0f;

    if (rgb565 == nullptr || width <= 0 || height <= 0) return out;
    if (bottom_y <= top_y + 4) return out;

    top_y = clampi(top_y, 0, height - 1);
    bottom_y = clampi(bottom_y, 0, height - 1);
    if (bottom_y <= top_y + 4) return out;

    const int span = bottom_y - top_y;
    int step = span / kMaxSamples;
    if (step < 1) step = 1;

    const uint16_t *frame = reinterpret_cast<const uint16_t *>(rgb565);

    /* ---- 第一遍：算搜索带内的平均亮度（决定自适应阈值） ---- */
    long luma_sum = 0;
    long luma_cnt = 0;
    {
        int y = bottom_y;
        while (y >= top_y) {
            const int x = top_x + (int)((long)(bottom_x - top_x) * (y - top_y) / span);
            for (int dx = -kSearchHalfPx; dx <= kSearchHalfPx; dx += 3) {
                const int xx = clampi(x + dx, 0, width - 1);
                luma_sum += luma565(frame[y * width + xx]);
                ++luma_cnt;
            }
            y -= step;
        }
    }
    const int mean_luma = (luma_cnt > 0) ? (int)(luma_sum / luma_cnt) : 128;
    const int white_min  = clampi((int)(mean_luma * kWhiteGain), kWhiteLo, kWhiteHi);
    const int yellow_min = clampi((int)(mean_luma * kYellowGain), kYellowLo, kYellowHi);

    /* ---- 第二遍：逐采样点判命中 ---- */
    unsigned char hits[kMaxSamples + 2];
    int n = 0;
    for (int y = bottom_y; y >= top_y && n < kMaxSamples; y -= step) {
        const int x = top_x + (int)((long)(bottom_x - top_x) * (y - top_y) / span);
        int hit = 0;
        for (int dx = -kSearchHalfPx; dx <= kSearchHalfPx && !hit; ++dx) {
            const int xx = clampi(x + dx, 0, width - 1);
            if (is_lane_pixel(frame[y * width + xx], white_min, yellow_min)) hit = 1;
        }
        hits[n++] = (unsigned char)hit;
    }
    if (n <= 0) return out;

    int hitCount = 0;
    int gapRuns = 0;
    int hitRuns = 0;
    int prev = 1; /* 边界按"命中"起算，避免首尾误计 */
    for (int i = 0; i < n; ++i) {
        const int cur = hits[i];
        if (cur) ++hitCount;
        if (i > 0 && cur != prev) {
            if (prev) ++hitRuns; else ++gapRuns;
        }
        prev = cur;
    }
    if (prev) ++hitRuns; else ++gapRuns;

    /* 统计"够长的间隙"，短抖动不算 */
    int longGaps = 0;
    int gapLenTotal = 0;
    {
        int len = 0;
        for (int i = 0; i < n; ++i) {
            if (!hits[i]) {
                ++len;
            } else {
                if (len >= kMinGapSamples) { ++longGaps; gapLenTotal += len; }
                len = 0;
            }
        }
        if (len >= kMinGapSamples) { ++longGaps; gapLenTotal += len; }
    }

    const float hitRatio = (float)hitCount / (float)n;
    if (hitRatio < kMinHitRatio) {
        out.type = ADAS_LANE_UNKNOWN;
        out.confidence = hitRatio;
        return out;
    }

    /* v2：虚线还要求"间隙总长占一定比例"，避免把磨损/遮挡误判成虚线 */
    const float gapRatio = (float)gapLenTotal / (float)n;
    if (longGaps >= 2 && hitRuns >= 3 && gapRatio >= 0.12f) {
        out.type = ADAS_LANE_DASHED;
        out.confidence = 0.45f * hitRatio
                       + 0.35f * (longGaps >= 4 ? 1.0f : (float)longGaps / 4.0f)
                       + 0.20f * (gapRatio > 0.40f ? 1.0f : gapRatio / 0.40f);
    } else if (hitRatio >= 0.85f) {
        out.type = ADAS_LANE_SOLID;
        out.confidence = hitRatio;
    } else if (hitRatio >= 0.70f && longGaps == 0) {
        /* 磨损但连续：仍算实线，置信度降低（夜间常见） */
        out.type = ADAS_LANE_SOLID;
        out.confidence = hitRatio * 0.8f;
    } else {
        out.type = ADAS_LANE_UNKNOWN;
        out.confidence = hitRatio;
    }
    return out;
}

} // namespace

int adas_classify_lane_type(const uint8_t *rgb565, int width, int height,
                            const AdasLaneGeometry *g,
                            int *out_side, float *out_confidence)
{
    if (out_side) *out_side = 0;
    if (out_confidence) *out_confidence = 0.0f;
    if (g == nullptr || !g->valid) return ADAS_LANE_UNKNOWN;

    const SideResult left = classify_side(rgb565, width, height,
                                          g->top_y, g->bottom_y,
                                          g->left_top_x, g->left_bottom_x);
    const SideResult right = classify_side(rgb565, width, height,
                                           g->top_y, g->bottom_y,
                                           g->right_top_x, g->right_bottom_x);

    const SideResult *best = &left;
    int side = -1;
    if (right.confidence > left.confidence) {
        best = &right;
        side = +1;
    }
    if (best->type == ADAS_LANE_UNKNOWN && out_side) {
        *out_side = 0;
        if (out_confidence) *out_confidence = best->confidence;
        return ADAS_LANE_UNKNOWN;
    }
    if (out_side) *out_side = side;
    if (out_confidence) *out_confidence = best->confidence;
    return best->type;
}

/* ---------------- v2 多帧投票 ---------------- */

void adas_lane_filter_reset(AdasLaneFilter *f)
{
    if (!f) return;
    for (int i = 0; i < 5; ++i) f->history[i] = ADAS_LANE_UNKNOWN;
    f->count = 0;
    f->idx = 0;
}

int adas_lane_filter_push(AdasLaneFilter *f, int raw_type)
{
    if (!f) return raw_type;
    f->history[f->idx] = raw_type;
    f->idx = (f->idx + 1) % 5;
    if (f->count < 5) ++f->count;

    int solid = 0, dashed = 0;
    for (int i = 0; i < f->count; ++i) {
        if (f->history[i] == ADAS_LANE_SOLID) ++solid;
        else if (f->history[i] == ADAS_LANE_DASHED) ++dashed;
    }
    /* 明确多数：>=3 票且严格多于对方 */
    if (solid >= 3 && solid > dashed) return ADAS_LANE_SOLID;
    if (dashed >= 3 && dashed > solid) return ADAS_LANE_DASHED;
    /* 平局（2:2）保守取"实线"：宁可少放行变道，也不要在路上误判成虚线 */
    if (solid >= 2 && solid >= dashed) return ADAS_LANE_SOLID;
    if (dashed >= 2 && dashed > solid) return ADAS_LANE_DASHED;
    return ADAS_LANE_UNKNOWN;
}
