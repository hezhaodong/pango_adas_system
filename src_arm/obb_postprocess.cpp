// 严格按照 Rockchip rknn_model_zoo postprocess.cc 参考实现
#include "obb_postprocess.h"
#include "obb_detector.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <set>
#include <algorithm>
#include <utility>

namespace ObbPostProcess {

// ---------- 基础工具 ----------
static inline float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}
static inline float unsigmoid(float y) {
    return -logf(1.0f / y - 1.0f);
}
static inline int32_t clip32(float v, float lo, float hi) {
    if (v <= lo) return (int32_t)lo;
    if (v >= hi) return (int32_t)hi;
    return (int32_t)v;
}
static inline int8_t qntF32ToI8(float f32, int32_t zp, float s) {
    return (int8_t)clip32(f32 / s + zp, -128, 127);
}
static inline float deqntI8ToF32(int8_t q, int32_t zp, float s) {
    return ((float)q - (float)zp) * s;
}

static void softmax(float *x, int n) {
    float mx = x[0];
    for (int i = 1; i < n; i++) {
        if (x[i] > mx) mx = x[i];
    }
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        x[i] = expf(x[i] - mx);
        sum += x[i];
    }
    for (int i = 0; i < n; i++) {
        x[i] /= sum;
    }
}

static int quickSortDesc(std::vector<float> &v, int left, int right,
                         std::vector<int> &idx) {
    if (left >= right) return left;
    int low = left, high = right;
    float key = v[left];
    int keyIdx = idx[left];
    while (low < high) {
        while (low < high && v[high] <= key) {
            high--;
        }
        v[low] = v[high];
        idx[low] = idx[high];
        while (low < high && v[low] >= key) {
            low++;
        }
        v[high] = v[low];
        idx[high] = idx[low];
    }
    v[low] = key;
    idx[low] = keyIdx;
    quickSortDesc(v, left, low - 1, idx);
    quickSortDesc(v, low + 1, right, idx);
    return low;
}

// ---------- OBB IoU (参考 Cal_IOU) ----------
static std::vector<float> rbboxToCorners(const float rb[5]) {
    float cx = rb[0] + rb[2] / 2.0f;
    float cy = rb[1] + rb[3] / 2.0f;
    float aCos = cosf(rb[4]), aSin = sinf(rb[4]);
    float dx[4] = {-rb[2]/2, -rb[2]/2, rb[2]/2, rb[2]/2};
    float dy[4] = {-rb[3]/2, rb[3]/2, rb[3]/2, -rb[3]/2};
    std::vector<float> c(8);
    for (int i = 0; i < 4; i++) {
        c[2*i]   = aCos * dx[i] - aSin * dy[i] + cx;
        c[2*i+1] = aSin * dx[i] + aCos * dy[i] + cy;
    }
    return c;
}

static bool ptInQuad(float px, float py, const std::vector<float> &q) {
    float ab0 = q[2] - q[0], ab1 = q[3] - q[1];
    float ad0 = q[6] - q[0], ad1 = q[7] - q[1];
    float ap0 = px - q[0],   ap1 = py - q[1];
    float abab = ab0 * ab0 + ab1 * ab1;
    float abap = ab0 * ap0 + ab1 * ap1;
    float adad = ad0 * ad0 + ad1 * ad1;
    float adap = ad0 * ap0 + ad1 * ap1;
    return abab >= abap && abap >= 0 && adad >= adap && adap >= 0;
}

static bool segInter(const std::vector<float> &p1, const std::vector<float> &p2,
                     int i, int j, float &ix, float &iy) {
    float A[2] = {p1[2*i], p1[2*i+1]};
    float B[2] = {p1[2*((i+1)%4)], p1[2*((i+1)%4)+1]};
    float C[2] = {p2[2*j], p2[2*j+1]};
    float D[2] = {p2[2*((j+1)%4)], p2[2*((j+1)%4)+1]};
    float BA0 = B[0] - A[0], BA1 = B[1] - A[1];
    float DA0 = D[0] - A[0], DA1 = D[1] - A[1];
    float CA0 = C[0] - A[0], CA1 = C[1] - A[1];
    bool acd = DA1 * CA0 > CA1 * DA0;
    bool bcd = (D[1]-B[1]) * (C[0]-B[0]) > (C[1]-B[1]) * (D[0]-B[0]);
    if (acd != bcd) {
        bool abc = CA1 * BA0 > BA1 * CA0;
        bool abd = DA1 * BA0 > BA1 * DA0;
        if (abc != abd) {
            float DC0 = D[0] - C[0], DC1 = D[1] - C[1];
            float ABBA = A[0]*B[1] - B[0]*A[1];
            float CDDC = C[0]*D[1] - D[0]*C[1];
            float DH = BA1*DC0 - BA0*DC1;
            ix = (ABBA*DC0 - BA0*CDDC) / DH;
            iy = (ABBA*DC1 - BA1*CDDC) / DH;
            return true;
        }
    }
    return false;
}

static float calIoU(const float rb1[5], const float rb2[5]) {
    auto c1 = rbboxToCorners(rb1);
    auto c2 = rbboxToCorners(rb2);
    std::vector<std::vector<float>> pts;
    int np = 0;
    for (int i = 0; i < 4; i++) {
        if (ptInQuad(c1[2*i], c1[2*i+1], c2)) {
            pts.push_back({c1[2*i], c1[2*i+1]});
            np++;
        }
    }
    for (int i = 0; i < 4; i++) {
        if (ptInQuad(c2[2*i], c2[2*i+1], c1)) {
            pts.push_back({c2[2*i], c2[2*i+1]});
            np++;
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float ix, iy;
            if (segInter(c1, c2, i, j, ix, iy)) {
                pts.push_back({ix, iy});
                np++;
            }
        }
    }
    if (np < 3) return 0.0f;

    float cx = 0, cy = 0;
    for (auto &p : pts) { cx += p[0]; cy += p[1]; }
    cx /= np; cy /= np;

    std::sort(pts.begin(), pts.end(),
        [&](const std::vector<float> &a, const std::vector<float> &b) {
            float vx1 = a[0] - cx, vy1 = a[1] - cy;
            float vx2 = b[0] - cx, vy2 = b[1] - cy;
            float d1 = sqrtf(vx1*vx1 + vy1*vy1);
            float d2 = sqrtf(vx2*vx2 + vy2*vy2);
            vx1 /= d1; vy1 /= d1;
            vx2 /= d2; vy2 /= d2;
            if (vy1 < 0) vx1 = -2 - vx1;
            if (vy2 < 0) vx2 = -2 - vx2;
            return vx1 < vx2;
        });

    float iArea = 0.0f;
    for (int i = 1; i < np - 1; i++) {
        float a0 = pts[0][0], a1 = pts[0][1];
        float b0 = pts[i][0], b1 = pts[i][1];
        float c0 = pts[i+1][0], c1 = pts[i+1][1];
        iArea += fabsf((a0-c0)*(b1-c1) - (a1-c1)*(b0-c0)) / 2.0f;
    }
    float uArea = rb1[2]*rb1[3] + rb2[2]*rb2[3] - iArea;
    return iArea / (uArea + 1e-6f);
}

static void obbNms(int validCount, std::vector<float> &boxes,
                   std::vector<int> &classIds, std::vector<int> &order,
                   int filterId, float threshold) {
    for (int i = 0; i < validCount; i++) {
        if (order[i] == -1) continue;
        int n = order[i];
        if (classIds[n] != filterId) continue;
        for (int j = i + 1; j < validCount; j++) {
            int m = order[j];
            if (m == -1 || classIds[m] != filterId) continue;
            float r1[5] = {boxes[n*5], boxes[n*5+1], boxes[n*5+2],
                           boxes[n*5+3], boxes[n*5+4]};
            float r2[5] = {boxes[m*5], boxes[m*5+1], boxes[m*5+2],
                           boxes[m*5+3], boxes[m*5+4]};
            if (calIoU(r1, r2) > threshold) {
                order[j] = -1;
            }
        }
    }
}

// ---------- process_i8 (参考完全一致, NCHW布局) ----------
static int processI8(const int8_t *input, const int8_t *angleFeat,
                     int gridH, int gridW, int stride,
                     std::vector<float> &boxes, std::vector<float> &scores,
                     std::vector<int> &classIds, float threshold,
                     int32_t zp, float scale, int32_t angZp, float angScale,
                     int numClasses, int angleOffset) {
    int locLen = 64;
    int validCount = 0;

    // Inspect the raw quantized class range without changing Model Zoo's
    // required logit semantics. This reveals quantization saturation or an
    // incorrect tensor/quantization attribute immediately on the target.
    const int8_t thresI8 = qntF32ToI8(unsigmoid(threshold), zp, scale);

    for (int h = 0; h < gridH; h++) {
        for (int w = 0; w < gridW; w++) {
            for (int a = 0; a < numClasses; a++) {
                // NCHW: input[(locLen + a) * H * W + h * W + w]
                int ci = (locLen + a) * gridH * gridW + h * gridW + w;
                if (input[ci] < thresI8) continue;

                const float rawScore = deqntI8ToF32(input[ci], zp, scale);
                const float clsScore = sigmoid(rawScore);

                float loc[64];
                for (int k = 0; k < locLen; k++) {
                    loc[k] = deqntI8ToF32(
                        input[k * gridH * gridW + h * gridW + w], zp, scale);
                }

                for (int k = 0; k < locLen / 16; k++) {
                    softmax(&loc[k * 16], 16);
                }

                float xywh_[4] = {0, 0, 0, 0};
                for (int dfl = 0; dfl < 16; dfl++) {
                    xywh_[0] += loc[dfl] * dfl;
                    xywh_[1] += loc[16 + dfl] * dfl;
                    xywh_[2] += loc[32 + dfl] * dfl;
                    xywh_[3] += loc[48 + dfl] * dfl;
                }

                float add[2] = {xywh_[0] + xywh_[2], xywh_[1] + xywh_[3]};
                float sub[2] = {(xywh_[2] - xywh_[0]) / 2.0f,
                                (xywh_[3] - xywh_[1]) / 2.0f};

                float af = deqntI8ToF32(
                    angleFeat[angleOffset + h * gridW + w], angZp, angScale);
                af = (af - 0.25f) * 3.1415927410125732f;
                float c = cosf(af), s = sinf(af);

                float cx = (sub[0]*c - sub[1]*s + w + 0.5f) * stride;
                float cy = (sub[0]*s + sub[1]*c + h + 0.5f) * stride;
                float bw = add[0] * stride;
                float bh = add[1] * stride;

                // top-left corner
                boxes.push_back(cx - bw / 2.0f);
                boxes.push_back(cy - bh / 2.0f);
                boxes.push_back(bw);
                boxes.push_back(bh);
                boxes.push_back(af);
                scores.push_back(clsScore);
                classIds.push_back(a);
                validCount++;
            }
        }
    }
    return validCount;
}

// ---------- process_f32 (由 RKNN Runtime 完成解量化) ----------
static int processFloat32(const float *input, const float *angleFeat,
                          int gridH, int gridW, int stride,
                          std::vector<float> &boxes, std::vector<float> &scores,
                          std::vector<int> &classIds, float threshold,
                          int numClasses, int angleOffset)
{
    constexpr int locLen = 64;
    int validCount = 0;
    const float rawThreshold = unsigmoid(threshold);

    for (int h = 0; h < gridH; ++h) {
        for (int w = 0; w < gridW; ++w) {
            for (int a = 0; a < numClasses; ++a) {
                const int ci = (locLen + a) * gridH * gridW + h * gridW + w;
                const float classLogit = input[ci];
                if (classLogit < rawThreshold) continue;

                float loc[locLen];
                for (int k = 0; k < locLen; ++k) {
                    loc[k] = input[k * gridH * gridW + h * gridW + w];
                }
                for (int k = 0; k < locLen / 16; ++k) {
                    softmax(&loc[k * 16], 16);
                }

                float xywh[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                for (int dfl = 0; dfl < 16; ++dfl) {
                    xywh[0] += loc[dfl] * dfl;
                    xywh[1] += loc[16 + dfl] * dfl;
                    xywh[2] += loc[32 + dfl] * dfl;
                    xywh[3] += loc[48 + dfl] * dfl;
                }

                const float addX = xywh[0] + xywh[2];
                const float addY = xywh[1] + xywh[3];
                const float subX = (xywh[2] - xywh[0]) * 0.5f;
                const float subY = (xywh[3] - xywh[1]) * 0.5f;

                float angle = angleFeat[angleOffset + h * gridW + w];
                angle = (angle - 0.25f) * 3.1415927410125732f;
                const float cosAngle = cosf(angle);
                const float sinAngle = sinf(angle);
                const float cx = (subX * cosAngle - subY * sinAngle + w + 0.5f) * stride;
                const float cy = (subX * sinAngle + subY * cosAngle + h + 0.5f) * stride;
                const float width = addX * stride;
                const float height = addY * stride;

                boxes.push_back(cx - width * 0.5f);
                boxes.push_back(cy - height * 0.5f);
                boxes.push_back(width);
                boxes.push_back(height);
                boxes.push_back(angle);
                scores.push_back(sigmoid(classLogit));
                classIds.push_back(a);
                ++validCount;
            }
        }
    }
    return validCount;
}

// ---------- 主入口 ----------
bool decodeInt8(const int8_t *outputs[4], const int *dims[4],
                int numClasses, int32_t zp[4], float scale[4],
                float boxThreshold, float nmsThreshold,
                int modelWidth, int /*modelHeight*/,
                float letterboxScale, int xPad, int yPad,
                int /*srcWidth*/, int /*srcHeight*/,
                std::vector<ObbBox> &outBoxes) {
    outBoxes.clear();

    std::vector<float> filterBoxes, objProbs;
    std::vector<int> classIds;
    int validCount = 0;
    int angleOff = 0;

    for (int i = 0; i < 3; i++) {
        int gH = dims[i][2], gW = dims[i][3];
        int stride = modelWidth / gW;
        validCount += processI8(outputs[i], outputs[3], gH, gW, stride,
                                filterBoxes, objProbs, classIds, boxThreshold,
                                zp[i], scale[i], zp[3], scale[3],
                                numClasses, angleOff);
        angleOff += gH * gW;
    }

    if (validCount <= 0) return false;

    // 排序 (降序)
    std::vector<int> idx(validCount);
    for (int i = 0; i < validCount; i++) idx[i] = i;
    quickSortDesc(objProbs, 0, validCount - 1, idx);

    // 每 30 次解码打印一次真实浮点分数，区分“界面取整”与“模型原始 logit≈0”。
    // NMS (按类别)
    std::set<int> classSet(classIds.begin(), classIds.end());
    for (int c : classSet) {
        obbNms(validCount, filterBoxes, classIds, idx, c, nmsThreshold);
    }

    // 收集结果 + letterbox反算
    int cnt = 0;
    for (int i = 0; i < validCount; i++) {
        if (idx[i] == -1 || cnt >= 128) continue;
        int n = idx[i];

        float x = filterBoxes[n*5 + 0] - (float)xPad;
        float y = filterBoxes[n*5 + 1] - (float)yPad;
        float w = filterBoxes[n*5 + 2];
        float h = filterBoxes[n*5 + 3];
        float ang = filterBoxes[n*5 + 4];

        ObbBox b;
        b.cx = (x + w / 2.0f) / letterboxScale;
        b.cy = (y + h / 2.0f) / letterboxScale;
        b.w  = w / letterboxScale;
        b.h  = h / letterboxScale;
        b.angle = ang;
        b.confidence = objProbs[i];
        b.classId = classIds[n];
        b.trackId = -1;
        outBoxes.push_back(b);
        cnt++;
    }
    return !outBoxes.empty();
}

bool decodeFloat32(const float *outputs[4], const int *dims[4],
                   int numClasses,
                   float boxThreshold, float nmsThreshold,
                   int modelWidth, int /*modelHeight*/,
                   float letterboxScale, int xPad, int yPad,
                   int /*srcWidth*/, int /*srcHeight*/,
                   std::vector<ObbBox> &outBoxes)
{
    outBoxes.clear();

    std::vector<float> filterBoxes, scores;
    std::vector<int> classIds;
    int validCount = 0;
    int angleOffset = 0;
    for (int i = 0; i < 3; ++i) {
        const int gridH = dims[i][2];
        const int gridW = dims[i][3];
        const int stride = modelWidth / gridW;
        validCount += processFloat32(outputs[i], outputs[3], gridH, gridW, stride,
                                     filterBoxes, scores, classIds, boxThreshold,
                                     numClasses, angleOffset);
        angleOffset += gridH * gridW;
    }
    if (validCount <= 0) return false;

    std::vector<int> indices(validCount);
    for (int i = 0; i < validCount; ++i) indices[i] = i;
    quickSortDesc(scores, 0, validCount - 1, indices);

    const std::set<int> classSet(classIds.begin(), classIds.end());
    for (int classId : classSet) {
        obbNms(validCount, filterBoxes, classIds, indices, classId, nmsThreshold);
    }

    int keptCount = 0;
    for (int i = 0; i < validCount; ++i) {
        if (indices[i] == -1 || keptCount >= 128) continue;
        const int sourceIndex = indices[i];
        const float x = filterBoxes[sourceIndex * 5] - static_cast<float>(xPad);
        const float y = filterBoxes[sourceIndex * 5 + 1] - static_cast<float>(yPad);

        ObbBox box;
        box.cx = (x + filterBoxes[sourceIndex * 5 + 2] * 0.5f) / letterboxScale;
        box.cy = (y + filterBoxes[sourceIndex * 5 + 3] * 0.5f) / letterboxScale;
        box.w = filterBoxes[sourceIndex * 5 + 2] / letterboxScale;
        box.h = filterBoxes[sourceIndex * 5 + 3] / letterboxScale;
        box.angle = filterBoxes[sourceIndex * 5 + 4];
        box.confidence = scores[i];
        box.classId = classIds[sourceIndex];
        box.trackId = -1;
        outBoxes.push_back(std::move(box));
        ++keptCount;
    }
    return !outBoxes.empty();
}

} // namespace ObbPostProcess
