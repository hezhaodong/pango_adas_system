#include "image_converter.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

// ---- 静态成员初始化 ----
bool    ImageConverter::s_tableReady = false;
uint8_t ImageConverter::s_r5to8[32];
uint8_t ImageConverter::s_g6to8[64];
uint8_t ImageConverter::s_b5to8[32];

// ---- 预计算查找表 ----
void ImageConverter::ensureTable()
{
    if (s_tableReady) return;

    for (int i = 0; i < 32; i++) {
        s_r5to8[i] = (uint8_t)((i << 3) | (i >> 2));
        s_b5to8[i] = (uint8_t)((i << 3) | (i >> 2));
    }
    for (int i = 0; i < 64; i++) {
        s_g6to8[i] = (uint8_t)((i << 2) | (i >> 4));
    }

    s_tableReady = true;
}

// ---- 基础版本: 逐像素计算 ----
void ImageConverter::rgb565ToRGB888(
    const uint8_t *src,
    uint8_t       *dst,
    int            width,
    int            height)
{
    int pixelCount = width * height;
    const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);

    for (int i = 0; i < pixelCount; i++) {
        uint16_t px = src16[i];
        uint8_t r5 = (px >> 11) & 0x1F;
        uint8_t g6 = (px >> 5)  & 0x3F;
        uint8_t b5 =  px        & 0x1F;

        uint8_t *d = dst + i * 3;
        d[0] = (r5 << 3) | (r5 >> 2);   // R
        d[1] = (g6 << 2) | (g6 >> 4);   // G
        d[2] = (b5 << 3) | (b5 >> 2);   // B
    }
}

// ---- 快速版本: 查找表 ----
void ImageConverter::rgb565ToRGB888Fast(
    const uint8_t *src,
    uint8_t       *dst,
    int            width,
    int            height)
{
    ensureTable();

    int pixelCount = width * height;
    const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);
    uint8_t *d = dst;

    for (int i = 0; i < pixelCount; i++) {
        uint16_t px = src16[i];
        *d++ = s_r5to8[(px >> 11) & 0x1F];
        *d++ = s_g6to8[(px >> 5)  & 0x3F];
        *d++ = s_b5to8[ px        & 0x1F];
    }
}

// ---- 快速版本: RGB565 → ARGB32 (用于 QImage::Format_RGB32) ----
void ImageConverter::rgb565ToARGB32(
    const uint8_t *src,
    uint8_t       *dst,
    int            width,
    int            height)
{
    ensureTable();

    int pixelCount = width * height;
    const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);
    uint32_t *dst32 = reinterpret_cast<uint32_t *>(dst);

    for (int i = 0; i < pixelCount; i++) {
        uint16_t px = src16[i];
        uint8_t r = s_r5to8[(px >> 11) & 0x1F];
        uint8_t g = s_g6to8[(px >> 5)  & 0x3F];
        uint8_t b = s_b5to8[ px        & 0x1F];
        // ARGB32: A=0xFF, R, G, B (little-endian: B, G, R, A)
        dst32[i] = 0xFF000000 | (r << 16) | (g << 8) | b;
    }
}

// ---- RGB565 转灰度 ----
void ImageConverter::rgb565ToGray(
    const uint8_t *src,
    uint8_t       *dst,
    int            width,
    int            height)
{
    int pixelCount = width * height;
    const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);

    for (int i = 0; i < pixelCount; i++) {
        uint16_t px = src16[i];
        int r8 = ((px >> 11) & 0x1F) << 3;
        int g8 = ((px >> 5)  & 0x3F) << 2;
        int b8 = ( px        & 0x1F) << 3;

        // ITU-R BT.601 亮度公式
        dst[i] = (uint8_t)((r8 * 77 + g8 * 150 + b8 * 29) >> 8);
    }
}

// ---- 3×3 中值滤波 (RGB565, in-place) ----
// 对每个像素取其 3×3 邻域 (含自身), 对 R/G/B 通道分别取中值
// 边界像素采用 clamp 复制 (不缩小图像尺寸)

namespace {

// 取9个 uint8_t 的中值 (插入排序, 9 个元素约 25~36 次比较)
inline uint8_t medianOf9(uint8_t a[9])
{
    // 插入排序 (9 元素)
    for (int i = 1; i < 9; i++) {
        uint8_t key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            --j;
        }
        a[j + 1] = key;
    }
    return a[4];
}

} // namespace

void ImageConverter::medianFilter3x3(
    uint8_t *data,
    int      width,
    int      height)
{
    if (width < 3 || height < 3)
        return;

    const int pixelCount = width * height;
    uint16_t *pixels = reinterpret_cast<uint16_t *>(data);

    // 临时缓冲区存放滤波结果, 避免滤波过程中覆盖原像素
    std::vector<uint16_t> tmp(pixelCount);

    const int stride = width; // 每行像素数

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint8_t rVals[9];
            uint8_t gVals[9];
            uint8_t bVals[9];
            int idx = 0;

            // 收集 3×3 邻域, 超出图像的坐标 clamp 到边界
            for (int dy = -1; dy <= 1; ++dy) {
                int ny = y + dy;
                if (ny < 0)        ny = 0;
                if (ny >= height)  ny = height - 1;

                for (int dx = -1; dx <= 1; ++dx) {
                    int nx = x + dx;
                    if (nx < 0)       nx = 0;
                    if (nx >= width)  nx = width - 1;

                    const uint16_t px = pixels[ny * stride + nx];
                    rVals[idx] = (px >> 11) & 0x1F;   // R: 5-bit
                    gVals[idx] = (px >> 5)  & 0x3F;   // G: 6-bit
                    bVals[idx] =  px        & 0x1F;   // B: 5-bit
                    ++idx;
                }
            }

            const uint8_t rMed = medianOf9(rVals);
            const uint8_t gMed = medianOf9(gVals);
            const uint8_t bMed = medianOf9(bVals);

            tmp[y * stride + x] = static_cast<uint16_t>(
                (rMed << 11) | (gMed << 5) | bMed);
        }
    }

    // 写回原缓冲区
    std::memcpy(data, tmp.data(), static_cast<size_t>(pixelCount) * 2);
}

