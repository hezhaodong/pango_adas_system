#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include <cstdint>

// ---- 图像格式转换工具 ----
// 所有函数均为静态方法，无需实例化
class ImageConverter
{
public:
    // RGB565 (16-bit) → RGB888 (24-bit)
    // src: RGB565 像素数组 (little-endian: [B-low, A-high])
    // dst: RGB888 像素数组 (R, G, B 顺序)
    // 使用位扩展算法消除色阶断层:
    //   5-bit → 8-bit: (c5 << 3) | (c5 >> 2)
    //   6-bit → 8-bit: (g6 << 2) | (g6 >> 4)
    static void rgb565ToRGB888(
        const uint8_t *src,
        uint8_t       *dst,
        int            width,
        int            height
    );

    // RGB565 → 灰度 (8-bit)
    // 使用加权公式: Gray = 0.299*R + 0.587*G + 0.114*B
    static void rgb565ToGray(
        const uint8_t *src,
        uint8_t       *dst,
        int            width,
        int            height
    );

    // 快速 RGB565 → ARGB32 (QImage::Format_RGB32 兼容)
    static void rgb565ToARGB32(
        const uint8_t *src,
        uint8_t       *dst,
        int            width,
        int            height
    );

    // 快速 RGB565 → RGB888 (查找表)
    static void rgb565ToRGB888Fast(
        const uint8_t *src,
        uint8_t       *dst,
        int            width,
        int            height
    );

    // 3×3 中值滤波 (RGB565 格式, in-place)
    // 用于降噪, 边界采用像素复制 (clamp)
    // data: RGB565 原始数据, width × height 像素
    static void medianFilter3x3(
        uint8_t *data,
        int      width,
        int      height
    );

private:
    static bool   s_tableReady;
    static uint8_t s_r5to8[32];
    static uint8_t s_g6to8[64];
    static uint8_t s_b5to8[32];
    static void ensureTable();
};

#endif // IMAGE_CONVERTER_H
