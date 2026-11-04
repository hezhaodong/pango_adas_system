#ifndef OBB_POSTPROCESS_H
#define OBB_POSTPROCESS_H

#include <vector>
#include <cstdint>

struct ObbBox;
struct DetectResult;

namespace ObbPostProcess {

// 量化模型后处理: 接收原始 INT8 输出 buffer + 量化参数
bool decodeInt8(const int8_t *outputs[4], const int *dims[4],
                int numClasses,
                int32_t zp[4], float scale[4],
                float boxThreshold, float nmsThreshold,
                int modelWidth, int modelHeight,
                float letterboxScale, int xPad, int yPad,
                int srcWidth, int srcHeight,
                std::vector<ObbBox> &outBoxes);

// RKNN Runtime 已解量化的 FLOAT32 输出后处理。
bool decodeFloat32(const float *outputs[4], const int *dims[4],
                   int numClasses,
                   float boxThreshold, float nmsThreshold,
                   int modelWidth, int modelHeight,
                   float letterboxScale, int xPad, int yPad,
                   int srcWidth, int srcHeight,
                   std::vector<ObbBox> &outBoxes);

} // namespace ObbPostProcess

#endif
