#ifndef OBB_DETECTOR_H
#define OBB_DETECTOR_H

#include <vector>
#include <string>
#include <cstdint>
#include "rknn_api.h"    // 本地拷贝, 来自 rknn_model_zoo/3rdparty/rknpu2/include/

// ---- 检测框结构 ----
struct ObbBox {
    float cx, cy;           // 中心点
    float w, h;             // 宽高
    float angle;            // 弧度
    float confidence;       // 置信度
    int   classId;          // 类别ID
    int   trackId = -1;     // 跨帧跟踪ID，-1表示尚未分配
    std::string plateName;  // 车牌号 (LPRNet 识别结果)
    std::string vehicleLabel; // 综合 YOLO 类别和车牌字符得到的车辆标签
    bool plateRecognizedThisFrame = false; // 本帧是否由 LPRNet 得到有效字符，区别于历史缓存
};

struct DetectResult {
    std::vector<ObbBox> boxes;
    float letterboxScale = 1.0f; // letterbox 缩放系数
    int   letterboxXPad = 0;     // letterbox X方向 padding
    int   letterboxYPad = 0;     // letterbox Y方向 padding
    int   srcWidth = 0;          // 原始图像宽度
    int   srcHeight = 0;         // 原始图像高度
};

// ---- 模型参数 ----
struct ObbModelParams {
    int   numClasses   = 15;       // 类别数 (DOTAv1: 15)
    float boxThreshold = 0.5f;     // 置信度阈值
    float nmsThreshold = 0.4f;     // NMS IoU 阈值
    int   modelWidth   = 640;      // 模型输入宽度
    int   modelHeight  = 640;      // 模型输入高度
};

// ---- YOLOv8 OBB 检测器 ----
// 封装 RKNN 模型加载、预处理、推理、后处理全流程
class ObbDetector
{
public:
    ObbDetector();
    ~ObbDetector();

    // 加载模型
    bool loadModel(const char *modelPath, const char *labelPath,
                   const ObbModelParams &params = ObbModelParams());
    void release();
    bool isLoaded() const { return m_loaded; }

    // 执行检测 (src: RGB888 数据, w×h 原始分辨率)
    bool detect(const uint8_t *srcRGB, int width, int height,
                DetectResult &result);

    // 类别名称
    const char *className(int clsId) const;
    int  numClasses() const { return m_params.numClasses; }

    // 模型信息
    int modelWidth()  const { return m_params.modelWidth; }
    int modelHeight() const { return m_params.modelHeight; }

private:
    // letterbox 预处理
    void letterbox(const uint8_t *src, int srcW, int srcH,
                   uint8_t *dst, float &scale, int &xPad, int &yPad);

    // DFL + OBB 解码 (调 postprocess 模块)
    bool decodeOutputs(float *outputs[4], int *dims[4],
                       const DetectResult &meta,
                       std::vector<ObbBox> &boxes);

    rknn_context   m_ctx;
    rknn_input_output_num m_ioNum;
    rknn_tensor_attr *m_inputAttrs;
    rknn_tensor_attr *m_outputAttrs;

    ObbModelParams m_params;
    bool           m_loaded;
    bool           m_isQuant;

    // 输入缓冲区
    uint8_t       *m_letterboxBuf;     // 640×640×3
    int            m_bufSize;

    // 标签
    std::vector<std::string> m_labels;
};

#endif // OBB_DETECTOR_H
