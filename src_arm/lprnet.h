#ifndef _LPRNET_H_
#define _LPRNET_H_

#include "rknn_api.h"
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

#define MODEL_HEIGHT 24
#define MODEL_WIDTH 94
#define OUT_ROWS 76
#define OUT_COLS 18

// 车牌字符字典（在lprnet.cc中定义）
extern const std::vector<std::string> plate_code;

// 简化版上下文（匹配成功运行的独立测试版）
typedef struct {
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr *input_attrs;
    rknn_tensor_attr *output_attrs;
    int model_channel;
    int model_width;
    int model_height;
    bool is_quant;
} lprnet_context_t;

// 车牌类别定义（对应YOLOv8的cls_id）
enum PlateType {
    PLATE_BLUE = 0,        // 蓝牌 (7位)
    PLATE_YELLOW_SINGLE,   // 单层黄牌 (7位)
    PLATE_YELLOW_DOUBLE,   // 双层黄牌 (7位)
    PLATE_GREEN,           // 绿牌/新能源 (8位，第3位必须是D/F)
    PLATE_GREEN_LARGE      // 大型新能源车牌 (8位)
};

typedef struct {
    std::string plate_name;
} lprnet_result;

// 初始化
int init_lprnet_model(const char *model_path, lprnet_context_t *app_ctx);

// 释放
int release_lprnet_model(lprnet_context_t *app_ctx);

// 推理：使用 cv::Mat 接口（BGR格式输入，内部预处理）
// 结合规则引擎和YOLO类别进行智能解码
// double_yellow_ctx: 双层黄牌专用模型上下文，为NULL时使用默认模型
// aspect_ratio: 检测框宽高比，仅用于诊断日志；双层黄牌由 plate_type 决定
// save_dir: 保存处理后图像的目录，为NULL时不保存
// frame_id: 视频帧序号，用于保证跨帧兜底按不同帧计数；负数表示每次调用都算新帧
// track_id: 车辆跟踪ID，用于隔离多车牌的跨帧状态；负数表示不区分轨迹
int inference_lprnet_model(lprnet_context_t *app_ctx, cv::Mat &plate_img,
                           lprnet_result *out_result, PlateType plate_type = PLATE_BLUE,
                           lprnet_context_t *double_yellow_ctx = nullptr,
                           float aspect_ratio = 0.0f,
                           const char* save_dir = nullptr,
                           long long frame_id = -1,
                           int track_id = -1);

#endif // _LPRNET_H_
