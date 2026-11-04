#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include "lprnet.h"
#include "opencv2/opencv.hpp"

// Normal ARM_final builds suppress the legacy per-character LPR trace stream.
// Define ARM_FINAL_LPR_DIAGNOSTICS only when that detailed trace is required.
#ifndef ARM_FINAL_LPR_DIAGNOSTICS
#define printf(...) do { } while (false)
#endif

namespace {

constexpr int kTemporalFallbackFrames = 5;

struct TemporalFallbackState {
    int plate_type = -1;
    int consecutive_invalid_frames = 0;
    long long last_frame_id = -1;
    std::string best_plate;
    std::string best_source_plate;
    std::string best_repeated_char;
    int best_duration = -1;
    float best_probability = -1.0f;
};

struct TemporalFallbackKey {
    const lprnet_context_t *app_ctx;
    int track_id;

    bool operator==(const TemporalFallbackKey &other) const
    {
        return app_ctx == other.app_ctx && track_id == other.track_id;
    }
};

struct TemporalFallbackKeyHash {
    size_t operator()(const TemporalFallbackKey &key) const
    {
        const size_t context_hash = std::hash<const lprnet_context_t *>()(key.app_ctx);
        const size_t track_hash = std::hash<int>()(key.track_id);
        return context_hash ^ (track_hash + 0x9e3779b9u + (context_hash << 6) +
                               (context_hash >> 2));
    }
};

std::mutex g_temporal_fallback_mutex;
std::unordered_map<TemporalFallbackKey, TemporalFallbackState,
                   TemporalFallbackKeyHash> g_temporal_fallback_states;

static void clear_temporal_fallback_state(const lprnet_context_t *app_ctx)
{
    std::lock_guard<std::mutex> lock(g_temporal_fallback_mutex);
    for (auto it = g_temporal_fallback_states.begin();
         it != g_temporal_fallback_states.end(); ) {
        if (it->first.app_ctx == app_ctx)
            it = g_temporal_fallback_states.erase(it);
        else
            ++it;
    }
}

static void reset_temporal_fallback_state(TemporalFallbackState &state, PlateType plate_type)
{
    state = TemporalFallbackState();
    state.plate_type = static_cast<int>(plate_type);
}

} // namespace

const std::vector<std::string> plate_code{

    "京", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑",
    "苏", "浙", "皖", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤",
    "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁", "新",

    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",

    "A", "B", "C", "D", "E", "F", "G", "H", "J", "K",
    "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V",
    "W", "X", "Y", "Z", "I", "O",

    "-", "学", "警", "港", "澳", "领","临","挂"

};
static int read_data_from_file(const char *path, char **out_data)
{
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "fopen %s fail!\n", path);
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    int file_size = ftell(fp);
    char *data = (char *)malloc(file_size);
    if (data == NULL) {
        fprintf(stderr, "malloc fail!\n");
        fclose(fp);
        return -1;
    }
    fseek(fp, 0, SEEK_SET);
    if (fread(data, 1, file_size, fp) != file_size) {
        fprintf(stderr, "fread %s fail!\n", path);
        free(data);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    *out_data = data;
    return file_size;
}

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, n_elems=%d, size=%d, zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->n_elems, attr->size, attr->zp, attr->scale);
}

int init_lprnet_model(const char *model_path, lprnet_context_t *app_ctx)
{
    clear_temporal_fallback_state(app_ctx);

    int ret;
    int model_len = 0;
    char *model = NULL;
    rknn_context ctx = 0;

    model_len = read_data_from_file(model_path, &model);
    if (model_len < 0 || model == NULL) {
        fprintf(stderr, "load_model fail! path=%s\n", model_path);
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0) {
        fprintf(stderr, "rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC) {
        fprintf(stderr, "rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    printf("input tensors:\n");
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            fprintf(stderr, "rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            fprintf(stderr, "rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(output_attrs[i]));
    }

    app_ctx->rknn_ctx = ctx;
    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        printf("model is NCHW input fmt\n");
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[3];
    } else {
        printf("model is NHWC input fmt\n");
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[1];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    return 0;
}

int release_lprnet_model(lprnet_context_t *app_ctx)
{
    clear_temporal_fallback_state(app_ctx);

    if (app_ctx->input_attrs != NULL) {
        free(app_ctx->input_attrs);
        app_ctx->input_attrs = NULL;
    }
    if (app_ctx->output_attrs != NULL) {
        free(app_ctx->output_attrs);
        app_ctx->output_attrs = NULL;
    }
    if (app_ctx->rknn_ctx != 0) {
        rknn_destroy(app_ctx->rknn_ctx);
        app_ctx->rknn_ctx = 0;
    }
    return 0;
}

// 处理双层黄牌分割（仿照Python切割算法）
static cv::Mat process_double_yellow_plate(const cv::Mat& img) {
    int h = img.rows;
    // 分割上下半部分（上半部分固定10像素）
    cv::Mat upper = img(cv::Range(0, 10), cv::Range::all());
    cv::Mat lower = img(cv::Range(10, h), cv::Range::all());
    
    // ===== 处理上半部分 =====
    int upper_w = upper.cols;
    int left_cut = int(upper_w * 0.2);
    int right_cut = int(upper_w * 0.8);
    cv::Mat upper_cropped = upper(cv::Range::all(), cv::Range(left_cut, right_cut));

    int upper_target_width = 29;  // 改为29，使29+65=94
    cv::Mat upper_processed;
    cv::resize(upper_cropped, upper_processed, cv::Size(upper_target_width, 10));
    
    // ===== 处理下半部分 =====
    int lower_new_width = int(lower.cols * 0.7);
    int lower_new_height = int(lower.rows * 0.7);
    cv::Mat lower_resized;
    cv::resize(lower, lower_resized, cv::Size(lower_new_width, lower_new_height));
    
    int lower_target_width = 65;
    int lower_target_height = 20;
    cv::Mat lower_processed;
    cv::resize(lower_resized, lower_processed, cv::Size(lower_target_width, lower_target_height));
    
    // ===== 调整高度到24像素 =====
    int final_height = 24;
    cv::Mat upper_for_combine;
    cv::Mat lower_for_combine;
    cv::resize(upper_processed, upper_for_combine, cv::Size(upper_target_width, final_height));
    cv::resize(lower_processed, lower_for_combine, cv::Size(lower_target_width, final_height));
    
    // ===== 水平拼接 =====
    cv::Mat combined;
    cv::hconcat(upper_for_combine, lower_for_combine, combined);
    printf("[process_double_yellow_plate] Combined size: %dx%d\n", combined.cols, combined.rows);
    
    // 确保最终尺寸为94x24（必须resize，因为26+65=91≠94）
    cv::Mat result;
    cv::resize(combined, result, cv::Size(94, 24));
    printf("[process_double_yellow_plate] Output size: %dx%d\n", result.cols, result.rows);
    
    return result;
}

// 内部推理函数（复用）
static int do_inference(lprnet_context_t *app_ctx, cv::Mat &input_img, lprnet_result *out_result) {
    int ret;
    rknn_input inputs[1];
    rknn_output outputs[1];

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    cv::Mat resized;
    if (input_img.cols != 94 || input_img.rows != 24) {
        cv::resize(input_img, resized, cv::Size(94, 24), 0, 0, cv::INTER_CUBIC);
    } else {
        resized = input_img;
    }

    if (!resized.isContinuous()) {
        resized = resized.clone();
    }

    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = 94 * 24 * 3;
    inputs[0].buf = resized.data;

    ret = rknn_inputs_set(app_ctx->rknn_ctx, 1, inputs);
    if (ret < 0) {
        fprintf(stderr, "rknn_input_set fail! ret=%d\n", ret);
        return -1;
    }

    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0) {
        fprintf(stderr, "rknn_run fail! ret=%d\n", ret);
        return -1;
    }

    outputs[0].want_float = 1;
    ret = rknn_outputs_get(app_ctx->rknn_ctx, 1, outputs, NULL);
    if (ret < 0) {
        fprintf(stderr, "rknn_outputs_get fail! ret=%d\n", ret);
        return ret;
    }

    //=========================================================================
    // 改进的CTC解码：检测重复字符（基于18个时间步的持续时间）
    //=========================================================================
    
    struct CharNode {
        int char_idx;      // 字符索引
        int duration;      // 在18个时间步中持续的长度（关键）
        float max_prob;    // 最大概率
    };
    
    std::vector<CharNode> nodes;
    int current_idx = -1;
    int current_dur = 0;
    float current_max_prob = 0.0f;
    
    // 第一遍扫描：统计每个字符的持续时间
    for (int x = 0; x < OUT_COLS; x++) {
        float *ptr = (float *)outputs[0].buf;
        float preb[OUT_ROWS];
        
        // 保持原有内存访问方式
        for (int y = 0; y < OUT_ROWS; y++) {
            preb[y] = ptr[x];
            ptr += OUT_COLS;
        }
        
        int max_idx = std::max_element(preb, preb + OUT_ROWS) - preb;
        float max_val = preb[max_idx];
        
        // 跳过blank（最后一类）
        if (max_idx == OUT_ROWS - 1) {
            if (current_idx != -1) {
                nodes.push_back({current_idx, current_dur, current_max_prob});
                current_idx = -1;
                current_dur = 0;
                current_max_prob = 0.0f;
            }
            continue;
        }
        
        if (max_idx == current_idx) {
            // 相同字符，持续时间+1
            current_dur++;
            if (max_val > current_max_prob) current_max_prob = max_val;
        } else {
            // 不同字符，保存旧的，开始新的
            if (current_idx != -1) {
                nodes.push_back({current_idx, current_dur, current_max_prob});
            }
            current_idx = max_idx;
            current_dur = 1;
            current_max_prob = max_val;
        }
    }
    // 保存最后一个
    if (current_idx != -1) {
        nodes.push_back({current_idx, current_dur, current_max_prob});
    }
    
    // 第二遍：根据持续时间决定是否重复输出
    out_result->plate_name.clear();
    printf("[LPRNet] CTC nodes: %zu\n", nodes.size());

    for (auto& node : nodes) {
        int repeat = 1;
        
        // 核心逻辑：根据持续时间判断重复次数
        // 在18个时间步中，每个字符通常占2-3个时间步
        // 提高阈值：≥6才认为是重复，避免绿牌D/F被误判（因分隔符导致持续时间较长）
        if (node.duration >= 6 && node.duration <= 8) {
            repeat = 2;  // 可能是"HH"或"88"
        } else if (node.duration >= 9 && node.duration <= 11) {
            repeat = 3;  // 可能是"888"
        } else if (node.duration >= 12) {
            repeat = 4;  // 可能是"8888"
        }
        
        // 限制总长度
        if (out_result->plate_name.length() + repeat > 8) {
            repeat = 8 - out_result->plate_name.length();
            if (repeat < 1) repeat = 1;
        }
        
        // 输出
        for (int i = 0; i < repeat; i++) {
            out_result->plate_name += plate_code[node.char_idx];
        }

        printf("  idx=%d %s x%d (dur=%d)\n", node.char_idx, plate_code[node.char_idx].c_str(), repeat, node.duration);
    }

    rknn_outputs_release(app_ctx->rknn_ctx, 1, outputs);
    return 0;
}

static int count_chars_utf8(const std::string &s)
{
    int count = 0;
    for (size_t i = 0; i < s.length(); ) {
        unsigned char c = static_cast<unsigned char>(s[i]);
        if ((c & 0x80) == 0) { i++; }
        else if ((c & 0xE0) == 0xC0) { i += 2; }
        else if ((c & 0xF0) == 0xE0) { i += 3; }
        else if ((c & 0xF8) == 0xF0) { i += 4; }
        else { i++; }
        count++;
    }
    return count;
}

static bool is_valid_plate_text(const std::string &plate_name, int expected_len)
{
    if (plate_name.empty()) {
        return false;
    }

    // 领事馆牌照固定按 7 位校验；例如“沪22602领”的第二位允许为数字。
    // 其他车牌继续使用 YOLO 颜色类别给出的期望长度。
    const bool is_consular = plate_name.find("领") != std::string::npos;
    const int required_len = is_consular ? 7 : expected_len;
    if (count_chars_utf8(plate_name) != required_len) {
        return false;
    }

    unsigned char first_byte = static_cast<unsigned char>(plate_name[0]);
    if ((first_byte & 0xF0) != 0xE0) {
        return false;
    }

    if (plate_name.length() < 4) {
        return false;
    }

    const char second_char = plate_name[3];
    const bool is_letter = second_char >= 'A' && second_char <= 'Z';
    const bool is_digit = second_char >= '0' && second_char <= '9';
    return is_letter || (is_consular && is_digit);
}

static bool update_temporal_fallback(lprnet_context_t *app_ctx,
                                     PlateType plate_type,
                                     long long frame_id,
                                     int track_id,
                                     bool current_result_valid,
                                     const std::string &candidate_plate,
                                     const std::string &source_plate,
                                     const std::string &repeated_char,
                                     int candidate_duration,
                                     float candidate_probability,
                                     std::string &accepted_plate)
{
    std::lock_guard<std::mutex> lock(g_temporal_fallback_mutex);
    const TemporalFallbackKey key{app_ctx, track_id};
    TemporalFallbackState &state = g_temporal_fallback_states[key];

    // 清理同一模型下已经长时间未出现的轨迹，避免长时间运行时状态累积。
    if (frame_id >= 0) {
        for (auto it = g_temporal_fallback_states.begin();
             it != g_temporal_fallback_states.end(); ) {
            const bool expired = it->first.app_ctx == app_ctx &&
                it->first.track_id != track_id && it->second.last_frame_id >= 0 &&
                frame_id - it->second.last_frame_id > 60;
            if (expired)
                it = g_temporal_fallback_states.erase(it);
            else
                ++it;
        }
    }

    if (state.plate_type != static_cast<int>(plate_type)) {
        reset_temporal_fallback_state(state, plate_type);
    }

    if (current_result_valid) {
        g_temporal_fallback_states.erase(key);
        return false;
    }

    // frame_id < 0 用于兼容没有帧号的调用方，此时每次调用都视为新的一帧。
    const bool is_new_frame = (frame_id < 0 || frame_id != state.last_frame_id);
    if (is_new_frame) {
        state.consecutive_invalid_frames = std::min(
            state.consecutive_invalid_frames + 1, kTemporalFallbackFrames);
        state.last_frame_id = frame_id;
    }

    // 在连续无效窗口内保留 dur 最大的“少一位”候选；dur 相同时取置信度更高者。
    if (!candidate_plate.empty() &&
        (candidate_duration > state.best_duration ||
         (candidate_duration == state.best_duration &&
          candidate_probability > state.best_probability))) {
        state.best_plate = candidate_plate;
        state.best_source_plate = source_plate;
        state.best_repeated_char = repeated_char;
        state.best_duration = candidate_duration;
        state.best_probability = candidate_probability;
    }

    printf("[LPRNet] Temporal fallback: track=%d, invalid_frames=%d/%d, candidate=%s, best_dur=%d\n",
           track_id, state.consecutive_invalid_frames, kTemporalFallbackFrames,
           candidate_plate.empty() ? "none" : candidate_plate.c_str(),
           state.best_duration);

    if (state.consecutive_invalid_frames < kTemporalFallbackFrames || state.best_plate.empty()) {
        return false;
    }

    accepted_plate = state.best_plate;
    printf("[LPRNet] Temporal fallback accepted: track=%d, %s -> %s, repeated=%s, dur=%d\n",
           track_id, state.best_source_plate.c_str(), state.best_plate.c_str(),
           state.best_repeated_char.c_str(), state.best_duration);
    g_temporal_fallback_states.erase(key);
    return true;
}

int inference_lprnet_model(lprnet_context_t *app_ctx, cv::Mat &plate_img, 
                           lprnet_result *out_result, PlateType plate_type,
                           lprnet_context_t *double_yellow_ctx,
                           float aspect_ratio,
                           const char* save_dir,
                           long long frame_id,
                           int track_id)
{
    static int save_counter = 0;          // 保存图像计数器
    
    // 期望长度：绿牌8位，其他都是7位
    // 蓝牌=7位，黄牌=7位，白牌=7位，绿牌=8位
    int expected_len = (plate_type == PLATE_GREEN || plate_type == PLATE_GREEN_LARGE) ? 8 : 7;
    (void)aspect_ratio;
    
    // 双层黄牌完全由 YOLO 的 doubleline_yellow 类路由，不再按宽高比猜测。
    const bool is_double_yellow = (plate_type == PLATE_YELLOW_DOUBLE);
    
    // 调试输出
    printf("[LPRNet] plate_type=%d, expected_len=%d, aspect_ratio=%.2f, is_double_yellow=%d\n", 
           plate_type, expected_len, aspect_ratio, is_double_yellow);
    int ret;
    rknn_input inputs[1];
    rknn_output outputs[1];

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    // 预处理：确保 3 通道 BGR，resize 到 94x24
    cv::Mat plate_bgr;
    if (plate_img.channels() == 1) {
        cv::cvtColor(plate_img, plate_bgr, cv::COLOR_GRAY2BGR);
    } else if (plate_img.channels() == 4) {
        cv::cvtColor(plate_img, plate_bgr, cv::COLOR_BGRA2BGR);
    } else {
        plate_bgr = plate_img;
    }
    
    cv::Mat resized;
    cv::resize(plate_bgr, resized, cv::Size(94, 24), 0, 0, cv::INTER_CUBIC);

    if (!resized.isContinuous()) {
        resized = resized.clone();
    }

    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = 94 * 24 * 3;
    inputs[0].buf = resized.data;

    ret = rknn_inputs_set(app_ctx->rknn_ctx, 1, inputs);
    if (ret < 0) {
        fprintf(stderr, "rknn_input_set fail! ret=%d\n", ret);
        return -1;
    }

    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0) {
        fprintf(stderr, "rknn_run fail! ret=%d\n", ret);
        return -1;
    }

    outputs[0].want_float = 1;
    ret = rknn_outputs_get(app_ctx->rknn_ctx, 1, outputs, NULL);
    if (ret < 0) {
        fprintf(stderr, "rknn_outputs_get fail! ret=%d\n", ret);
        return ret;
    }

    //=========================================================================
    // 改进的CTC解码：检测重复字符（基于18个时间步的持续时间）
    //=========================================================================
    
    struct CharNode {
        int char_idx;      // 字符索引
        int duration;      // 在18个时间步中持续的长度（关键）
        float max_prob;    // 最大概率
    };
    
    std::vector<CharNode> nodes;
    int current_idx = -1;
    int current_dur = 0;
    float current_max_prob = 0.0f;
    
    // 第一遍扫描：统计每个字符的持续时间
    for (int x = 0; x < OUT_COLS; x++) {
        float *ptr = (float *)outputs[0].buf;
        float preb[OUT_ROWS];
        
        // 保持原有内存访问方式
        for (int y = 0; y < OUT_ROWS; y++) {
            preb[y] = ptr[x];
            ptr += OUT_COLS;
        }
        
        int max_idx = std::max_element(preb, preb + OUT_ROWS) - preb;
        float max_val = preb[max_idx];
        
        // 跳过blank（最后一类）
        if (max_idx == OUT_ROWS - 1) {
            if (current_idx != -1) {
                nodes.push_back({current_idx, current_dur, current_max_prob});
                current_idx = -1;
                current_dur = 0;
                current_max_prob = 0.0f;
            }
            continue;
        }
        
        if (max_idx == current_idx) {
            // 相同字符，持续时间+1
            current_dur++;
            if (max_val > current_max_prob) current_max_prob = max_val;
        } else {
            // 不同字符，保存旧的，开始新的
            if (current_idx != -1) {
                nodes.push_back({current_idx, current_dur, current_max_prob});
            }
            current_idx = max_idx;
            current_dur = 1;
            current_max_prob = max_val;
        }
    }
    // 保存最后一个
    if (current_idx != -1) {
        nodes.push_back({current_idx, current_dur, current_max_prob});
    }
    
    // 第二遍：根据持续时间决定是否重复输出
    out_result->plate_name.clear();
    printf("[LPRNet] CTC nodes: %zu\n", nodes.size());

    struct DecodedNodeSpan {
        int char_idx;
        int duration;
        float max_prob;
        int first_output_char;
        size_t end_byte_offset;
    };
    std::vector<DecodedNodeSpan> decoded_spans;
    int decoded_char_count = 0;
    
    for (auto& node : nodes) {
        int repeat = 1;
        
        // 核心逻辑：根据持续时间判断重复次数
        // 在18个时间步中，每个字符通常占2-3个时间步
        // 提高阈值：≥6才认为是重复，避免绿牌D/F被误判（因分隔符导致持续时间较长）
        if (node.duration >= 6 && node.duration <= 8) {
            repeat = 2;  // 可能是"HH"或"88"
        } else if (node.duration >= 9 && node.duration <= 11) {
            repeat = 3;  // 可能是"888"
        } else if (node.duration >= 12) {
            repeat = 4;  // 可能是"8888"
        }
        
        // 限制总长度
        if (out_result->plate_name.length() + repeat > 8) {
            repeat = 8 - out_result->plate_name.length();
            if (repeat < 1) repeat = 1;
        }
        
        // 输出
        for (int i = 0; i < repeat; i++) {
            out_result->plate_name += plate_code[node.char_idx];
        }

        decoded_spans.push_back({node.char_idx, node.duration, node.max_prob,
                                 decoded_char_count, out_result->plate_name.size()});
        decoded_char_count += repeat;
        
        printf("  idx=%d %s x%d (dur=%d)\n", node.char_idx, plate_code[node.char_idx].c_str(), repeat, node.duration);
    }
    
    // 计算字符数（中文字符算1个）
    auto count_chars = [](const std::string& s) -> int {
        int count = 0;
        for (size_t i = 0; i < s.length(); ) {
            unsigned char c = s[i];
            if ((c & 0x80) == 0) {
                // ASCII字符 (0xxxxxxx)
                count++;
                i++;
            } else if ((c & 0xE0) == 0xC0) {
                // 2字节UTF-8 (110xxxxx)
                count++;
                i += 2;
            } else if ((c & 0xF0) == 0xE0) {
                // 3字节UTF-8 (1110xxxx) - 中文字符
                count++;
                i += 3;
            } else if ((c & 0xF8) == 0xF0) {
                // 4字节UTF-8 (11110xxx)
                count++;
                i += 4;
            } else {
                i++;
            }
        }
        return count;
    };
    
    int char_count = count_chars(out_result->plate_name);
    printf("[LPRNet] Result: %s (chars=%d)\n", 
           out_result->plate_name.c_str(), char_count);
    
    // 补全省份
    bool province_was_added = false;
    if (!out_result->plate_name.empty() && out_result->plate_name[0] >= 'A' && out_result->plate_name[0] <= 'Z') {
        out_result->plate_name = "苏" + out_result->plate_name;
        char_count++;  // 增加字符计数
        province_was_added = true;
        printf("[LPRNet] Fixed: added province -> %s\n", out_result->plate_name.c_str());
    }

    // 为跨帧兜底预生成“重复 dur 最大字符一次”的候选。
    // 只允许重复第3位及之后的车牌主体，避免把省份或第二位地区字母补成两位。
    std::string temporal_candidate;
    std::string temporal_repeated_char;
    int temporal_candidate_duration = -1;
    float temporal_candidate_probability = -1.0f;
    const std::string primary_plate = out_result->plate_name;
    const int primary_char_count = count_chars_utf8(primary_plate);

    if (primary_char_count == expected_len - 1) {
        const size_t added_province_bytes = province_was_added ? std::string("苏").size() : 0;
        const int added_province_chars = province_was_added ? 1 : 0;

        for (const DecodedNodeSpan &span : decoded_spans) {
            const int final_char_position = span.first_output_char + added_province_chars;
            if (final_char_position < 2) {
                continue;
            }

            std::string candidate = primary_plate;
            candidate.insert(span.end_byte_offset + added_province_bytes,
                             plate_code[span.char_idx]);
            if (!is_valid_plate_text(candidate, expected_len)) {
                continue;
            }

            if (span.duration > temporal_candidate_duration ||
                (span.duration == temporal_candidate_duration &&
                 span.max_prob > temporal_candidate_probability)) {
                temporal_candidate = candidate;
                temporal_repeated_char = plate_code[span.char_idx];
                temporal_candidate_duration = span.duration;
                temporal_candidate_probability = span.max_prob;
            }
        }

        if (!temporal_candidate.empty()) {
            printf("[LPRNet] One-short candidate: %s -> %s, repeated=%s, dur=%d\n",
                   primary_plate.c_str(), temporal_candidate.c_str(),
                   temporal_repeated_char.c_str(), temporal_candidate_duration);
        }
    }
    
    // 直接输出模型结果，不做合法性判断和修正
    printf("[LPRNet] Final result: %s (chars=%d)\n",
           out_result->plate_name.c_str(), char_count);

    rknn_outputs_release(app_ctx->rknn_ctx, 1, outputs);

    //=========================================================================
    // 第一次识别结果校验并保存到 last_valid_plate
    //=========================================================================
    {
        const bool v1_valid = is_valid_plate_text(out_result->plate_name, expected_len);
        if (v1_valid) {
            printf("[LPRNet] First result valid: %s\n", out_result->plate_name.c_str());
        } else {
            printf("[LPRNet] First result invalid\n");
        }
    }

    //=========================================================================
    // 双层黄牌二次识别：使用裁剪后的图像和double_yellow模型
    //=========================================================================
    cv::Mat processed_double_yellow;
    std::string secondary_result;  // 保存二次识别结果
    lprnet_context_t *secondary_ctx =
        (double_yellow_ctx != nullptr && double_yellow_ctx->rknn_ctx != 0)
            ? double_yellow_ctx : app_ctx;
    printf("[LPRNet] Checking double_yellow: is_double_yellow=%d, special_ctx=%p, "
           "selected_ctx=%p, using_special=%d\n",
           is_double_yellow, (void*)double_yellow_ctx, (void*)secondary_ctx,
           secondary_ctx == double_yellow_ctx);
    if (is_double_yellow && secondary_ctx != nullptr && secondary_ctx->rknn_ctx != 0) {
        printf("[LPRNet] YOLO double-yellow class selected (aspect_ratio=%.2f for diagnostics), running secondary recognition...\n",
               aspect_ratio);

        // 对resize后的图像进行裁剪重组
        processed_double_yellow = process_double_yellow_plate(resized);

        // 使用双层黄牌专用模型进行二次推理
        lprnet_result double_yellow_result;
        int dy_ret = do_inference(secondary_ctx, processed_double_yellow, &double_yellow_result);

        if (dy_ret == 0 && !double_yellow_result.plate_name.empty()) {
            secondary_result = double_yellow_result.plate_name;
            printf("[LPRNet] Secondary result: %s\n", secondary_result.c_str());
            
            // 校验双层黄牌识别结果，合法才覆盖
            const bool dy_valid = is_valid_plate_text(secondary_result, expected_len);
            
            printf("[LPRNet] Double yellow validation: expected=%d, valid=%d\n",
                   expected_len, dy_valid);
            
            if (dy_valid) {
                out_result->plate_name = secondary_result;
                printf("[LPRNet] Double yellow valid, overwritten: %s\n", out_result->plate_name.c_str());
            } else {
                printf("[LPRNet] Double yellow invalid, keeping first result\n");
            }
        } else {
            printf("[LPRNet] Double yellow model failed\n");
        }
    }

    // 输出最终结果
    printf("[LPRNet] Final result: %s\n", out_result->plate_name.c_str());

    //=========================================================================
    // 状态机：合法性校验与覆盖逻辑
    //=========================================================================
    const bool is_valid = is_valid_plate_text(out_result->plate_name, expected_len);

    printf("[LPRNet] Validation: expected=%d, valid=%d\n", expected_len, is_valid);

    if (is_valid) {
        std::string unused_plate;
        update_temporal_fallback(app_ctx, plate_type, frame_id, track_id, true,
                                 std::string(), std::string(), std::string(),
                                 -1, -1.0f, unused_plate);
        printf("[LPRNet] Valid plate accepted: %s\n", out_result->plate_name.c_str());
    } else {
        std::string fallback_plate;
        const bool fallback_accepted = update_temporal_fallback(
            app_ctx, plate_type, frame_id, track_id, false,
            temporal_candidate, primary_plate, temporal_repeated_char,
            temporal_candidate_duration, temporal_candidate_probability,
            fallback_plate);

        if (fallback_accepted && is_valid_plate_text(fallback_plate, expected_len)) {
            out_result->plate_name = fallback_plate;
            printf("[LPRNet] Invalid result repaired by temporal fallback: %s\n",
                   out_result->plate_name.c_str());
        } else {
            out_result->plate_name.clear();
            printf("[LPRNet] Invalid plate cleared in model layer\n");
        }
    }

    //=========================================================================
    // 保存处理后图像（如果指定了保存目录，则所有车牌都保存）
    //=========================================================================
    if (save_dir != nullptr) {
        const char *plate_type_tag = "blue";
        switch (plate_type) {
        case PLATE_YELLOW_SINGLE:
            plate_type_tag = "yellow_single";
            break;
        case PLATE_YELLOW_DOUBLE:
            plate_type_tag = "yellow_double";
            break;
        case PLATE_GREEN:
            plate_type_tag = "green";
            break;
        case PLATE_GREEN_LARGE:
            plate_type_tag = "green_large";
            break;
        case PLATE_BLUE:
        default:
            plate_type_tag = "blue";
            break;
        }
        const std::string plate_name_for_file =
            out_result->plate_name.empty() ? std::string("invalid") : out_result->plate_name;

        // 创建保存目录
        char rectified_dir[256];
        char resized_dir[256];
        char cropped_dir[256];
        snprintf(rectified_dir, sizeof(rectified_dir), "%s/rectified", save_dir);
        snprintf(resized_dir, sizeof(resized_dir), "%s/resized_94x24", save_dir);
        snprintf(cropped_dir, sizeof(cropped_dir), "%s/cropped", save_dir);

        // 使用system创建目录（RK3568上可用）
        char mkdir_cmd[768];
        snprintf(mkdir_cmd, sizeof(mkdir_cmd), "mkdir -p %s %s %s",
                 rectified_dir, resized_dir, cropped_dir);
        system(mkdir_cmd);

        // 保存矫正后的原始牌图
        char rectified_path[512];
        snprintf(rectified_path, sizeof(rectified_path), "%s/%s_%04d_%s.jpg",
                 rectified_dir, plate_type_tag, save_counter, plate_name_for_file.c_str());
        cv::imwrite(rectified_path, plate_bgr);
        printf("[LPRNet] Saved rectified image: %s\n", rectified_path);

        // 保存resize后的94x24图像（使用最终识别结果作为文件名）
        char resized_path[512];
        snprintf(resized_path, sizeof(resized_path), "%s/%s_%04d_%s.jpg",
                 resized_dir, plate_type_tag, save_counter, plate_name_for_file.c_str());
        cv::imwrite(resized_path, resized);
        printf("[LPRNet] Saved resized image: %s\n", resized_path);

        // 如果是双层黄牌且已处理，保存裁剪后的图像
        if (is_double_yellow && !processed_double_yellow.empty()) {
            char cropped_path[512];
            snprintf(cropped_path, sizeof(cropped_path), "%s/double_yellow_%04d_%s.jpg",
                     cropped_dir, save_counter, plate_name_for_file.c_str());
            cv::imwrite(cropped_path, processed_double_yellow);
            printf("[LPRNet] Saved cropped image: %s\n", cropped_path);
        }

        save_counter++;
    }

    return 0;
}
