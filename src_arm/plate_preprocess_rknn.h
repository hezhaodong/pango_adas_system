#ifndef PLATE_PREPROCESS_RKNN_H
#define PLATE_PREPROCESS_RKNN_H

#include "rknn_api.h"

#include <opencv2/core/mat.hpp>

#include <string>
#include <vector>

enum class PlatePreprocessKind {
    Sci,
    MimoPlus,
};

struct PlatePreprocessTiming {
    double preprocess_ms = 0.0;
    double runtime_ms = 0.0;
    double npu_ms = -1.0;
    double postprocess_ms = 0.0;
};

class PlatePreprocessorRknn {
public:
    explicit PlatePreprocessorRknn(PlatePreprocessKind kind);
    ~PlatePreprocessorRknn();

    PlatePreprocessorRknn(const PlatePreprocessorRknn &) = delete;
    PlatePreprocessorRknn &operator=(const PlatePreprocessorRknn &) = delete;

    bool init(const std::string &model_path, std::string *error = nullptr);
    void release();

    // Input and output use the BGR cv::Mat convention used by ARM_final1.
    // The wrapper handles BGR->RGB, RKNN normalization, and RGB->BGR.
    bool infer(const cv::Mat &plate_bgr,
               cv::Mat *enhanced_bgr,
               PlatePreprocessTiming *timing = nullptr,
               std::string *error = nullptr);

    bool is_initialized() const { return context_ != 0; }
    PlatePreprocessKind kind() const { return kind_; }
    const char *name() const;
    int network_width() const { return network_width_; }
    int network_height() const { return network_height_; }

private:
    bool set_error(const std::string &message, std::string *error) const;
    bool float_output_to_bgr(const float *data,
                             std::size_t element_count,
                             cv::Mat *enhanced_bgr,
                             std::string *error) const;

    PlatePreprocessKind kind_;
    rknn_context context_ = 0;
    rknn_input_output_num io_num_{};
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;
    // MIMO+ stage5 model consumes a fixed 188x48 input (no extra padding);
    // content and network width are identical for the MimoPlus branch.
    int content_width_ = 0;
    int network_width_ = 0;
    int network_height_ = 0;
};

#endif  // PLATE_PREPROCESS_RKNN_H
