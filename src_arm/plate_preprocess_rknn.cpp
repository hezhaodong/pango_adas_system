#include "plate_preprocess_rknn.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <utility>

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point begin, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

std::string tensor_description(const rknn_tensor_attr &attr)
{
    std::ostringstream stream;
    stream << "name=" << attr.name << " fmt=" << get_format_string(attr.fmt)
           << " type=" << get_type_string(attr.type) << " dims=[";
    for (uint32_t i = 0; i < attr.n_dims; ++i) {
        if (i != 0) {
            stream << ',';
        }
        stream << attr.dims[i];
    }
    stream << "] elems=" << attr.n_elems << " zp=" << attr.zp
           << " scale=" << attr.scale;
    return stream.str();
}

}  // namespace

PlatePreprocessorRknn::PlatePreprocessorRknn(PlatePreprocessKind kind) : kind_(kind)
{
    if (kind_ == PlatePreprocessKind::Sci) {
        content_width_ = 94;
        network_width_ = 94;
        network_height_ = 24;
    } else {
        content_width_ = 188;
        network_width_ = 188;  // MIMO+ stage5 fixed 188x48 input (matches training/ONNX)
        network_height_ = 48;
    }
}

PlatePreprocessorRknn::~PlatePreprocessorRknn()
{
    release();
}

const char *PlatePreprocessorRknn::name() const
{
    return kind_ == PlatePreprocessKind::Sci ? "SCI" : "Deblur-8M";
}

bool PlatePreprocessorRknn::set_error(const std::string &message, std::string *error) const
{
    if (error != nullptr) {
        *error = message;
    }
    std::fprintf(stderr, "[PlatePreprocess:%s] %s\n", name(), message.c_str());
    return false;
}

bool PlatePreprocessorRknn::init(const std::string &model_path, std::string *error)
{
    release();

    // size=0 tells RKNN Runtime that model_path is a file name. This avoids
    // keeping a second 30+ MiB copy of MIMO+ in ARM memory during init.
    int ret = rknn_init(&context_, const_cast<char *>(model_path.c_str()), 0, 0, nullptr);
    if (ret != RKNN_SUCC) {
        context_ = 0;
        return set_error("rknn_init failed for " + model_path + ", ret=" +
                             std::to_string(ret),
                         error);
    }

    ret = rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != RKNN_SUCC) {
        const std::string message = "RKNN_QUERY_IN_OUT_NUM failed, ret=" +
                                    std::to_string(ret);
        release();
        return set_error(message, error);
    }
    if (io_num_.n_input != 1 || io_num_.n_output != 1) {
        const std::string message = "expected one input and one output, got " +
                                    std::to_string(io_num_.n_input) + "/" +
                                    std::to_string(io_num_.n_output);
        release();
        return set_error(message, error);
    }

    input_attrs_.assign(io_num_.n_input, {});
    output_attrs_.assign(io_num_.n_output, {});
    for (uint32_t i = 0; i < io_num_.n_input; ++i) {
        input_attrs_[i].index = i;
        ret = rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attrs_[i],
                         sizeof(input_attrs_[i]));
        if (ret != RKNN_SUCC) {
            const std::string message = "RKNN_QUERY_INPUT_ATTR failed, ret=" +
                                        std::to_string(ret);
            release();
            return set_error(message, error);
        }
    }
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        output_attrs_[i].index = i;
        ret = rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i],
                         sizeof(output_attrs_[i]));
        if (ret != RKNN_SUCC) {
            const std::string message = "RKNN_QUERY_OUTPUT_ATTR failed, ret=" +
                                        std::to_string(ret);
            release();
            return set_error(message, error);
        }
    }

    const std::size_t expected_elements = static_cast<std::size_t>(network_width_) *
                                          network_height_ * 3U;
    if (input_attrs_[0].n_elems != expected_elements ||
        output_attrs_[0].n_elems != expected_elements) {
        const std::string message =
            "tensor shape does not match expected " + std::to_string(network_width_) +
            "x" + std::to_string(network_height_) + "x3 model";
        release();
        return set_error(message, error);
    }

    std::printf("[PlatePreprocess:%s] model=%s\n", name(), model_path.c_str());
    std::printf("[PlatePreprocess:%s] input  %s\n", name(),
                tensor_description(input_attrs_[0]).c_str());
    std::printf("[PlatePreprocess:%s] output %s\n", name(),
                tensor_description(output_attrs_[0]).c_str());
    return true;
}

void PlatePreprocessorRknn::release()
{
    input_attrs_.clear();
    output_attrs_.clear();
    std::memset(&io_num_, 0, sizeof(io_num_));
    if (context_ != 0) {
        rknn_destroy(context_);
        context_ = 0;
    }
}

bool PlatePreprocessorRknn::float_output_to_bgr(const float *data,
                                                std::size_t element_count,
                                                cv::Mat *enhanced_bgr,
                                                std::string *error) const
{
    if (data == nullptr || enhanced_bgr == nullptr) {
        return set_error("null output buffer", error);
    }
    const std::size_t pixels = static_cast<std::size_t>(network_width_) * network_height_;
    if (element_count < pixels * 3U) {
        return set_error("output buffer is smaller than expected", error);
    }

    cv::Mat rgb(network_height_, network_width_, CV_8UC3);
    const bool is_nhwc = output_attrs_[0].fmt == RKNN_TENSOR_NHWC;
    for (int y = 0; y < network_height_; ++y) {
        cv::Vec3b *row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < network_width_; ++x) {
            const std::size_t pixel = static_cast<std::size_t>(y) * network_width_ + x;
            for (int c = 0; c < 3; ++c) {
                const std::size_t index = is_nhwc ? pixel * 3U + c
                                                  : static_cast<std::size_t>(c) * pixels + pixel;
                const float value = std::clamp(data[index], 0.0f, 1.0f);
                row[x][c] = cv::saturate_cast<unsigned char>(std::lround(value * 255.0f));
            }
        }
    }

    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    if (kind_ == PlatePreprocessKind::MimoPlus) {
        // Discard the four replicated columns inserted at input.  The UI and
        // downstream display code keep their existing 94x24 plate format.
        const cv::Mat plate_only = bgr(cv::Rect(0, 0, content_width_, network_height_));
        cv::resize(plate_only, *enhanced_bgr, cv::Size(94, 24), 0.0, 0.0, cv::INTER_AREA);
    } else {
        *enhanced_bgr = std::move(bgr);
    }
    return true;
}

bool PlatePreprocessorRknn::infer(const cv::Mat &plate_bgr,
                                  cv::Mat *enhanced_bgr,
                                  PlatePreprocessTiming *timing,
                                  std::string *error)
{
    if (!is_initialized()) {
        return set_error("model is not initialized", error);
    }
    if (plate_bgr.empty()) {
        return set_error("input image is empty", error);
    }
    if (plate_bgr.depth() != CV_8U) {
        return set_error("input image must use 8-bit pixels", error);
    }

    PlatePreprocessTiming local_timing;
    const auto preprocess_begin = Clock::now();
    cv::Mat resized;
    // Match the training pipeline exactly: normalize the YOLO crop to 94x24
    // first (same as LPR), then bilinear x2 to the 188x48 network input.
    cv::resize(plate_bgr, resized, cv::Size(94, 24), 0.0, 0.0, cv::INTER_LINEAR);
    cv::resize(resized, resized, cv::Size(content_width_, network_height_),
               0.0, 0.0, cv::INTER_LINEAR);
    if (network_width_ > content_width_) {
        cv::copyMakeBorder(resized, resized, 0, 0, 0, network_width_ - content_width_,
                           cv::BORDER_REPLICATE);
    }
    cv::Mat rgb;
    if (resized.channels() == 3) {
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    } else if (resized.channels() == 4) {
        cv::cvtColor(resized, rgb, cv::COLOR_BGRA2RGB);
    } else if (resized.channels() == 1) {
        cv::cvtColor(resized, rgb, cv::COLOR_GRAY2RGB);
    } else {
        return set_error("unsupported input channel count", error);
    }
    if (!rgb.isContinuous()) {
        rgb = rgb.clone();
    }
    local_timing.preprocess_ms = elapsed_ms(preprocess_begin, Clock::now());

    rknn_input input{};
    input.index = 0;
    input.buf = rgb.data;
    input.size = static_cast<uint32_t>(rgb.total() * rgb.elemSize());
    input.pass_through = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;

    rknn_output output{};
    output.index = 0;
    output.want_float = 1;
    output.is_prealloc = 0;

    const auto runtime_begin = Clock::now();
    int ret = rknn_inputs_set(context_, 1, &input);
    if (ret != RKNN_SUCC) {
        return set_error("rknn_inputs_set failed, ret=" + std::to_string(ret), error);
    }
    ret = rknn_run(context_, nullptr);
    if (ret != RKNN_SUCC) {
        return set_error("rknn_run failed, ret=" + std::to_string(ret), error);
    }
    ret = rknn_outputs_get(context_, 1, &output, nullptr);
    local_timing.runtime_ms = elapsed_ms(runtime_begin, Clock::now());
    if (ret != RKNN_SUCC) {
        return set_error("rknn_outputs_get failed, ret=" + std::to_string(ret), error);
    }

    rknn_perf_run performance{};
    if (rknn_query(context_, RKNN_QUERY_PERF_RUN, &performance, sizeof(performance)) ==
        RKNN_SUCC) {
        local_timing.npu_ms = static_cast<double>(performance.run_duration) / 1000.0;
    }

    const auto postprocess_begin = Clock::now();
    const bool converted = float_output_to_bgr(
        static_cast<const float *>(output.buf), output_attrs_[0].n_elems, enhanced_bgr, error);
    rknn_outputs_release(context_, 1, &output);
    local_timing.postprocess_ms = elapsed_ms(postprocess_begin, Clock::now());

    if (timing != nullptr) {
        *timing = local_timing;
    }
    return converted;
}
