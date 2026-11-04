#include "plate_preprocess_rknn.h"
#include "lprnet.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace {

struct Options {
    fs::path sci_model = "/userdata/model/sci_plate_32x2_fp16_rk3568.rknn";
    fs::path mimo_model = "/userdata/model/mimo_plus_stage5_188x48_fp16_rk3568.rknn";
    fs::path lpr_model = "/userdata/model/bluespecial.rknn";
    fs::path labels;
    fs::path input;
    fs::path output_dir = "/mnt/usb/plate_preprocess_results";
    std::size_t limit = 0;
    std::size_t progress = 50;
    unsigned int seed = 20260803U;
    int warmup = 3;
    int repeat = 1;
    bool cascade = false;
    bool run_lpr = true;
    bool quiet_lpr = true;
    bool output_dir_explicit = false;
};

struct RecognitionStats {
    std::size_t attempted = 0;
    std::size_t correct = 0;
    std::size_t errors = 0;
    double runtime_ms = 0.0;
};

struct Aggregate {
    std::size_t processed = 0;
    std::size_t failed = 0;
    double sci_runtime_ms = 0.0;
    double sci_npu_ms = 0.0;
    std::size_t sci_npu_samples = 0;
    double mimo_runtime_ms = 0.0;
    double mimo_npu_ms = 0.0;
    std::size_t mimo_npu_samples = 0;
    double cascade_runtime_ms = 0.0;
    RecognitionStats raw_lpr;
    RecognitionStats sci_lpr;
    RecognitionStats mimo_lpr;
    RecognitionStats cascade_lpr;
};

void usage(const char *program)
{
    std::cout
        << "Usage:\n  " << program
        << " --input PLATE_IMAGE_OR_DIRECTORY"
           " [--sci-model /userdata/model/sci_plate_32x2_fp16_rk3568.rknn]"
           " [--mimo-model /userdata/model/mimo_plus_stage5_188x48_fp16_rk3568.rknn]"
           " [--lpr-model /userdata/model/bluespecial.rknn] [--labels test.txt]"
           " [--output-dir /mnt/usb/plate_preprocess_results]"
           " [--limit 100] [--warmup 3] [--repeat 1] [--progress 50]"
           " [--seed 20260803] [--cascade] [--no-lpr] [--verbose-lpr]\n\n"
           "Input must contain YOLO-cropped plate images, not full vehicle frames.\n";
}

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value after " + key);
            }
            return argv[++i];
        };
        if (key == "--input") {
            options.input = value();
        } else if (key == "--sci-model") {
            options.sci_model = value();
        } else if (key == "--mimo-model") {
            options.mimo_model = value();
        } else if (key == "--lpr-model") {
            options.lpr_model = value();
        } else if (key == "--labels") {
            options.labels = value();
        } else if (key == "--output-dir") {
            options.output_dir = value();
            options.output_dir_explicit = true;
        } else if (key == "--limit") {
            options.limit = std::stoull(value());
        } else if (key == "--warmup") {
            options.warmup = std::stoi(value());
        } else if (key == "--repeat") {
            options.repeat = std::stoi(value());
        } else if (key == "--progress") {
            options.progress = std::stoull(value());
        } else if (key == "--seed") {
            options.seed = static_cast<unsigned int>(std::stoul(value()));
        } else if (key == "--cascade") {
            options.cascade = true;
        } else if (key == "--no-lpr") {
            options.run_lpr = false;
        } else if (key == "--verbose-lpr") {
            options.quiet_lpr = false;
        } else if (key == "--help" || key == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown option: " + key);
        }
    }
    if (options.input.empty()) {
        throw std::runtime_error("--input is required");
    }
    if (options.warmup < 0 || options.repeat <= 0) {
        throw std::runtime_error("--warmup must be >= 0 and --repeat must be > 0");
    }
    return options;
}

bool is_image_path(const fs::path &path)
{
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return extension == ".jpg" || extension == ".jpeg" || extension == ".png" ||
           extension == ".bmp";
}

std::vector<fs::path> discover_images(const Options &options)
{
    std::vector<fs::path> images;
    if (fs::is_regular_file(options.input)) {
        if (!is_image_path(options.input)) {
            throw std::runtime_error("Input file is not a supported image: " +
                                     options.input.string());
        }
        images.push_back(options.input);
    } else if (fs::is_directory(options.input)) {
        for (const auto &entry : fs::recursive_directory_iterator(options.input)) {
            if (entry.is_regular_file() && is_image_path(entry.path())) {
                images.push_back(entry.path());
            }
        }
        std::sort(images.begin(), images.end());
    } else {
        throw std::runtime_error("Input path does not exist: " + options.input.string());
    }
    if (options.limit > 0 && images.size() > options.limit) {
        std::mt19937 random(options.seed);
        std::shuffle(images.begin(), images.end(), random);
        images.resize(options.limit);
        std::sort(images.begin(), images.end());
    }
    if (images.empty()) {
        throw std::runtime_error("No images found under " + options.input.string());
    }
    return images;
}

std::string trim(const std::string &value)
{
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

fs::path resolve_label_file(const Options &options)
{
    if (!options.labels.empty()) {
        return options.labels;
    }
    if (!fs::is_directory(options.input)) {
        return {};
    }
    if (fs::is_regular_file(options.input / "test.txt")) {
        return options.input / "test.txt";
    }
    if (options.input.filename() == "test_images" &&
        fs::is_regular_file(options.input.parent_path() / "test.txt")) {
        return options.input.parent_path() / "test.txt";
    }
    return {};
}

std::unordered_map<std::string, std::string> load_labels(const fs::path &path)
{
    std::unordered_map<std::string, std::string> labels;
    if (path.empty()) {
        return labels;
    }
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open label file: " + path.string());
    }
    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        std::istringstream row(line);
        std::string image_name;
        row >> image_name;
        std::string label;
        std::getline(row, label);
        label = trim(label);
        if (image_name.size() >= 3 &&
            static_cast<unsigned char>(image_name[0]) == 0xEFU &&
            static_cast<unsigned char>(image_name[1]) == 0xBBU &&
            static_cast<unsigned char>(image_name[2]) == 0xBFU) {
            image_name.erase(0, 3);
        }
        if (!image_name.empty() && !label.empty()) {
            labels[image_name] = label;
        }
    }
    return labels;
}

class ScopedStdoutSilence {
public:
    explicit ScopedStdoutSilence(bool enabled)
    {
        if (!enabled) {
            return;
        }
        std::cout.flush();
        std::fflush(stdout);
        saved_fd_ = dup(STDOUT_FILENO);
        null_fd_ = open("/dev/null", O_WRONLY);
        if (saved_fd_ >= 0 && null_fd_ >= 0 && dup2(null_fd_, STDOUT_FILENO) >= 0) {
            active_ = true;
        }
    }

    ~ScopedStdoutSilence()
    {
        if (active_) {
            std::fflush(stdout);
            dup2(saved_fd_, STDOUT_FILENO);
        }
        if (null_fd_ >= 0) {
            close(null_fd_);
        }
        if (saved_fd_ >= 0) {
            close(saved_fd_);
        }
    }

    ScopedStdoutSilence(const ScopedStdoutSilence &) = delete;
    ScopedStdoutSilence &operator=(const ScopedStdoutSilence &) = delete;

private:
    int saved_fd_ = -1;
    int null_fd_ = -1;
    bool active_ = false;
};

class LprRunner {
public:
    explicit LprRunner(const fs::path &model_path)
    {
        if (init_lprnet_model(model_path.string().c_str(), &context_) != 0) {
            throw std::runtime_error("Failed to load LPR model: " + model_path.string());
        }
        initialized_ = true;
        if (context_.io_num.n_input != 1 || context_.io_num.n_output != 1 ||
            context_.input_attrs == nullptr || context_.output_attrs == nullptr ||
            context_.input_attrs[0].n_elems != MODEL_WIDTH * MODEL_HEIGHT * 3 ||
            context_.output_attrs[0].n_elems != OUT_ROWS * OUT_COLS) {
            release_lprnet_model(&context_);
            initialized_ = false;
            throw std::runtime_error("LPR model shape does not match ARM_final1 decoder");
        }
    }

    ~LprRunner()
    {
        if (initialized_) {
            release_lprnet_model(&context_);
        }
    }

    LprRunner(const LprRunner &) = delete;
    LprRunner &operator=(const LprRunner &) = delete;

    bool infer(cv::Mat &plate_bgr,
               long long frame_id,
               int track_id,
               bool quiet,
               std::string *prediction,
               double *runtime_ms,
               std::string *error)
    {
        lprnet_result result;
        const float aspect_ratio = plate_bgr.rows > 0
                                       ? static_cast<float>(plate_bgr.cols) / plate_bgr.rows
                                       : 0.0f;
        const auto begin = std::chrono::steady_clock::now();
        int ret = 0;
        {
            ScopedStdoutSilence silence(quiet);
            ret = inference_lprnet_model(&context_, plate_bgr, &result, PLATE_BLUE,
                                         nullptr, aspect_ratio, nullptr, frame_id, track_id);
        }
        const auto end = std::chrono::steady_clock::now();
        *runtime_ms = std::chrono::duration<double, std::milli>(end - begin).count();
        if (ret != 0) {
            *error = "inference_lprnet_model failed, ret=" + std::to_string(ret);
            return false;
        }
        *prediction = result.plate_name;
        return true;
    }

private:
    lprnet_context_t context_{};
    bool initialized_ = false;
};

std::string csv_escape(const std::string &value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) {
        return value;
    }
    std::string result = "\"";
    for (const char c : value) {
        result += c == '\"' ? "\"\"" : std::string(1, c);
    }
    result += '\"';
    return result;
}

double mean_luma(const cv::Mat &bgr)
{
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return cv::mean(gray)[0];
}

double laplacian_variance(const cv::Mat &bgr)
{
    cv::Mat gray;
    cv::Mat laplacian;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

bool run_repeated(PlatePreprocessorRknn *model,
                  const cv::Mat &input,
                  int repeat,
                  cv::Mat *output,
                  PlatePreprocessTiming *average,
                  std::string *error)
{
    PlatePreprocessTiming sum;
    sum.npu_ms = 0.0;
    int npu_samples = 0;
    for (int i = 0; i < repeat; ++i) {
        PlatePreprocessTiming current;
        if (!model->infer(input, output, &current, error)) {
            return false;
        }
        sum.preprocess_ms += current.preprocess_ms;
        sum.runtime_ms += current.runtime_ms;
        sum.postprocess_ms += current.postprocess_ms;
        if (current.npu_ms >= 0.0) {
            sum.npu_ms += current.npu_ms;
            ++npu_samples;
        }
    }
    average->preprocess_ms = sum.preprocess_ms / repeat;
    average->runtime_ms = sum.runtime_ms / repeat;
    average->postprocess_ms = sum.postprocess_ms / repeat;
    average->npu_ms = npu_samples > 0 ? sum.npu_ms / npu_samples : -1.0;
    return true;
}

void warm_up(PlatePreprocessorRknn *model, const cv::Mat &image, int iterations)
{
    cv::Mat ignored;
    std::string error;
    for (int i = 0; i < iterations; ++i) {
        if (!model->infer(image, &ignored, nullptr, &error)) {
            throw std::runtime_error(std::string(model->name()) + " warmup failed: " + error);
        }
    }
}

void update_recognition_stats(RecognitionStats *stats,
                              const std::string &label,
                              const std::string &prediction,
                              double runtime_ms)
{
    if (label.empty()) {
        return;
    }
    ++stats->attempted;
    stats->runtime_ms += runtime_ms;
    if (prediction == label) {
        ++stats->correct;
    }
}

double exact_accuracy(const RecognitionStats &stats)
{
    return stats.attempted == 0
               ? 0.0
               : 100.0 * static_cast<double>(stats.correct) / stats.attempted;
}

void print_recognition_stats(const char *name, const RecognitionStats &stats)
{
    if (stats.attempted == 0) {
        return;
    }
    std::cout << name << " exact accuracy: " << stats.correct << '/' << stats.attempted
              << " = " << exact_accuracy(stats) << "%"
              << ", average LPR=" << stats.runtime_ms / stats.attempted << " ms"
              << ", errors=" << stats.errors << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        const Options options = parse_options(argc, argv);
        const std::vector<fs::path> images = discover_images(options);
        const fs::path label_file = resolve_label_file(options);
        const std::unordered_map<std::string, std::string> labels = load_labels(label_file);
        fs::path output_dir = options.output_dir;
        if (!options.output_dir_explicit && fs::is_directory(options.input) &&
            options.input.filename() == "test_images") {
            output_dir /= options.input.parent_path().filename();
        }
        fs::create_directories(output_dir);

        std::cout << "SCI model:  " << options.sci_model << '\n'
                  << "MIMO model: " << options.mimo_model << '\n'
                  << "LPR model:  " << (options.run_lpr ? options.lpr_model.string() : "disabled")
                  << '\n'
                  << "Input:      " << options.input << '\n'
                  << "Images:     " << images.size() << '\n'
                  << "Seed:       " << options.seed << '\n'
                  << "Labels:     "
                  << (label_file.empty() ? std::string("not found") : label_file.string())
                  << " (" << labels.size() << ")\n"
                  << "Output:     " << output_dir << '\n';

        PlatePreprocessorRknn sci(PlatePreprocessKind::Sci);
        PlatePreprocessorRknn mimo(PlatePreprocessKind::MimoPlus);
        std::string error;
        if (!sci.init(options.sci_model.string(), &error)) {
            throw std::runtime_error(error);
        }
        if (!mimo.init(options.mimo_model.string(), &error)) {
            throw std::runtime_error(error);
        }
        std::unique_ptr<LprRunner> lpr;
        if (options.run_lpr) {
            lpr = std::make_unique<LprRunner>(options.lpr_model);
        }

        cv::Mat first = cv::imread(images.front().string(), cv::IMREAD_COLOR);
        if (first.empty()) {
            throw std::runtime_error("Cannot read warmup image: " + images.front().string());
        }
        warm_up(&sci, first, options.warmup);
        warm_up(&mimo, first, options.warmup);
        if (lpr) {
            for (int i = 0; i < options.warmup; ++i) {
                std::string prediction;
                double runtime_ms = 0.0;
                if (!lpr->infer(first, -1000 - i, -1000 - i, options.quiet_lpr,
                                &prediction, &runtime_ms, &error)) {
                    throw std::runtime_error("LPR warmup failed: " + error);
                }
            }
        }

        const fs::path csv_path = output_dir / "plate_preprocess_results.csv";
        std::ofstream csv(csv_path);
        if (!csv) {
            throw std::runtime_error("Cannot create " + csv_path.string());
        }
        csv << "index,input,label,status,raw_prediction,sci_prediction,mimo_prediction,"
               "cascade_prediction,raw_correct,sci_correct,mimo_correct,cascade_correct,"
               "raw_lpr_ms,sci_lpr_ms,mimo_lpr_ms,cascade_lpr_ms,raw_luma,raw_laplacian,"
               "sci_pre_ms,sci_runtime_ms,sci_npu_ms,sci_post_ms,sci_luma,sci_laplacian,"
               "mimo_pre_ms,mimo_runtime_ms,mimo_npu_ms,mimo_post_ms,mimo_luma,mimo_laplacian,"
               "raw_output,sci_output,mimo_output,cascade_output\n";

        Aggregate total;
        for (std::size_t index = 0; index < images.size(); ++index) {
            const fs::path &path = images[index];
            const auto label_it = labels.find(path.filename().string());
            const std::string label = label_it == labels.end() ? std::string() : label_it->second;
            cv::Mat raw = cv::imread(path.string(), cv::IMREAD_COLOR);
            if (raw.empty()) {
                ++total.failed;
                csv << index << ',' << csv_escape(path.string()) << ',' << csv_escape(label)
                    << ",read_error\n";
                continue;
            }

            cv::Mat sci_output;
            cv::Mat mimo_output;
            PlatePreprocessTiming sci_timing;
            PlatePreprocessTiming mimo_timing;
            if (!run_repeated(&sci, raw, options.repeat, &sci_output, &sci_timing, &error) ||
                !run_repeated(&mimo, raw, options.repeat, &mimo_output, &mimo_timing, &error)) {
                ++total.failed;
                csv << index << ',' << csv_escape(path.string()) << ',' << csv_escape(label)
                    << ",inference_error\n";
                continue;
            }

            cv::Mat cascade_output;
            PlatePreprocessTiming cascade_timing;
            if (options.cascade &&
                !run_repeated(&mimo, sci_output, options.repeat, &cascade_output,
                              &cascade_timing, &error)) {
                ++total.failed;
                csv << index << ',' << csv_escape(path.string()) << ',' << csv_escape(label)
                    << ",cascade_error\n";
                continue;
            }

            std::string raw_prediction;
            std::string sci_prediction;
            std::string mimo_prediction;
            std::string cascade_prediction;
            double raw_lpr_ms = 0.0;
            double sci_lpr_ms = 0.0;
            double mimo_lpr_ms = 0.0;
            double cascade_lpr_ms = 0.0;
            bool lpr_ok = true;
            if (lpr) {
                const long long frame_id = static_cast<long long>(index);
                const int track_base = static_cast<int>(index * 4U);
                auto recognize = [&](cv::Mat &image,
                                     int branch,
                                     std::string *prediction,
                                     double *runtime_ms,
                                     RecognitionStats *stats) {
                    if (!lpr->infer(image, frame_id, track_base + branch, options.quiet_lpr,
                                    prediction, runtime_ms, &error)) {
                        ++stats->errors;
                        if (!label.empty()) {
                            ++stats->attempted;
                            stats->runtime_ms += *runtime_ms;
                        }
                        *prediction = "<ERROR>";
                        lpr_ok = false;
                        return;
                    }
                    update_recognition_stats(stats, label, *prediction, *runtime_ms);
                };
                recognize(raw, 0, &raw_prediction, &raw_lpr_ms, &total.raw_lpr);
                recognize(sci_output, 1, &sci_prediction, &sci_lpr_ms, &total.sci_lpr);
                recognize(mimo_output, 2, &mimo_prediction, &mimo_lpr_ms, &total.mimo_lpr);
                if (options.cascade) {
                    recognize(cascade_output, 3, &cascade_prediction, &cascade_lpr_ms,
                              &total.cascade_lpr);
                }
            }

            std::ostringstream prefix;
            prefix << std::setw(6) << std::setfill('0') << index << '_' << path.stem().string();
            const fs::path raw_path = output_dir / (prefix.str() + "_raw.jpg");
            const fs::path sci_path = output_dir / (prefix.str() + "_sci.jpg");
            const fs::path mimo_path = output_dir / (prefix.str() + "_mimo.jpg");
            fs::path cascade_path;
            cv::imwrite(raw_path.string(), raw);
            cv::imwrite(sci_path.string(), sci_output);
            cv::imwrite(mimo_path.string(), mimo_output);
            if (options.cascade) {
                cascade_path = output_dir / (prefix.str() + "_sci_mimo.jpg");
                cv::imwrite(cascade_path.string(), cascade_output);
            }

            csv << index << ',' << csv_escape(path.string()) << ',' << csv_escape(label) << ','
                << (lpr_ok ? "ok" : "lpr_error") << ',' << csv_escape(raw_prediction) << ','
                << csv_escape(sci_prediction) << ',' << csv_escape(mimo_prediction) << ','
                << csv_escape(cascade_prediction) << ','
                << (!label.empty() && raw_prediction == label ? 1 : 0) << ','
                << (!label.empty() && sci_prediction == label ? 1 : 0) << ','
                << (!label.empty() && mimo_prediction == label ? 1 : 0) << ','
                << (!label.empty() && cascade_prediction == label ? 1 : 0) << ','
                << std::fixed << std::setprecision(4) << raw_lpr_ms << ',' << sci_lpr_ms << ','
                << mimo_lpr_ms << ',' << cascade_lpr_ms << ',' << mean_luma(raw) << ','
                << laplacian_variance(raw)
                << ',' << sci_timing.preprocess_ms << ',' << sci_timing.runtime_ms << ','
                << sci_timing.npu_ms << ',' << sci_timing.postprocess_ms << ','
                << mean_luma(sci_output) << ',' << laplacian_variance(sci_output) << ','
                << mimo_timing.preprocess_ms << ',' << mimo_timing.runtime_ms << ','
                << mimo_timing.npu_ms << ',' << mimo_timing.postprocess_ms << ','
                << mean_luma(mimo_output) << ',' << laplacian_variance(mimo_output) << ','
                << csv_escape(raw_path.string()) << ',' << csv_escape(sci_path.string()) << ','
                << csv_escape(mimo_path.string()) << ',' << csv_escape(cascade_path.string())
                << '\n';

            ++total.processed;
            total.sci_runtime_ms += sci_timing.runtime_ms;
            total.mimo_runtime_ms += mimo_timing.runtime_ms;
            if (sci_timing.npu_ms >= 0.0) {
                total.sci_npu_ms += sci_timing.npu_ms;
                ++total.sci_npu_samples;
            }
            if (mimo_timing.npu_ms >= 0.0) {
                total.mimo_npu_ms += mimo_timing.npu_ms;
                ++total.mimo_npu_samples;
            }
            if (options.cascade) {
                total.cascade_runtime_ms += cascade_timing.runtime_ms;
            }

            if (options.progress > 0 &&
                ((index + 1) % options.progress == 0 || index + 1 == images.size())) {
                std::cout << "Processed " << (index + 1) << '/' << images.size()
                          << " SCI=" << sci_timing.runtime_ms << " ms"
                          << " MIMO=" << mimo_timing.runtime_ms << " ms";
                if (total.raw_lpr.attempted > 0) {
                    std::cout << " accuracy(raw/sci/mimo)=" << exact_accuracy(total.raw_lpr)
                              << '/' << exact_accuracy(total.sci_lpr) << '/'
                              << exact_accuracy(total.mimo_lpr);
                    if (options.cascade) {
                        std::cout << '/' << exact_accuracy(total.cascade_lpr);
                    }
                }
                std::cout << '\n';
            }
        }

        if (total.processed == 0) {
            throw std::runtime_error("No image completed successfully");
        }
        const double count = static_cast<double>(total.processed);
        std::cout << std::fixed << std::setprecision(3)
                  << "Done: processed=" << total.processed << " failed=" << total.failed << '\n'
                  << "SCI average runtime:  " << total.sci_runtime_ms / count << " ms\n"
                  << "MIMO average runtime: " << total.mimo_runtime_ms / count << " ms\n";
        if (total.sci_npu_samples > 0) {
            std::cout << "SCI RKNN reported NPU:  "
                      << total.sci_npu_ms / total.sci_npu_samples << " ms\n";
        }
        if (total.mimo_npu_samples > 0) {
            std::cout << "MIMO RKNN reported NPU: "
                      << total.mimo_npu_ms / total.mimo_npu_samples << " ms\n";
        }
        if (options.cascade) {
            std::cout << "SCI->MIMO extra MIMO runtime: "
                      << total.cascade_runtime_ms / count << " ms\n";
        }
        print_recognition_stats("Raw", total.raw_lpr);
        print_recognition_stats("SCI", total.sci_lpr);
        print_recognition_stats("MIMO", total.mimo_lpr);
        if (options.cascade) {
            print_recognition_stats("SCI->MIMO", total.cascade_lpr);
        }
        if (total.raw_lpr.attempted > 0) {
            const double raw_accuracy = exact_accuracy(total.raw_lpr);
            std::cout << "Accuracy delta vs raw: SCI="
                      << exact_accuracy(total.sci_lpr) - raw_accuracy << " pp, MIMO="
                      << exact_accuracy(total.mimo_lpr) - raw_accuracy << " pp";
            if (options.cascade) {
                std::cout << ", SCI->MIMO="
                          << exact_accuracy(total.cascade_lpr) - raw_accuracy << " pp";
            }
            std::cout << '\n';
        }
        std::cout << "CSV: " << csv_path << '\n';
        return 0;
    } catch (const std::exception &exception) {
        std::cerr << "ERROR: " << exception.what() << '\n';
        usage(argv[0]);
        return 1;
    }
}
