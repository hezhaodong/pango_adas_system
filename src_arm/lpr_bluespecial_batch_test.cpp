#include "lprnet.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Options {
    fs::path model;
    fs::path dataset;
    fs::path output;
    std::string split;
    std::size_t limit = 0;
    std::size_t progress_interval = 500;
    int warmup_runs = 5;
};

struct Sample {
    std::string split;
    fs::path image_path;
    std::string image_name;
    std::string label;
};

struct Stats {
    std::size_t listed = 0;
    std::size_t processed = 0;
    std::size_t correct = 0;
    std::size_t missing = 0;
    std::size_t read_errors = 0;
    std::size_t inference_errors = 0;
    std::size_t label_chars = 0;
    std::size_t edit_distance = 0;
    double inference_ms = 0.0;
};

struct SplitResult {
    std::string name;
    Stats stats;
};

std::string trim(const std::string &value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string csv_escape(const std::string &value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) {
        return value;
    }
    std::string escaped = "\"";
    for (char c : value) {
        if (c == '"') {
            escaped += "\"\"";
        } else {
            escaped += c;
        }
    }
    escaped += '"';
    return escaped;
}

std::vector<std::string> utf8_characters(const std::string &value)
{
    std::vector<std::string> result;
    for (std::size_t i = 0; i < value.size();) {
        const unsigned char lead = static_cast<unsigned char>(value[i]);
        std::size_t length = 1;
        if ((lead & 0xE0U) == 0xC0U) {
            length = 2;
        } else if ((lead & 0xF0U) == 0xE0U) {
            length = 3;
        } else if ((lead & 0xF8U) == 0xF0U) {
            length = 4;
        }
        if (i + length > value.size()) {
            length = 1;
        }
        result.emplace_back(value.substr(i, length));
        i += length;
    }
    return result;
}

std::size_t levenshtein_distance(const std::vector<std::string> &left,
                                 const std::vector<std::string> &right)
{
    std::vector<std::size_t> previous(right.size() + 1);
    std::vector<std::size_t> current(right.size() + 1);
    std::iota(previous.begin(), previous.end(), 0);
    for (std::size_t i = 1; i <= left.size(); ++i) {
        current[0] = i;
        for (std::size_t j = 1; j <= right.size(); ++j) {
            const std::size_t substitution = previous[j - 1] +
                                             (left[i - 1] == right[j - 1] ? 0U : 1U);
            current[j] = std::min({previous[j] + 1, current[j - 1] + 1, substitution});
        }
        previous.swap(current);
    }
    return previous.back();
}

void print_usage(const char *program)
{
    std::cout
        << "Usage:\n  " << program
        << " --model bluespecial.rknn"
           " --dataset /media/usb/extracted_datasets_split_v4"
           " [--output /media/usb/bluespecial_board_results.csv]"
           " [--split ccpd_db] [--limit 100] [--warmup 5] [--progress 500]\n";
}

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto require_value = [&]() -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value after " + key);
            }
            return argv[++i];
        };
        if (key == "--model") {
            options.model = require_value();
        } else if (key == "--dataset") {
            options.dataset = require_value();
        } else if (key == "--output") {
            options.output = require_value();
        } else if (key == "--split") {
            options.split = require_value();
        } else if (key == "--limit") {
            options.limit = std::stoull(require_value());
        } else if (key == "--warmup") {
            options.warmup_runs = std::stoi(require_value());
        } else if (key == "--progress") {
            options.progress_interval = std::stoull(require_value());
        } else if (key == "--help" || key == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown option: " + key);
        }
    }
    if (options.model.empty() || options.dataset.empty()) {
        throw std::runtime_error("--model and --dataset are required");
    }
    if (options.output.empty()) {
        options.output = options.dataset / "bluespecial_board_results.csv";
    }
    if (options.warmup_runs < 0) {
        throw std::runtime_error("--warmup cannot be negative");
    }
    return options;
}

std::vector<fs::path> discover_split_directories(const Options &options)
{
    std::vector<fs::path> result;
    const auto is_split = [](const fs::path &path) {
        return fs::is_regular_file(path / "test.txt") &&
               fs::is_directory(path / "test_images");
    };

    if (is_split(options.dataset)) {
        if (options.split.empty() || options.dataset.filename().string() == options.split) {
            result.push_back(options.dataset);
        }
    } else {
        for (const auto &entry : fs::directory_iterator(options.dataset)) {
            if (!entry.is_directory() || !is_split(entry.path())) {
                continue;
            }
            if (!options.split.empty() && entry.path().filename().string() != options.split) {
                continue;
            }
            result.push_back(entry.path());
        }
    }
    std::sort(result.begin(), result.end());
    if (result.empty()) {
        throw std::runtime_error("No matching test.txt/test_images dataset found under " +
                                 options.dataset.string());
    }
    return result;
}

std::vector<Sample> load_samples(const Options &options)
{
    std::vector<Sample> samples;
    for (const fs::path &split_path : discover_split_directories(options)) {
        std::ifstream input(split_path / "test.txt");
        if (!input) {
            throw std::runtime_error("Cannot open " + (split_path / "test.txt").string());
        }
        const std::string split_name = split_path.filename().string();
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
            if (image_name.empty() || label.empty()) {
                std::cerr << "Warning: ignored malformed row: " << line << '\n';
                continue;
            }
            samples.push_back({split_name, split_path / "test_images" / image_name,
                               image_name, label});
            if (options.limit > 0 && samples.size() >= options.limit) {
                return samples;
            }
        }
    }
    return samples;
}

class OriginalVideoPipelineLpr {
public:
    explicit OriginalVideoPipelineLpr(const fs::path &model_path)
    {
        if (init_lprnet_model(model_path.string().c_str(), &context_) != 0) {
            throw std::runtime_error("Original video pipeline failed to load model: " +
                                     model_path.string());
        }
        loaded_ = true;
        std::cout << "Original ARM_final1 LPR pipeline model attributes:\n"
                  << "  parsed input=" << context_.model_width << 'x'
                  << context_.model_height << 'x' << context_.model_channel << '\n'
                  << "  input elements="
                  << (context_.input_attrs ? context_.input_attrs[0].n_elems : 0) << '\n'
                  << "  output elements="
                  << (context_.output_attrs ? context_.output_attrs[0].n_elems : 0) << '\n';
        if (context_.io_num.n_input != 1 || context_.io_num.n_output != 1 ||
            context_.input_attrs == nullptr || context_.output_attrs == nullptr ||
            context_.input_attrs[0].n_elems != MODEL_WIDTH * MODEL_HEIGHT * 3 ||
            context_.output_attrs[0].n_elems != OUT_ROWS * OUT_COLS) {
            release_lprnet_model(&context_);
            loaded_ = false;
            throw std::runtime_error("Model tensor shape does not match ARM_final1 LPR pipeline");
        }
        std::cout << "Original ARM_final1 LPR pipeline loaded: input="
                  << context_.model_width << 'x' << context_.model_height
                  << ", output=" << OUT_ROWS << 'x' << OUT_COLS << '\n';
    }

    OriginalVideoPipelineLpr(const OriginalVideoPipelineLpr &) = delete;
    OriginalVideoPipelineLpr &operator=(const OriginalVideoPipelineLpr &) = delete;

    ~OriginalVideoPipelineLpr()
    {
        if (loaded_) {
            release_lprnet_model(&context_);
        }
    }

    bool infer(cv::Mat &plate_image, long long frame_id, int track_id,
               std::string &prediction, double &inference_ms, std::string &error)
    {
        if (plate_image.empty()) {
            error = "empty image";
            return false;
        }
        lprnet_result result;
        const float aspect_ratio = plate_image.rows > 0
                                       ? static_cast<float>(plate_image.cols) /
                                             static_cast<float>(plate_image.rows)
                                       : 0.0f;
        const auto begin = std::chrono::steady_clock::now();
        const int ret = inference_lprnet_model(&context_, plate_image, &result, PLATE_BLUE,
                                               nullptr, aspect_ratio, nullptr, frame_id,
                                               track_id);
        const auto end = std::chrono::steady_clock::now();
        inference_ms = std::chrono::duration<double, std::milli>(end - begin).count();
        if (ret != 0) {
            error = "inference_lprnet_model failed: " + std::to_string(ret);
            return false;
        }
        prediction = result.plate_name;
        return true;
    }

private:
    lprnet_context_t context_{};
    bool loaded_ = false;
};

void add_stats(Stats &destination, const Stats &source)
{
    destination.listed += source.listed;
    destination.processed += source.processed;
    destination.correct += source.correct;
    destination.missing += source.missing;
    destination.read_errors += source.read_errors;
    destination.inference_errors += source.inference_errors;
    destination.label_chars += source.label_chars;
    destination.edit_distance += source.edit_distance;
    destination.inference_ms += source.inference_ms;
}

double exact_accuracy(const Stats &stats)
{
    return stats.processed == 0 ? 0.0
                                : 100.0 * static_cast<double>(stats.correct) /
                                      static_cast<double>(stats.processed);
}

double character_accuracy(const Stats &stats)
{
    if (stats.label_chars == 0) {
        return 0.0;
    }
    const double value = 1.0 - static_cast<double>(stats.edit_distance) /
                                   static_cast<double>(stats.label_chars);
    return 100.0 * std::max(0.0, value);
}

double percentile(std::vector<double> values, double fraction)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(
        std::min<double>(values.size() - 1, fraction * static_cast<double>(values.size() - 1)));
    return values[index];
}

void print_stats(std::ostream &output, const std::string &name, const Stats &stats)
{
    const double average_ms = stats.processed == 0 ? 0.0 : stats.inference_ms / stats.processed;
    output << std::fixed << std::setprecision(3)
           << name << ": listed=" << stats.listed
           << ", processed=" << stats.processed
           << ", correct=" << stats.correct
           << ", exact_accuracy=" << exact_accuracy(stats) << "%"
           << ", char_accuracy=" << character_accuracy(stats) << "%"
           << ", missing=" << stats.missing
           << ", read_errors=" << stats.read_errors
           << ", inference_errors=" << stats.inference_errors
           << ", avg_lpr_pipeline_ms=" << average_ms << '\n';
}

} // namespace

int main(int argc, char **argv)
{
    try {
        const Options options = parse_options(argc, argv);
        if (!fs::is_regular_file(options.model)) {
            throw std::runtime_error("Model does not exist: " + options.model.string());
        }
        if (!fs::is_directory(options.dataset)) {
            throw std::runtime_error("Dataset directory does not exist: " +
                                     options.dataset.string());
        }

        const std::vector<Sample> samples = load_samples(options);
        if (samples.empty()) {
            throw std::runtime_error("No labeled samples found");
        }
        std::cout << "Dataset root: " << options.dataset << '\n'
                  << "Listed samples: " << samples.size() << '\n'
                  << "CSV output: " << options.output << '\n';

        if (!options.output.parent_path().empty()) {
            fs::create_directories(options.output.parent_path());
        }
        std::ofstream csv(options.output, std::ios::binary);
        if (!csv) {
            throw std::runtime_error("Cannot create output CSV: " + options.output.string());
        }
        csv << "\xEF\xBB\xBF";
        csv << "split,image,label,prediction,correct,inference_ms,error\n";

        OriginalVideoPipelineLpr model(options.model);

        const auto first_existing = std::find_if(samples.begin(), samples.end(), [](const Sample &s) {
            return fs::is_regular_file(s.image_path);
        });
        if (first_existing == samples.end()) {
            throw std::runtime_error("All labeled images are missing");
        }
        const cv::Mat warmup_image = cv::imread(first_existing->image_path.string(), cv::IMREAD_COLOR);
        for (int i = 0; i < options.warmup_runs; ++i) {
            std::string prediction;
            std::string error;
            double elapsed_ms = 0.0;
            cv::Mat warmup_copy = warmup_image.clone();
            if (!model.infer(warmup_copy, -1000 - i, -1000 - i,
                             prediction, elapsed_ms, error)) {
                throw std::runtime_error("Warm-up failed: " + error);
            }
        }
        std::cout << "Warm-up runs: " << options.warmup_runs << '\n';

        std::vector<SplitResult> splits;
        std::vector<double> latencies;
        Stats total;
        const auto wall_begin = std::chrono::steady_clock::now();

        for (std::size_t index = 0; index < samples.size(); ++index) {
            const Sample &sample = samples[index];
            auto split_it = std::find_if(splits.begin(), splits.end(), [&](const SplitResult &item) {
                return item.name == sample.split;
            });
            if (split_it == splits.end()) {
                splits.push_back({sample.split, {}});
                split_it = std::prev(splits.end());
            }
            Stats row_stats;
            row_stats.listed = 1;
            std::string prediction;
            std::string error;
            double elapsed_ms = 0.0;

            if (!fs::is_regular_file(sample.image_path)) {
                ++row_stats.missing;
                error = "missing image";
            } else {
                cv::Mat image = cv::imread(sample.image_path.string(), cv::IMREAD_COLOR);
                if (image.empty()) {
                    ++row_stats.read_errors;
                    error = "cv::imread failed";
                } else if (!model.infer(image, static_cast<long long>(index),
                                        static_cast<int>(index), prediction,
                                        elapsed_ms, error)) {
                    ++row_stats.inference_errors;
                } else {
                    ++row_stats.processed;
                    row_stats.inference_ms = elapsed_ms;
                    latencies.push_back(elapsed_ms);
                    if (prediction == sample.label) {
                        ++row_stats.correct;
                    }
                    const auto expected_chars = utf8_characters(sample.label);
                    const auto predicted_chars = utf8_characters(prediction);
                    row_stats.label_chars = expected_chars.size();
                    row_stats.edit_distance = levenshtein_distance(expected_chars, predicted_chars);
                }
            }

            add_stats(split_it->stats, row_stats);
            add_stats(total, row_stats);
            csv << csv_escape(sample.split) << ','
                << csv_escape(sample.image_name) << ','
                << csv_escape(sample.label) << ','
                << csv_escape(prediction) << ','
                << (row_stats.correct == 1 ? 1 : 0) << ','
                << std::fixed << std::setprecision(3) << elapsed_ms << ','
                << csv_escape(error) << '\n';

            if (options.progress_interval > 0 &&
                ((index + 1) % options.progress_interval == 0 || index + 1 == samples.size())) {
                std::cout << "Progress " << (index + 1) << '/' << samples.size()
                          << ", accuracy=" << std::fixed << std::setprecision(2)
                          << exact_accuracy(total) << "%\n";
                csv.flush();
            }
        }

        const auto wall_end = std::chrono::steady_clock::now();
        const double wall_seconds =
            std::chrono::duration<double>(wall_end - wall_begin).count();
        csv.close();

        fs::path summary_path = options.output;
        summary_path.replace_extension(".summary.txt");
        std::ofstream summary(summary_path);
        if (!summary) {
            throw std::runtime_error("Cannot create summary: " + summary_path.string());
        }
        summary << "BlueSpecial RK3568 INT8 board evaluation\n"
                << "model=" << options.model.string() << '\n'
                << "dataset=" << options.dataset.string() << '\n'
                << "csv=" << options.output.string() << '\n';
        for (const SplitResult &split : splits) {
            print_stats(std::cout, split.name, split.stats);
            print_stats(summary, split.name, split.stats);
        }
        print_stats(std::cout, "ALL", total);
        print_stats(summary, "ALL", total);

        const double pipeline_fps = wall_seconds <= 0.0
                                        ? 0.0
                                        : static_cast<double>(total.processed) / wall_seconds;
        const double average_ms = total.processed == 0 ? 0.0 : total.inference_ms / total.processed;
        const double p50_ms = percentile(latencies, 0.50);
        const double p95_ms = percentile(latencies, 0.95);
        const double lpr_fps = average_ms <= 0.0 ? 0.0 : 1000.0 / average_ms;

        auto print_timing = [&](std::ostream &output) {
            output << std::fixed << std::setprecision(3)
                   << "timing: wall_seconds=" << wall_seconds
                   << ", pipeline_fps=" << pipeline_fps
                   << ", avg_lpr_pipeline_ms=" << average_ms
                   << ", p50_lpr_pipeline_ms=" << p50_ms
                   << ", p95_lpr_pipeline_ms=" << p95_ms
                   << ", lpr_pipeline_fps=" << lpr_fps << '\n';
        };
        print_timing(std::cout);
        print_timing(summary);
        summary.close();

        std::cout << "Results written to:\n  " << options.output
                  << "\n  " << summary_path << '\n';
        return total.inference_errors == 0 ? 0 : 4;
    } catch (const std::exception &error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        print_usage(argv[0]);
        return 2;
    }
}
