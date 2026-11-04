#include "lprnet.h"
#include "obb_detector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

const std::vector<std::string> kProvinces = {
    "皖", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑", "苏",
    "浙", "京", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤", "桂", "琼",
    "川", "贵", "云", "藏", "陕", "甘", "青", "宁", "新"
};

const std::vector<std::string> kAlphabets = {
    "A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M",
    "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"
};

const std::vector<std::string> kAds = {
    "A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M",
    "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"
};

struct Options {
    fs::path dataset;
    fs::path manifest = "ccpd2019_random1000_per_subset_seed20260802.txt";
    fs::path output = "ccpd2019_e2e_results.csv";
    fs::path yolo_model = "/userdata/model/yolov8n_obb.rknn";
    fs::path yolo_labels = "/userdata/model/yolov8_obb_labels_list.txt";
    fs::path lpr_blue = "/userdata/model/bluespecial.rknn";
    fs::path lpr_other = "/userdata/model/lprnet.rknn";
    fs::path lpr_double = "/userdata/model/doubleyellow.rknn";
    std::size_t limit = 0;
    std::size_t progress = 100;
    int warmup = 3;
    float iou_threshold = 0.50f;
    float box_threshold = 0.25f;
    float nms_threshold = 0.60f;
};

struct GroundTruth {
    std::string label;
    std::vector<cv::Point2f> polygon;
};

struct Stats {
    std::size_t sampled = 0;
    std::size_t parsed = 0;
    std::size_t image_read = 0;
    std::size_t yolo_ok = 0;
    std::size_t any_detection = 0;
    std::size_t localized = 0;
    std::size_t lpr_attempted = 0;
    std::size_t lpr_nonempty = 0;
    std::size_t exact = 0;
    std::size_t parse_errors = 0;
    std::size_t read_errors = 0;
    std::size_t yolo_errors = 0;
    std::size_t crop_errors = 0;
    std::size_t lpr_errors = 0;
    std::size_t gt_characters = 0;
    std::size_t edit_distance = 0;
    double yolo_ms = 0.0;
    double lpr_ms = 0.0;
    double pipeline_ms = 0.0;
};

struct SplitStats {
    std::string name;
    Stats value;
};

std::string trim(const std::string &value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string &value, char delimiter)
{
    std::vector<std::string> fields;
    std::stringstream stream(value);
    std::string field;
    while (std::getline(stream, field, delimiter)) fields.push_back(field);
    return fields;
}

std::string lower_ascii(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
    });
    return text;
}

std::string csv_escape(const std::string &value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string output = "\"";
    for (char value : value) output += value == '"' ? "\"\"" : std::string(1, value);
    return output + '"';
}

std::vector<std::string> utf8_characters(const std::string &text)
{
    std::vector<std::string> result;
    for (std::size_t offset = 0; offset < text.size();) {
        const unsigned char lead = static_cast<unsigned char>(text[offset]);
        std::size_t length = 1;
        if ((lead & 0xE0U) == 0xC0U) length = 2;
        else if ((lead & 0xF0U) == 0xE0U) length = 3;
        else if ((lead & 0xF8U) == 0xF0U) length = 4;
        if (offset + length > text.size()) length = 1;
        result.push_back(text.substr(offset, length));
        offset += length;
    }
    return result;
}

std::size_t levenshtein(const std::vector<std::string> &left,
                        const std::vector<std::string> &right)
{
    std::vector<std::size_t> previous(right.size() + 1), current(right.size() + 1);
    std::iota(previous.begin(), previous.end(), 0);
    for (std::size_t row = 1; row <= left.size(); ++row) {
        current[0] = row;
        for (std::size_t column = 1; column <= right.size(); ++column) {
            const std::size_t substitution = previous[column - 1] +
                (left[row - 1] == right[column - 1] ? 0U : 1U);
            current[column] = std::min({previous[column] + 1,
                                        current[column - 1] + 1,
                                        substitution});
        }
        previous.swap(current);
    }
    return previous.back();
}

void usage(const char *program)
{
    std::cout
        << "Usage:\n  " << program
        << " --dataset /mnt/usb/CCPD2019"
           " [--manifest ccpd2019_random1000_per_subset_seed20260802.txt]"
           " [--output /mnt/usb/ccpd2019_e2e_results.csv]"
           " [--yolo-model /userdata/model/yolov8n_obb.rknn]"
           " [--yolo-labels /userdata/model/yolov8_obb_labels_list.txt]"
           " [--lpr-blue /userdata/model/bluespecial.rknn]"
           " [--lpr-other /userdata/model/lprnet.rknn]"
           " [--lpr-double /userdata/model/doubleyellow.rknn]"
           " [--limit 20] [--progress 100] [--warmup 3] [--iou 0.5]\n";
}

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string key = argv[index];
        auto value = [&]() -> std::string {
            if (++index >= argc) throw std::runtime_error("Missing value after " + key);
            return argv[index];
        };
        if (key == "--dataset") options.dataset = value();
        else if (key == "--manifest") options.manifest = value();
        else if (key == "--output") options.output = value();
        else if (key == "--yolo-model") options.yolo_model = value();
        else if (key == "--yolo-labels") options.yolo_labels = value();
        else if (key == "--lpr-blue") options.lpr_blue = value();
        else if (key == "--lpr-other") options.lpr_other = value();
        else if (key == "--lpr-double") options.lpr_double = value();
        else if (key == "--limit") options.limit = std::stoull(value());
        else if (key == "--progress") options.progress = std::stoull(value());
        else if (key == "--warmup") options.warmup = std::stoi(value());
        else if (key == "--iou") options.iou_threshold = std::stof(value());
        else if (key == "--box-threshold") options.box_threshold = std::stof(value());
        else if (key == "--nms-threshold") options.nms_threshold = std::stof(value());
        else if (key == "--help" || key == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else throw std::runtime_error("Unknown option: " + key);
    }
    if (options.dataset.empty()) throw std::runtime_error("--dataset is required");
    if (options.warmup < 0) throw std::runtime_error("--warmup cannot be negative");
    return options;
}

bool valid_relative_path(const fs::path &path)
{
    if (path.empty() || path.is_absolute()) return false;
    for (const auto &part : path) if (part == "..") return false;
    return true;
}

std::vector<fs::path> load_manifest(const Options &options,
                                    std::map<std::string, std::size_t> &counts)
{
    std::ifstream input(options.manifest);
    if (!input) throw std::runtime_error("Cannot open manifest: " + options.manifest.string());
    std::vector<fs::path> paths;
    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const fs::path relative = fs::path(line).lexically_normal();
        if (!valid_relative_path(relative))
            throw std::runtime_error("Unsafe manifest path: " + line);
        paths.push_back(relative);
        ++counts[relative.begin()->string()];
        if (options.limit > 0 && paths.size() >= options.limit) break;
    }
    if (paths.empty()) throw std::runtime_error("Manifest contains no image paths");
    return paths;
}

bool parse_point(const std::string &text, cv::Point2f &point)
{
    const auto coordinates = split(text, '&');
    if (coordinates.size() != 2) return false;
    try {
        point.x = static_cast<float>(std::stoi(coordinates[0]));
        point.y = static_cast<float>(std::stoi(coordinates[1]));
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_ccpd_filename(const fs::path &path, GroundTruth &truth, std::string &error)
{
    const auto fields = split(path.stem().string(), '-');
    if (fields.size() < 5) {
        error = "filename has fewer than five CCPD fields";
        return false;
    }
    const auto codes_text = split(fields[4], '_');
    if (codes_text.size() != 7) {
        error = "plate field does not contain seven indices";
        return false;
    }
    try {
        std::vector<int> codes;
        for (const auto &text : codes_text) codes.push_back(std::stoi(text));
        if (codes[0] < 0 || codes[0] >= static_cast<int>(kProvinces.size()) ||
            codes[1] < 0 || codes[1] >= static_cast<int>(kAlphabets.size())) {
            throw std::out_of_range("province/alphabet index");
        }
        truth.label = kProvinces[codes[0]] + kAlphabets[codes[1]];
        for (std::size_t i = 2; i < codes.size(); ++i) {
            if (codes[i] < 0 || codes[i] >= static_cast<int>(kAds.size()))
                throw std::out_of_range("ads index");
            truth.label += kAds[codes[i]];
        }
    } catch (const std::exception &exception) {
        error = std::string("invalid plate indices: ") + exception.what();
        return false;
    }

    const auto points = split(fields[3], '_');
    if (points.size() != 4) {
        error = "vertex field does not contain four points";
        return false;
    }
    truth.polygon.clear();
    for (const auto &text : points) {
        cv::Point2f point;
        if (!parse_point(text, point)) {
            error = "invalid vertex: " + text;
            return false;
        }
        truth.polygon.push_back(point);
    }
    std::vector<cv::Point2f> truth_hull;
    cv::convexHull(truth.polygon, truth_hull);
    truth.polygon.swap(truth_hull);
    return truth.polygon.size() >= 3;
}

bool is_plate_class(int class_id)
{
    return class_id >= 0 && class_id <= 3;
}

PlateType yolo_class_to_plate_type(int class_id, const char *class_name)
{
    const std::string label = lower_ascii(class_name ? class_name : "");
    if (label.find("double") != std::string::npos || label.find("双层") != std::string::npos)
        return PLATE_YELLOW_DOUBLE;
    if (label.find("green") != std::string::npos || label.find("新能源") != std::string::npos)
        return PLATE_GREEN;
    if (label.find("yellow") != std::string::npos || label.find("黄") != std::string::npos)
        return PLATE_YELLOW_SINGLE;
    if (label.find("blue") != std::string::npos || label.find("蓝") != std::string::npos)
        return PLATE_BLUE;
    if (class_id == 1) return PLATE_YELLOW_SINGLE;
    if (class_id == 2) return PLATE_GREEN;
    if (class_id == 3) return PLATE_YELLOW_DOUBLE;
    return PLATE_BLUE;
}

void normalize_plate_obb(const ObbBox &box, float &width, float &height, float &angle)
{
    width = box.w;
    height = box.h;
    angle = box.angle;
    if (width < height) {
        std::swap(width, height);
        angle -= static_cast<float>(CV_PI * 0.5);
    }
}

std::vector<cv::Point2f> obb_polygon(const ObbBox &box)
{
    float width = 0.0f, height = 0.0f, angle = 0.0f;
    normalize_plate_obb(box, width, height, angle);
    const float half_width = width * 0.5f;
    const float half_height = height * 0.5f;
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    std::vector<cv::Point2f> points = {
        {box.cx - half_width * cosine + half_height * sine,
         box.cy - half_width * sine - half_height * cosine},
        {box.cx - half_width * cosine - half_height * sine,
         box.cy - half_width * sine + half_height * cosine},
        {box.cx + half_width * cosine - half_height * sine,
         box.cy + half_width * sine + half_height * cosine},
        {box.cx + half_width * cosine + half_height * sine,
         box.cy + half_width * sine - half_height * cosine}
    };
    std::vector<cv::Point2f> hull;
    cv::convexHull(points, hull);
    return hull;
}

float polygon_iou(const std::vector<cv::Point2f> &left,
                  const std::vector<cv::Point2f> &right)
{
    if (left.size() < 3 || right.size() < 3) return 0.0f;
    std::vector<cv::Point2f> intersection;
    const float intersection_area = cv::intersectConvexConvex(left, right, intersection, true);
    const float left_area = static_cast<float>(std::fabs(cv::contourArea(left)));
    const float right_area = static_cast<float>(std::fabs(cv::contourArea(right)));
    const float union_area = left_area + right_area - intersection_area;
    return union_area > 0.0f ? intersection_area / union_area : 0.0f;
}

// Verbatim geometry used by ARM_final1/detection_worker.cpp after YOLO OBB.
bool rectify_plate_from_obb(const cv::Mat &bgr_frame, const ObbBox &box, cv::Mat &plate)
{
    if (bgr_frame.empty()) return false;
    float width = 0.0f, height = 0.0f, angle = 0.0f;
    normalize_plate_obb(box, width, height, angle);
    const float half_width = width * 0.5f;
    const float half_height = height * 0.5f;
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    cv::Point2f source[4] = {
        {box.cx - half_width * cosine + half_height * sine,
         box.cy - half_width * sine - half_height * cosine},
        {box.cx - half_width * cosine - half_height * sine,
         box.cy - half_width * sine + half_height * cosine},
        {box.cx + half_width * cosine - half_height * sine,
         box.cy + half_width * sine + half_height * cosine},
        {box.cx + half_width * cosine + half_height * sine,
         box.cy + half_width * sine - half_height * cosine}
    };
    const int destination_width = std::max(1, static_cast<int>(std::round(width)));
    const int destination_height = std::max(1, static_cast<int>(std::round(height)));
    cv::Point2f destination[4] = {
        {0.0f, 0.0f},
        {0.0f, static_cast<float>(destination_height - 1)},
        {static_cast<float>(destination_width - 1),
         static_cast<float>(destination_height - 1)},
        {static_cast<float>(destination_width - 1), 0.0f}
    };
    const cv::Mat transform = cv::getPerspectiveTransform(source, destination);
    cv::warpPerspective(bgr_frame, plate, transform,
                        cv::Size(destination_width, destination_height),
                        cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    return !plate.empty();
}

class LprModels {
public:
    explicit LprModels(const Options &options)
    {
        if (init_lprnet_model(options.lpr_blue.string().c_str(), &blue_) != 0)
            throw std::runtime_error("Cannot load blue LPR model: " + options.lpr_blue.string());
        blue_loaded_ = true;
        if (init_lprnet_model(options.lpr_other.string().c_str(), &other_) != 0) {
            release_lprnet_model(&blue_);
            blue_loaded_ = false;
            throw std::runtime_error("Cannot load other-plate LPR model: " + options.lpr_other.string());
        }
        other_loaded_ = true;
        if (!options.lpr_double.empty() && fs::is_regular_file(options.lpr_double)) {
            if (init_lprnet_model(options.lpr_double.string().c_str(), &double_) == 0)
                double_loaded_ = true;
            else
                std::cerr << "Warning: double-yellow LPR model failed to load\n";
        }
    }

    ~LprModels()
    {
        if (double_loaded_) release_lprnet_model(&double_);
        if (other_loaded_) release_lprnet_model(&other_);
        if (blue_loaded_) release_lprnet_model(&blue_);
    }

    lprnet_context_t *select(PlateType type)
    {
        return type == PLATE_BLUE ? &blue_ : &other_;
    }

    lprnet_context_t *double_yellow()
    {
        return double_loaded_ ? &double_ : nullptr;
    }

private:
    lprnet_context_t blue_{};
    lprnet_context_t other_{};
    lprnet_context_t double_{};
    bool blue_loaded_ = false;
    bool other_loaded_ = false;
    bool double_loaded_ = false;
};

void add_stats(Stats &target, const Stats &value)
{
    target.sampled += value.sampled;
    target.parsed += value.parsed;
    target.image_read += value.image_read;
    target.yolo_ok += value.yolo_ok;
    target.any_detection += value.any_detection;
    target.localized += value.localized;
    target.lpr_attempted += value.lpr_attempted;
    target.lpr_nonempty += value.lpr_nonempty;
    target.exact += value.exact;
    target.parse_errors += value.parse_errors;
    target.read_errors += value.read_errors;
    target.yolo_errors += value.yolo_errors;
    target.crop_errors += value.crop_errors;
    target.lpr_errors += value.lpr_errors;
    target.gt_characters += value.gt_characters;
    target.edit_distance += value.edit_distance;
    target.yolo_ms += value.yolo_ms;
    target.lpr_ms += value.lpr_ms;
    target.pipeline_ms += value.pipeline_ms;
}

double percentage(std::size_t numerator, std::size_t denominator)
{
    return denominator == 0 ? 0.0 : 100.0 * static_cast<double>(numerator) / denominator;
}

void report(std::ostream &output, const std::string &name, const Stats &stats)
{
    output << std::fixed << std::setprecision(3)
           << name
           << ": samples=" << stats.sampled
           << ", detection_recall_iou=" << percentage(stats.localized, stats.sampled) << "%"
           << ", conditional_lpr_exact=" << percentage(stats.exact, stats.localized) << "%"
           << ", end_to_end_exact=" << percentage(stats.exact, stats.sampled) << "%"
           << ", end_to_end_char="
           << percentage(stats.gt_characters - std::min(stats.gt_characters, stats.edit_distance),
                         stats.gt_characters) << "%"
           << ", any_box=" << stats.any_detection
           << ", localized=" << stats.localized
           << ", exact=" << stats.exact
           << ", parse/read/yolo/crop/lpr_errors="
           << stats.parse_errors << '/' << stats.read_errors << '/' << stats.yolo_errors
           << '/' << stats.crop_errors << '/' << stats.lpr_errors
           << ", avg_yolo_ms=" << (stats.yolo_ok ? stats.yolo_ms / stats.yolo_ok : 0.0)
           << ", avg_lpr_ms=" << (stats.lpr_attempted ? stats.lpr_ms / stats.lpr_attempted : 0.0)
           << ", avg_pipeline_ms=" << (stats.image_read ? stats.pipeline_ms / stats.image_read : 0.0)
           << '\n';
}

} // namespace

int main(int argc, char **argv)
{
    try {
        Options options = parse_options(argc, argv);
        if (!fs::is_directory(options.dataset))
            throw std::runtime_error("Dataset does not exist: " + options.dataset.string());
        if (!fs::is_regular_file(options.yolo_model) &&
            fs::is_regular_file("/userdata/model/yolov8n_obb_plate.rknn"))
            options.yolo_model = "/userdata/model/yolov8n_obb_plate.rknn";
        if (!fs::is_regular_file(options.yolo_labels) &&
            fs::is_regular_file("/userdata/model/yolov8_obb_plate_labels_list.txt"))
            options.yolo_labels = "/userdata/model/yolov8_obb_plate_labels_list.txt";
        for (const fs::path &required : {options.yolo_model, options.yolo_labels,
                                         options.lpr_blue, options.lpr_other}) {
            if (!fs::is_regular_file(required))
                throw std::runtime_error("Required model/file does not exist: " + required.string());
        }

        std::map<std::string, std::size_t> manifest_counts;
        const auto relative_paths = load_manifest(options, manifest_counts);
        std::cout << "Manifest samples=" << relative_paths.size() << '\n';
        for (const auto &entry : manifest_counts)
            std::cout << "  " << entry.first << '=' << entry.second << '\n';

        ObbModelParams detector_params;
        detector_params.boxThreshold = options.box_threshold;
        detector_params.nmsThreshold = options.nms_threshold;
        ObbDetector detector;
        if (!detector.loadModel(options.yolo_model.string().c_str(),
                                options.yolo_labels.string().c_str(), detector_params))
            throw std::runtime_error("Failed to load original YOLO OBB detector");
        if (detector.numClasses() != 4)
            throw std::runtime_error("YOLO label list must contain four plate classes");
        LprModels lpr_models(options);

        if (!options.output.parent_path().empty())
            fs::create_directories(options.output.parent_path());
        std::ofstream csv(options.output, std::ios::binary);
        if (!csv) throw std::runtime_error("Cannot create CSV: " + options.output.string());
        csv << "\xEF\xBB\xBF";
        csv << "subset,image,ground_truth,prediction,correct,plate_boxes,best_iou,"
               "localized,yolo_class,yolo_label,confidence,yolo_ms,lpr_ms,pipeline_ms,error\n";

        // Warm up using the first manifest image; this never affects metrics.
        cv::Mat warmup_bgr = cv::imread((options.dataset / relative_paths.front()).string(),
                                        cv::IMREAD_COLOR);
        for (int run = 0; run < options.warmup && !warmup_bgr.empty(); ++run) {
            cv::Mat rgb;
            cv::cvtColor(warmup_bgr, rgb, cv::COLOR_BGR2RGB);
            DetectResult detections;
            detector.detect(rgb.data, rgb.cols, rgb.rows, detections);
            if (!detections.boxes.empty() && is_plate_class(detections.boxes.front().classId)) {
                cv::Mat crop;
                if (rectify_plate_from_obb(warmup_bgr, detections.boxes.front(), crop)) {
                    const PlateType type = yolo_class_to_plate_type(
                        detections.boxes.front().classId,
                        detector.className(detections.boxes.front().classId));
                    lprnet_result result;
                    inference_lprnet_model(lpr_models.select(type), crop, &result, type,
                                           lpr_models.double_yellow(),
                                           detections.boxes.front().w /
                                               std::max(detections.boxes.front().h, 1.0f),
                                           nullptr, -1000 - run, -1000 - run);
                }
            }
        }

        Stats total;
        std::vector<SplitStats> splits;
        const auto wall_begin = std::chrono::steady_clock::now();

        for (std::size_t image_index = 0; image_index < relative_paths.size(); ++image_index) {
            const fs::path &relative = relative_paths[image_index];
            const std::string subset = relative.begin()->string();
            auto split_stats = std::find_if(splits.begin(), splits.end(), [&](const SplitStats &item) {
                return item.name == subset;
            });
            if (split_stats == splits.end()) {
                splits.push_back({subset, {}});
                split_stats = std::prev(splits.end());
            }

            Stats row;
            row.sampled = 1;
            GroundTruth truth;
            std::string error;
            std::string prediction;
            float best_iou = 0.0f;
            int best_box = -1;
            int best_class = -1;
            float best_confidence = 0.0f;
            std::string best_label;
            std::size_t plate_boxes = 0;

            if (!parse_ccpd_filename(relative.filename(), truth, error)) {
                ++row.parse_errors;
            } else {
                ++row.parsed;
                const auto gt_chars = utf8_characters(truth.label);
                row.gt_characters = gt_chars.size();
                const fs::path image_path = options.dataset / relative;
                cv::Mat bgr = cv::imread(image_path.string(), cv::IMREAD_COLOR);
                if (bgr.empty()) {
                    ++row.read_errors;
                    error = "cv::imread failed";
                } else {
                    ++row.image_read;
                    const auto pipeline_begin = std::chrono::steady_clock::now();
                    cv::Mat rgb;
                    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
                    DetectResult detections;
                    const auto yolo_begin = std::chrono::steady_clock::now();
                    const bool detected = detector.detect(rgb.data, rgb.cols, rgb.rows, detections);
                    const auto yolo_end = std::chrono::steady_clock::now();
                    row.yolo_ms = std::chrono::duration<double, std::milli>(
                        yolo_end - yolo_begin).count();
                    if (!detected) {
                        ++row.yolo_errors;
                        error = "ObbDetector::detect failed";
                    } else {
                        ++row.yolo_ok;
                        std::vector<std::string> box_predictions(detections.boxes.size());
                        std::vector<bool> box_lpr_ok(detections.boxes.size(), false);
                        for (std::size_t box_index = 0; box_index < detections.boxes.size(); ++box_index) {
                            ObbBox &box = detections.boxes[box_index];
                            if (!is_plate_class(box.classId)) continue;
                            ++plate_boxes;
                            const float iou = polygon_iou(obb_polygon(box), truth.polygon);
                            if (iou > best_iou) {
                                best_iou = iou;
                                best_box = static_cast<int>(box_index);
                            }

                            cv::Mat plate;
                            if (!rectify_plate_from_obb(bgr, box, plate)) {
                                ++row.crop_errors;
                                continue;
                            }
                            const PlateType type = yolo_class_to_plate_type(
                                box.classId, detector.className(box.classId));
                            float normalized_width = 0.0f, normalized_height = 0.0f, angle = 0.0f;
                            normalize_plate_obb(box, normalized_width, normalized_height, angle);
                            lprnet_result lpr_result;
                            const auto lpr_begin = std::chrono::steady_clock::now();
                            const int lpr_ret = inference_lprnet_model(
                                lpr_models.select(type), plate, &lpr_result, type,
                                lpr_models.double_yellow(),
                                normalized_width / std::max(normalized_height, 1.0f),
                                nullptr, static_cast<long long>(image_index),
                                static_cast<int>(image_index * 100 + box_index));
                            const auto lpr_end = std::chrono::steady_clock::now();
                            row.lpr_ms += std::chrono::duration<double, std::milli>(
                                lpr_end - lpr_begin).count();
                            ++row.lpr_attempted;
                            if (lpr_ret != 0) {
                                ++row.lpr_errors;
                                continue;
                            }
                            box_lpr_ok[box_index] = true;
                            box_predictions[box_index] = lpr_result.plate_name;
                            if (!lpr_result.plate_name.empty()) ++row.lpr_nonempty;
                        }

                        if (plate_boxes > 0) ++row.any_detection;
                        if (best_box >= 0) {
                            const ObbBox &box = detections.boxes[static_cast<std::size_t>(best_box)];
                            best_class = box.classId;
                            best_confidence = box.confidence;
                            best_label = detector.className(box.classId);
                            if (box_lpr_ok[static_cast<std::size_t>(best_box)])
                                prediction = box_predictions[static_cast<std::size_t>(best_box)];
                        }
                        if (best_iou >= options.iou_threshold) {
                            ++row.localized;
                            if (prediction == truth.label) ++row.exact;
                        }
                    }
                    const auto pipeline_end = std::chrono::steady_clock::now();
                    row.pipeline_ms = std::chrono::duration<double, std::milli>(
                        pipeline_end - pipeline_begin).count();
                }
                row.edit_distance = levenshtein(gt_chars, utf8_characters(prediction));
            }

            add_stats(split_stats->value, row);
            add_stats(total, row);
            csv << csv_escape(subset) << ',' << csv_escape(relative.generic_string()) << ','
                << csv_escape(truth.label) << ',' << csv_escape(prediction) << ','
                << row.exact << ',' << plate_boxes << ',' << std::fixed << std::setprecision(4)
                << best_iou << ',' << row.localized << ',' << best_class << ','
                << csv_escape(best_label) << ',' << best_confidence << ','
                << std::setprecision(3) << row.yolo_ms << ',' << row.lpr_ms << ','
                << row.pipeline_ms << ',' << csv_escape(error) << '\n';

            if (options.progress > 0 &&
                ((image_index + 1) % options.progress == 0 ||
                 image_index + 1 == relative_paths.size())) {
                std::cout << "Progress " << image_index + 1 << '/' << relative_paths.size()
                          << ", detect=" << std::fixed << std::setprecision(2)
                          << percentage(total.localized, total.sampled)
                          << "%, e2e=" << percentage(total.exact, total.sampled) << "%\n";
                csv.flush();
            }
        }

        const double wall_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - wall_begin).count();
        csv.close();
        fs::path summary_path = options.output;
        summary_path.replace_extension(".summary.txt");
        std::ofstream summary(summary_path);
        summary << "CCPD2019 original YOLO OBB -> rectify -> LPR pipeline evaluation\n"
                << "dataset=" << options.dataset.string() << '\n'
                << "manifest=" << options.manifest.string() << '\n'
                << "yolo_model=" << options.yolo_model.string() << '\n'
                << "lpr_blue=" << options.lpr_blue.string() << '\n'
                << "iou_threshold=" << options.iou_threshold << '\n';
        for (const auto &entry : splits) {
            report(std::cout, entry.name, entry.value);
            report(summary, entry.name, entry.value);
        }
        report(std::cout, "ALL", total);
        report(summary, "ALL", total);
        const double fps = wall_seconds > 0.0 ? total.sampled / wall_seconds : 0.0;
        std::cout << "wall_seconds=" << wall_seconds << ", end_to_end_fps=" << fps << '\n';
        summary << "wall_seconds=" << wall_seconds << ", end_to_end_fps=" << fps << '\n';
        summary.close();
        std::cout << "Results:\n  " << options.output << "\n  " << summary_path << '\n';
        return total.yolo_errors == 0 && total.lpr_errors == 0 ? 0 : 4;
    } catch (const std::exception &error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        usage(argv[0]);
        return 2;
    }
}
