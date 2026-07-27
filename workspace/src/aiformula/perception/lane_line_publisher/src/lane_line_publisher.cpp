#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_cpp/camera.hpp"
#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/tf2_transform.hpp"

namespace aiformula {

enum { LEFT, RIGHT, CENTER, NUM_LANE_LINES };

class LaneLine {
public:
    std::vector<cv::Point> pixels;
    std::vector<Eigen::Vector3d> points;
};

class LaneLines {
public:
    LaneLine left;
    LaneLine right;
    LaneLine center;
};

class LaneLinePublisher : public rclcpp::Node {
public:
    LaneLinePublisher();
    ~LaneLinePublisher() override = default;

private:
    void initMembers();
    void initConnections();

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const;
    void findLaneLines(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                       LaneLines& lane_lines) const;
    void publishAnnotatedMask(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                              const LaneLines& lane_lines) const;
    void publishContourPoints(const std::vector<std::vector<Eigen::Vector3d>>& contour_points,
                              const builtin_interfaces::msg::Time& timestamp) const;
    void publishLaneLines(const LaneLines& lane_lines, const builtin_interfaces::msg::Time& timestamp) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_mask_image_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lane_line_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> contour_point_pubs_;

    bool debug_;
    std::string vehicle_frame_id_;
    double xmin_, xmax_, ymin_, ymax_, spacing_;
    cv::Mat camera_matrix_;
    tf2::Transform vehicle_T_camera_;
};

namespace {

constexpr double REFERENCE_IMAGE_WIDTH = 1920.0;
constexpr double REFERENCE_IMAGE_HEIGHT = 1080.0;
constexpr double PI = 3.14159265358979323846;

struct ImageLineFit {
    bool valid = false;
    double a = 0.0;
    double b = 0.0;
    int lost_count = 0;
};

struct DynamicRoiConfig {
    int min_area = 100;
    int max_lost_frames = 5;
    double init_top_crop_ratio = 0.50;
    int tolerance = 10;
    double line_update_alpha = 0.90;
    double min_row_coverage_ratio = 0.10;
    int min_component_pixels = 180;
    double min_component_row_span_ratio = 0.05;
    int min_roi_component_area = 50;
    double near_field_start_ratio = 0.78;
    double min_projected_line_span_m = 1.0;
    int min_published_points = 3;
    double min_line_angle_deg = 7.0;
    double max_line_angle_deg = 173.0;
    double min_abs_line_slope = 0.08;
    double max_abs_line_slope = 10.0;
    double lane_path_max_run_width_ratio = 0.09;
    double lane_path_max_deviation_ratio = 0.06;
    int lane_path_min_points = 12;
    int lane_path_refine_iterations = 2;
    double tracking_recovery_bottom_ratio = 0.78;
    int tracking_recovery_bottom_advantage_rows = 12;
    double tracking_jump_margin_multiplier = 4.0;
    double tracking_jump_image_width_ratio = 0.40;
    double global_roi_top_row_ratio = 0.58;
    double global_roi_rect_top_ratio = 0.65;
    double global_roi_top_left_ratio = 0.30;
    double global_roi_top_right_ratio = 0.70;
    double min_lane_spacing_far = 40.0;
    double min_lane_spacing_near = 140.0;
    double xmin = 0.0;
    double xmax = 10.0;
    double ymin = -4.0;
    double ymax = 4.0;
    double spacing = 0.5;

    double marginFar() const { return std::max(60.0, 1.3 * static_cast<double>(std::max(1, tolerance))); }
    double marginNear() const { return std::max(180.0, 4.0 * static_cast<double>(std::max(1, tolerance))); }
};

struct SideDebugInfo {
    bool use_tracking_roi = false;
    ImageLineFit line;
    std::vector<cv::Point> roi_polygon;
    std::vector<cv::Point> extracted_pixels;
};

struct SideResult {
    std::string roi_mode = "init";
    std::string status = "not_processed";
    std::vector<cv::Point> component_pixels;
    std::vector<cv::Point> pixels;
    ImageLineFit current_line;
    double angle_deg = 0.0;
    double row_coverage_ratio = 0.0;
    int component_row_span = 0;
    int near_field_pixels = 0;
    int near_field_rows = 0;
    double image_fit_rmse_px = 0.0;
    int lost_count = 0;
};

struct FrameResult {
    cv::Mat cleaned_mask;
    std::vector<cv::Point> global_roi_polygon;
    std::array<SideResult, NUM_LANE_LINES> side_results;
};

class DynamicRoiProcessor;

std::unique_ptr<DynamicRoiProcessor> g_dynamic_roi_processor;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr g_dynamic_roi_image_pub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr g_linear_fit_image_pub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr g_vehicle_fit_image_pub;

int clampInt(const int value, const int low, const int high) { return std::max(low, std::min(value, high)); }

int topRow(const int rows, const DynamicRoiConfig& cfg) {
    return clampInt(static_cast<int>(std::lround(static_cast<double>(rows) * cfg.init_top_crop_ratio)), 0,
                    std::max(0, rows - 1));
}

double imageWidthScale(const int cols) { return std::max(0.05, static_cast<double>(cols) / REFERENCE_IMAGE_WIDTH); }

double imageAreaScale(const cv::Size& size) {
    return std::max(0.01, (static_cast<double>(size.area())) / (REFERENCE_IMAGE_WIDTH * REFERENCE_IMAGE_HEIGHT));
}

int scaledMinArea(const cv::Size& size, const DynamicRoiConfig& cfg) {
    return std::max(3, static_cast<int>(std::lround(static_cast<double>(cfg.min_area) * imageAreaScale(size))));
}

double scaledPixelWidth(const double value, const int cols) { return value * imageWidthScale(cols); }

double ratioAtRow(const int row, const int rows, const DynamicRoiConfig& cfg) {
    const int v_top = topRow(rows, cfg);
    const int v_bottom = std::max(v_top + 1, rows - 1);
    const double ratio = (static_cast<double>(row) - static_cast<double>(v_top)) /
                         static_cast<double>(v_bottom - v_top);
    return std::max(0.0, std::min(1.0, ratio));
}

double marginAtRow(const int row, const int rows, const int cols, const DynamicRoiConfig& cfg) {
    const double ratio = ratioAtRow(row, rows, cfg);
    const double far = scaledPixelWidth(cfg.marginFar(), cols);
    const double near = scaledPixelWidth(cfg.marginNear(), cols);
    return far + (near - far) * ratio;
}

double laneSpacingAtRow(const int row, const int rows, const int cols, const DynamicRoiConfig& cfg) {
    const double ratio = ratioAtRow(row, rows, cfg);
    const double far = scaledPixelWidth(cfg.min_lane_spacing_far, cols);
    const double near = scaledPixelWidth(cfg.min_lane_spacing_near, cols);
    return far + (near - far) * ratio;
}

cv::Mat toBinaryMask(const cv::Mat& mask) {
    cv::Mat gray;
    if (mask.channels() == 3) {
        cv::cvtColor(mask, gray, cv::COLOR_BGR2GRAY);
    } else if (mask.channels() == 4) {
        cv::cvtColor(mask, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = mask;
    }

    cv::Mat gray_u8;
    if (gray.type() != CV_8U) {
        gray.convertTo(gray_u8, CV_8U);
    } else {
        gray_u8 = gray;
    }

    cv::Mat binary;
    cv::threshold(gray_u8, binary, 0, 255, cv::THRESH_BINARY);
    return binary;
}

cv::Mat removeSmallComponents(const cv::Mat& binary_mask, const int min_area) {
    cv::Mat labels, stats, centroids;
    const int num_labels = cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8, CV_32S);

    cv::Mat cleaned = cv::Mat::zeros(binary_mask.size(), CV_8U);
    for (int label = 1; label < num_labels; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area >= min_area) cleaned.setTo(255, labels == label);
    }
    return cleaned;
}

std::vector<cv::Point> buildGlobalRoiPolygon(const int rows, const int cols, const DynamicRoiConfig& cfg) {
    const int v_top = clampInt(static_cast<int>(std::lround(rows * cfg.global_roi_top_row_ratio)), 0, rows - 1);
    const int v_rect_top =
        clampInt(static_cast<int>(std::lround(rows * cfg.global_roi_rect_top_ratio)), v_top, rows - 1);
    const int u_top_left =
        clampInt(static_cast<int>(std::lround(cols * cfg.global_roi_top_left_ratio)), 0, cols - 1);
    const int u_top_right =
        clampInt(static_cast<int>(std::lround(cols * cfg.global_roi_top_right_ratio)), 0, cols - 1);

    return {
        cv::Point(0, rows - 1),
        cv::Point(cols - 1, rows - 1),
        cv::Point(cols - 1, v_rect_top),
        cv::Point(u_top_right, v_top),
        cv::Point(u_top_left, v_top),
        cv::Point(0, v_rect_top),
    };
}

std::pair<cv::Mat, std::vector<cv::Point>> applyGlobalImageRoiCrop(const cv::Mat& cleaned_mask,
                                                                   const DynamicRoiConfig& cfg) {
    const int rows = cleaned_mask.rows;
    const int cols = cleaned_mask.cols;
    auto polygon = buildGlobalRoiPolygon(rows, cols, cfg);
    cv::Mat roi_mask = cv::Mat::zeros(cleaned_mask.size(), CV_8U);
    const std::vector<std::vector<cv::Point>> polygons = {polygon};
    cv::fillPoly(roi_mask, polygons, 255);

    cv::Mat cropped;
    cv::bitwise_and(cleaned_mask, roi_mask, cropped);
    return {cropped, polygon};
}

std::pair<int, int> baseSideBoundsAtRow(const int side, const int row, const int rows, const int cols,
                                        const DynamicRoiConfig& cfg) {
    const double center_u = 0.5 * static_cast<double>(cols - 1);
    const double half_gap = 0.5 * laneSpacingAtRow(row, rows, cols, cfg);
    if (side == LEFT) {
        return {0, clampInt(static_cast<int>(std::floor(center_u - half_gap)), 0, cols - 1)};
    }
    return {clampInt(static_cast<int>(std::ceil(center_u + half_gap)), 0, cols - 1), cols - 1};
}

std::pair<cv::Mat, std::vector<cv::Point>> buildInitRoi(const cv::Size& size, const int side,
                                                        const ImageLineFit& opposite_line,
                                                        const DynamicRoiConfig& cfg) {
    const int rows = size.height;
    const int cols = size.width;
    const int v_top = topRow(rows, cfg);
    cv::Mat roi_mask = cv::Mat::zeros(size, CV_8U);
    std::vector<cv::Point> left_boundary;
    std::vector<cv::Point> right_boundary;

    for (int row = v_top; row < rows; ++row) {
        auto [u_min, u_max] = baseSideBoundsAtRow(side, row, rows, cols, cfg);
        if (opposite_line.valid) {
            const double gap = laneSpacingAtRow(row, rows, cols, cfg);
            const double opposite_u = opposite_line.a * static_cast<double>(row) + opposite_line.b;
            if (side == LEFT) {
                u_max = std::min(u_max, clampInt(static_cast<int>(std::floor(opposite_u - gap)), 0, cols - 1));
            } else {
                u_min = std::max(u_min, clampInt(static_cast<int>(std::ceil(opposite_u + gap)), 0, cols - 1));
            }
        }

        if (u_min <= u_max) {
            roi_mask.row(row).colRange(u_min, u_max + 1).setTo(255);
            left_boundary.emplace_back(u_min, row);
            right_boundary.emplace_back(u_max, row);
        }
    }

    std::vector<cv::Point> polygon = left_boundary;
    polygon.insert(polygon.end(), right_boundary.rbegin(), right_boundary.rend());
    return {roi_mask, polygon};
}

std::pair<cv::Mat, std::vector<cv::Point>> buildTrackingRoi(const cv::Size& size, const ImageLineFit& line,
                                                            const int side, const ImageLineFit& opposite_line,
                                                            const DynamicRoiConfig& cfg) {
    const int rows = size.height;
    const int cols = size.width;
    const int v_top = topRow(rows, cfg);
    cv::Mat roi_mask = cv::Mat::zeros(size, CV_8U);
    std::vector<cv::Point> left_boundary;
    std::vector<cv::Point> right_boundary;

    for (int row = v_top; row < rows; ++row) {
        const double u_pred = line.a * static_cast<double>(row) + line.b;
        const double margin = marginAtRow(row, rows, cols, cfg);
        int u_min = clampInt(static_cast<int>(std::floor(u_pred - margin)), 0, cols - 1);
        int u_max = clampInt(static_cast<int>(std::ceil(u_pred + margin)), 0, cols - 1);

        const auto [base_min, base_max] = baseSideBoundsAtRow(side, row, rows, cols, cfg);
        u_min = std::max(u_min, base_min);
        u_max = std::min(u_max, base_max);

        if (opposite_line.valid) {
            const double gap = laneSpacingAtRow(row, rows, cols, cfg);
            const double opposite_u = opposite_line.a * static_cast<double>(row) + opposite_line.b;
            if (side == LEFT) {
                u_max = std::min(u_max, clampInt(static_cast<int>(std::floor(opposite_u - gap)), 0, cols - 1));
            } else {
                u_min = std::max(u_min, clampInt(static_cast<int>(std::ceil(opposite_u + gap)), 0, cols - 1));
            }
        }

        if (u_min <= u_max) {
            roi_mask.row(row).colRange(u_min, u_max + 1).setTo(255);
            left_boundary.emplace_back(u_min, row);
            right_boundary.emplace_back(u_max, row);
        }
    }

    std::vector<cv::Point> polygon = left_boundary;
    polygon.insert(polygon.end(), right_boundary.rbegin(), right_boundary.rend());
    return {roi_mask, polygon};
}

std::vector<std::vector<cv::Point>> componentPixels(const cv::Mat& binary_mask, const int min_area) {
    cv::Mat labels, stats, centroids;
    const int num_labels = cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8, CV_32S);

    std::vector<std::vector<cv::Point>> components;
    for (int label = 1; label < num_labels; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < std::max(3, min_area)) continue;

        cv::Mat component_mask = labels == label;
        std::vector<cv::Point> pixels;
        cv::findNonZero(component_mask, pixels);
        if (!pixels.empty()) components.emplace_back(std::move(pixels));
    }
    return components;
}

bool fitImageLineUv(const std::vector<cv::Point>& pixels, ImageLineFit& line, double& angle_deg) {
    if (pixels.size() < 3) return false;

    double sum_u = 0.0;
    double sum_v = 0.0;
    for (const auto& pixel : pixels) {
        sum_u += static_cast<double>(pixel.x);
        sum_v += static_cast<double>(pixel.y);
    }
    const double mean_u = sum_u / static_cast<double>(pixels.size());
    const double mean_v = sum_v / static_cast<double>(pixels.size());

    double var_v = 0.0;
    double cov_vu = 0.0;
    for (const auto& pixel : pixels) {
        const double dv = static_cast<double>(pixel.y) - mean_v;
        const double du = static_cast<double>(pixel.x) - mean_u;
        var_v += dv * dv;
        cov_vu += dv * du;
    }
    if (var_v < 1e-9) return false;

    line.valid = true;
    line.a = cov_vu / var_v;
    line.b = mean_u - line.a * mean_v;
    line.lost_count = 0;

    angle_deg = std::atan2(1.0, line.a) * 180.0 / PI;
    if (angle_deg < 0.0) angle_deg += 180.0;
    return true;
}

double imageFitRmse(const std::vector<cv::Point>& pixels, const ImageLineFit& line) {
    if (pixels.empty() || !line.valid) return std::numeric_limits<double>::infinity();

    double residual_sum = 0.0;
    for (const auto& pixel : pixels) {
        const double predicted = line.a * static_cast<double>(pixel.y) + line.b;
        const double residual = static_cast<double>(pixel.x) - predicted;
        residual_sum += residual * residual;
    }
    return std::sqrt(residual_sum / static_cast<double>(pixels.size()));
}

bool isNearlyHorizontal(const double angle_deg, const DynamicRoiConfig& cfg) {
    return angle_deg < cfg.min_line_angle_deg || angle_deg > cfg.max_line_angle_deg;
}

bool hasInvalidSideGeometry(const int side, const ImageLineFit& line, const double angle_deg,
                            const DynamicRoiConfig& cfg) {
    if (!line.valid) return true;
    if (isNearlyHorizontal(angle_deg, cfg)) return true;
    if (std::abs(line.a) < cfg.min_abs_line_slope || std::abs(line.a) > cfg.max_abs_line_slope) return true;
    if (side == LEFT && line.a >= 0.0) return true;
    if (side == RIGHT && line.a <= 0.0) return true;
    return false;
}

int uniqueRowCount(const std::vector<cv::Point>& pixels) {
    std::vector<int> rows;
    rows.reserve(pixels.size());
    for (const auto& pixel : pixels) rows.emplace_back(pixel.y);
    std::sort(rows.begin(), rows.end());
    return static_cast<int>(std::unique(rows.begin(), rows.end()) - rows.begin());
}

double rowCoverageRatio(const std::vector<cv::Point>& pixels, const int rows, const DynamicRoiConfig& cfg) {
    const int roi_height = std::max(1, rows - topRow(rows, cfg));
    if (pixels.empty()) return 0.0;
    return static_cast<double>(uniqueRowCount(pixels)) / static_cast<double>(roi_height);
}

bool populateComponentSupportMetrics(SideResult& result, const std::vector<cv::Point>& pixels, const cv::Size& image_size,
                                     const DynamicRoiConfig& cfg) {
    const int rows = image_size.height;
    if (static_cast<int>(pixels.size()) < cfg.min_component_pixels) {
        result.status = "too_few_component_pixels";
        return false;
    }

    std::vector<int> unique_rows;
    unique_rows.reserve(pixels.size());
    for (const auto& pixel : pixels) unique_rows.emplace_back(pixel.y);
    std::sort(unique_rows.begin(), unique_rows.end());
    unique_rows.erase(std::unique(unique_rows.begin(), unique_rows.end()), unique_rows.end());

    result.row_coverage_ratio = rowCoverageRatio(pixels, rows, cfg);
    result.component_row_span = unique_rows.empty() ? 0 : unique_rows.back() - unique_rows.front() + 1;

    const int near_start =
        clampInt(static_cast<int>(std::lround(static_cast<double>(rows) * cfg.near_field_start_ratio)), 0,
                 std::max(0, rows - 1));
    std::vector<int> near_rows;
    for (const auto& pixel : pixels) {
        if (pixel.y >= near_start) {
            ++result.near_field_pixels;
            near_rows.emplace_back(pixel.y);
        }
    }
    std::sort(near_rows.begin(), near_rows.end());
    result.near_field_rows = static_cast<int>(std::unique(near_rows.begin(), near_rows.end()) - near_rows.begin());

    const int min_span =
        std::max(1, static_cast<int>(std::lround(static_cast<double>(rows) * cfg.min_component_row_span_ratio)));
    if (result.row_coverage_ratio < cfg.min_row_coverage_ratio) {
        result.status = "insufficient_row_coverage";
        return false;
    }
    if (result.component_row_span < min_span) {
        result.status = "insufficient_component_vertical_span";
        return false;
    }
    return true;
}

double scoreInitComponent(const cv::Size& image_size, const std::vector<cv::Point>& pixels, const int side,
                          const double center_u, const DynamicRoiConfig& cfg) {
    if (pixels.size() < 3) return std::numeric_limits<double>::infinity();

    ImageLineFit line;
    double angle_deg = 0.0;
    if (!fitImageLineUv(pixels, line, angle_deg)) return std::numeric_limits<double>::infinity();
    if (hasInvalidSideGeometry(side, line, angle_deg, cfg))
        return std::numeric_limits<double>::infinity();

    const int rows = image_size.height;
    const int bottom_band_start = static_cast<int>(std::lround(static_cast<double>(rows) * 0.82));
    std::vector<cv::Point> bottom_pixels;
    for (const auto& pixel : pixels) {
        if (pixel.y >= bottom_band_start) bottom_pixels.emplace_back(pixel);
    }
    if (bottom_pixels.empty()) {
        std::vector<int> v_values;
        v_values.reserve(pixels.size());
        for (const auto& pixel : pixels) v_values.emplace_back(pixel.y);
        std::sort(v_values.begin(), v_values.end());
        const int threshold = v_values[static_cast<std::size_t>(0.75 * static_cast<double>(v_values.size() - 1))];
        for (const auto& pixel : pixels) {
            if (pixel.y >= threshold) bottom_pixels.emplace_back(pixel);
        }
    }
    if (bottom_pixels.empty()) return std::numeric_limits<double>::infinity();

    const auto minmax_u =
        std::minmax_element(bottom_pixels.begin(), bottom_pixels.end(),
                            [](const cv::Point& lhs, const cv::Point& rhs) { return lhs.x < rhs.x; });
    const double anchor_distance =
        side == LEFT ? center_u - static_cast<double>(minmax_u.second->x)
                     : static_cast<double>(minmax_u.first->x) - center_u;
    if (anchor_distance < 0.0) return std::numeric_limits<double>::infinity();

    const auto max_v =
        std::max_element(pixels.begin(), pixels.end(),
                         [](const cv::Point& lhs, const cv::Point& rhs) { return lhs.y < rhs.y; });
    const double bottom_reach_penalty = std::max(0.0, static_cast<double>(rows - 1 - max_v->y));
    const double coverage_bonus = std::min(200.0, 0.01 * static_cast<double>(uniqueRowCount(pixels)));
    return anchor_distance + 0.20 * bottom_reach_penalty - coverage_bonus;
}

int bottomRow(const std::vector<cv::Point>& pixels) {
    if (pixels.empty()) return -1;
    const auto max_v =
        std::max_element(pixels.begin(), pixels.end(),
                         [](const cv::Point& lhs, const cv::Point& rhs) { return lhs.y < rhs.y; });
    return max_v->y;
}

std::vector<cv::Point> extractLanePath(const std::vector<cv::Point>& pixels, const ImageLineFit* reference_line,
                                       const int side, const cv::Size& image_size, const DynamicRoiConfig& cfg) {
    if (pixels.empty()) return {};

    const int rows = image_size.height;
    const int cols = image_size.width;
    const int max_run_width = std::max(3, static_cast<int>(std::lround(static_cast<double>(cols) *
                                                                       cfg.lane_path_max_run_width_ratio)));

    std::vector<std::vector<int>> row_u_values(rows);
    for (const auto& pixel : pixels) {
        if (pixel.y >= 0 && pixel.y < rows) row_u_values[pixel.y].emplace_back(pixel.x);
    }

    std::vector<std::pair<int, std::vector<double>>> candidates_by_row;
    candidates_by_row.reserve(rows);
    for (int row = 0; row < rows; ++row) {
        auto& row_u = row_u_values[row];
        if (row_u.empty()) continue;
        std::sort(row_u.begin(), row_u.end());

        std::vector<double> row_candidates;
        std::size_t run_start = 0;
        while (run_start < row_u.size()) {
            std::size_t run_end = run_start;
            int sum_u = row_u[run_start];
            while (run_end + 1 < row_u.size() && row_u[run_end + 1] - row_u[run_end] <= 1) {
                ++run_end;
                sum_u += row_u[run_end];
            }

            const int run_width = row_u[run_end] - row_u[run_start] + 1;
            const int run_count = static_cast<int>(run_end - run_start + 1);
            if (run_width <= max_run_width && run_count > 0) {
                row_candidates.emplace_back(static_cast<double>(sum_u) / static_cast<double>(run_count));
            }
            run_start = run_end + 1;
        }

        if (!row_candidates.empty()) candidates_by_row.emplace_back(row, std::move(row_candidates));
    }

    if (static_cast<int>(candidates_by_row.size()) < cfg.lane_path_min_points) return {};

    const double center_u = 0.5 * static_cast<double>(cols - 1);
    ImageLineFit path_line;
    bool fit_ok = false;
    double angle_deg = 0.0;
    if (reference_line != nullptr && reference_line->valid) {
        path_line = *reference_line;
        fit_ok = true;
    } else {
        std::vector<cv::Point> seed_points;
        seed_points.reserve(candidates_by_row.size());
        for (const auto& [row, candidates] : candidates_by_row) {
            double selected_u = candidates.front();
            if (side == LEFT) {
                bool found = false;
                for (const double candidate : candidates) {
                    if (candidate <= center_u && (!found || candidate > selected_u)) {
                        selected_u = candidate;
                        found = true;
                    }
                }
                if (!found) selected_u = *std::max_element(candidates.begin(), candidates.end());
            } else {
                bool found = false;
                for (const double candidate : candidates) {
                    if (candidate >= center_u && (!found || candidate < selected_u)) {
                        selected_u = candidate;
                        found = true;
                    }
                }
                if (!found) selected_u = *std::min_element(candidates.begin(), candidates.end());
            }
            seed_points.emplace_back(static_cast<int>(std::lround(selected_u)), row);
        }
        fit_ok = fitImageLineUv(seed_points, path_line, angle_deg);
    }
    if (!fit_ok) return {};

    const double max_deviation = std::max(12.0, static_cast<double>(cols) * cfg.lane_path_max_deviation_ratio);
    std::vector<cv::Point> path;
    for (int iteration = 0; iteration < std::max(1, cfg.lane_path_refine_iterations); ++iteration) {
        std::vector<cv::Point> selected_points;
        selected_points.reserve(candidates_by_row.size());
        for (const auto& [row, candidates] : candidates_by_row) {
            const double predicted_u = path_line.a * static_cast<double>(row) + path_line.b;
            const auto closest =
                std::min_element(candidates.begin(), candidates.end(), [predicted_u](const double lhs, const double rhs) {
                    return std::abs(lhs - predicted_u) < std::abs(rhs - predicted_u);
                });
            if (closest != candidates.end() && std::abs(*closest - predicted_u) <= max_deviation) {
                selected_points.emplace_back(static_cast<int>(std::lround(*closest)), row);
            }
        }
        if (static_cast<int>(selected_points.size()) < cfg.lane_path_min_points) return {};
        path = std::move(selected_points);
        if (!fitImageLineUv(path, path_line, angle_deg)) return {};
    }

    return path;
}

std::vector<cv::Point> selectBestComponent(const cv::Mat& cleaned_mask, const cv::Mat& roi_mask, const int side,
                                           const ImageLineFit* reference_line, const DynamicRoiConfig& cfg) {
    cv::Mat masked;
    cv::bitwise_and(cleaned_mask, roi_mask, masked);
    auto components = componentPixels(masked, cfg.min_roi_component_area);
    if (components.empty()) return {};

    std::sort(components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
        return bottomRow(lhs) > bottomRow(rhs);
    });

    const double center_u = 0.5 * static_cast<double>(cleaned_mask.cols - 1);
    const int bottom_group_tolerance = std::max(2, cleaned_mask.rows / 100);
    std::size_t index = 0;
    while (index < components.size()) {
        const int group_bottom = bottomRow(components[index]);
        double best_score = std::numeric_limits<double>::infinity();
        std::vector<cv::Point> best_component;

        while (index < components.size() && bottomRow(components[index]) >= group_bottom - bottom_group_tolerance) {
            const auto& pixels = components[index];
            double score = std::numeric_limits<double>::infinity();
            if (reference_line != nullptr && reference_line->valid) {
                double error_sum = 0.0;
                for (const auto& pixel : pixels) {
                    const double pred = reference_line->a * static_cast<double>(pixel.y) + reference_line->b;
                    error_sum += std::abs(static_cast<double>(pixel.x) - pred);
                }
                score = error_sum / static_cast<double>(pixels.size());
            } else {
                score = scoreInitComponent(cleaned_mask.size(), pixels, side, center_u, cfg);
            }

            if (score < best_score) {
                best_score = score;
                best_component = pixels;
            }
            ++index;
        }

        if (!best_component.empty() && std::isfinite(best_score)) return best_component;
    }
    return {};
}

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> extractInitPixels(const cv::Mat& cleaned_mask,
                                                                            const int side,
                                                                            const ImageLineFit& opposite_line,
                                                                            const DynamicRoiConfig& cfg) {
    auto [roi_mask, roi_polygon] = buildInitRoi(cleaned_mask.size(), side, opposite_line, cfg);
    return {selectBestComponent(cleaned_mask, roi_mask, side, nullptr, cfg), roi_polygon};
}

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> extractTrackingPixels(const cv::Mat& cleaned_mask,
                                                                                const ImageLineFit& previous_line,
                                                                                const int side,
                                                                                const ImageLineFit& opposite_line,
                                                                                const DynamicRoiConfig& cfg) {
    auto [roi_mask, roi_polygon] = buildTrackingRoi(cleaned_mask.size(), previous_line, side, opposite_line, cfg);
    return {selectBestComponent(cleaned_mask, roi_mask, side, &previous_line, cfg), roi_polygon};
}

bool hasExcessiveJump(const ImageLineFit& previous_line, const ImageLineFit& current_line, const int rows,
                      const int cols, const DynamicRoiConfig& cfg) {
    if (!previous_line.valid) return false;

    const int v_top = topRow(rows, cfg);
    const int v_bottom = rows - 1;
    const double prev_top = previous_line.a * static_cast<double>(v_top) + previous_line.b;
    const double curr_top = current_line.a * static_cast<double>(v_top) + current_line.b;
    const double prev_bottom = previous_line.a * static_cast<double>(v_bottom) + previous_line.b;
    const double curr_bottom = current_line.a * static_cast<double>(v_bottom) + current_line.b;
    const double top_jump = std::abs(curr_top - prev_top);
    const double bottom_jump = std::abs(curr_bottom - prev_bottom);
    const double top_threshold =
        std::max(cfg.tracking_jump_margin_multiplier * marginAtRow(v_top, rows, cols, cfg),
                 cfg.tracking_jump_image_width_ratio * static_cast<double>(cols));
    const double bottom_threshold =
        std::max(cfg.tracking_jump_margin_multiplier * marginAtRow(v_bottom, rows, cols, cfg),
                 cfg.tracking_jump_image_width_ratio * static_cast<double>(cols));
    return top_jump > top_threshold && bottom_jump > bottom_threshold;
}

bool hasInsufficientLaneSpacing(const ImageLineFit& left_line, const ImageLineFit& right_line, const int rows,
                                const int cols, const DynamicRoiConfig& cfg) {
    if (!left_line.valid || !right_line.valid) return false;

    for (const int row : {static_cast<int>(std::lround(rows * 0.78)), rows - 1}) {
        const double left_u = left_line.a * static_cast<double>(row) + left_line.b;
        const double right_u = right_line.a * static_cast<double>(row) + right_line.b;
        const double spacing = right_u - left_u;
        const double min_spacing = 0.65 * laneSpacingAtRow(row, rows, cols, cfg);
        if (spacing < min_spacing) return true;
    }
    return false;
}

class DynamicRoiProcessor {
public:
    explicit DynamicRoiProcessor(const DynamicRoiConfig& cfg) : cfg_(cfg) {}

    FrameResult findLanePixels(const cv::Mat& mask) {
        FrameResult frame;
        const cv::Mat binary_mask = toBinaryMask(mask);
        cv::Mat cleaned_mask = removeSmallComponents(binary_mask, scaledMinArea(binary_mask.size(), cfg_));
        auto [cropped_mask, global_roi_polygon] = applyGlobalImageRoiCrop(cleaned_mask, cfg_);

        frame.cleaned_mask = cropped_mask;
        frame.global_roi_polygon = std::move(global_roi_polygon);

        for (const int side : {LEFT, RIGHT}) frame.side_results[side] = processSide(frame.cleaned_mask, side);
        applyRightReferenceSpacingGuard(frame.side_results, frame.cleaned_mask.size());
        return frame;
    }

    const std::array<SideDebugInfo, 2>& debugInfo() const { return debug_info_; }

    void registerProjectedOutputFailure(const int side) {
        if (side != LEFT && side != RIGHT) return;
        registerDetectionFailure(stored_lines_[side], true);
    }

private:
    SideResult processSide(const cv::Mat& cleaned_mask, const int side) {
        ImageLineFit& stored_line = stored_lines_[side];
        ImageLineFit& opposite_line = stored_lines_[side == LEFT ? RIGHT : LEFT];
        bool use_tracking_roi = stored_line.valid && stored_line.lost_count <= cfg_.max_lost_frames;

        std::vector<cv::Point> roi_polygon;
        SideResult result;
        if (use_tracking_roi) {
            auto [tracking_pixels, tracking_polygon] =
                extractTrackingPixels(cleaned_mask, stored_line, side, opposite_line, cfg_);
            result = evaluateSideCandidate(tracking_pixels, "tracking", &stored_line, side, cleaned_mask.size());
            roi_polygon = std::move(tracking_polygon);

            const int tracking_bottom = bottomRow(result.component_pixels);
            const int recovery_row =
                static_cast<int>(std::lround(static_cast<double>(cleaned_mask.rows) * cfg_.tracking_recovery_bottom_ratio));
            const bool should_try_init = result.status != "ok" || tracking_bottom < recovery_row;
            if (should_try_init) {
                auto [init_pixels, init_polygon] = extractInitPixels(cleaned_mask, side, opposite_line, cfg_);
                auto init_result = evaluateSideCandidate(init_pixels, "init", nullptr, side, cleaned_mask.size());
                const int init_bottom = bottomRow(init_result.component_pixels);
                if (init_result.status == "ok" &&
                    (result.status != "ok" ||
                     init_bottom >= tracking_bottom + cfg_.tracking_recovery_bottom_advantage_rows)) {
                    result = std::move(init_result);
                    roi_polygon = std::move(init_polygon);
                    use_tracking_roi = false;
                }
            }
        } else {
            auto [init_pixels, init_polygon] = extractInitPixels(cleaned_mask, side, opposite_line, cfg_);
            result = evaluateSideCandidate(init_pixels, "init", nullptr, side, cleaned_mask.size());
            roi_polygon = std::move(init_polygon);
        }

        debug_info_[side] = SideDebugInfo{use_tracking_roi, stored_line, roi_polygon, result.component_pixels};

        if (result.status != "ok") {
            registerDetectionFailure(stored_line, true);
            result.lost_count = stored_line.lost_count;
            return result;
        }

        const bool reject_jump =
            use_tracking_roi && hasExcessiveJump(stored_line, result.current_line, cleaned_mask.rows, cleaned_mask.cols, cfg_);
        if (reject_jump) {
            result.status = "rejected_excessive_jump";
            registerDetectionFailure(stored_line, false);
            result.lost_count = stored_line.lost_count;
            return result;
        }

        updateStoredLine(stored_line, result.current_line);
        result.status = "ok";
        result.lost_count = stored_line.lost_count;
        return result;
    }

    SideResult evaluateSideCandidate(const std::vector<cv::Point>& component, const std::string& roi_mode,
                                     const ImageLineFit* reference_line, const int side, const cv::Size& image_size) const {
        SideResult result;
        result.roi_mode = roi_mode;
        result.component_pixels = component;
        result.pixels = component;

        if (!populateComponentSupportMetrics(result, component, image_size, cfg_)) return result;

        auto path_pixels = extractLanePath(component, reference_line, side, image_size, cfg_);
        if (static_cast<int>(path_pixels.size()) < cfg_.lane_path_min_points) {
            result.status = "insufficient_continuous_lane_path";
            return result;
        }

        result.pixels = std::move(path_pixels);
        ImageLineFit current_line;
        double angle_deg = 0.0;
        const bool fit_ok = fitImageLineUv(result.pixels, current_line, angle_deg);
        result.current_line = current_line;
        result.angle_deg = angle_deg;
        result.image_fit_rmse_px = imageFitRmse(result.pixels, current_line);
        if (!fit_ok) {
            result.status = "too_few_or_degenerate_pixels";
        } else if (hasInvalidSideGeometry(side, current_line, angle_deg, cfg_)) {
            result.status = "rejected_side_geometry";
        } else {
            result.status = "ok";
        }
        return result;
    }

    void applyRightReferenceSpacingGuard(std::array<SideResult, NUM_LANE_LINES>& side_results,
                                         const cv::Size& image_size) {
        auto& left_result = side_results[LEFT];
        auto& right_result = side_results[RIGHT];
        if (left_result.status != "ok" || right_result.status != "ok") return;
        if (!hasInsufficientLaneSpacing(left_result.current_line, right_result.current_line, image_size.height,
                                        image_size.width, cfg_)) {
            return;
        }

        left_result.status = "rejected_lane_spacing";
        left_result.current_line = ImageLineFit{};
        left_result.pixels.clear();
        registerDetectionFailure(stored_lines_[LEFT], true);
        left_result.lost_count = stored_lines_[LEFT].lost_count;
    }

    void updateStoredLine(ImageLineFit& stored_line, const ImageLineFit& current_line) const {
        if (!stored_line.valid) {
            stored_line = current_line;
        } else {
            const double alpha = cfg_.line_update_alpha;
            stored_line.a = alpha * current_line.a + (1.0 - alpha) * stored_line.a;
            stored_line.b = alpha * current_line.b + (1.0 - alpha) * stored_line.b;
            stored_line.valid = true;
        }
        stored_line.lost_count = 0;
    }

    void registerDetectionFailure(ImageLineFit& stored_line, const bool force_reset) const {
        if (force_reset) {
            stored_line.valid = false;
            stored_line.lost_count = cfg_.max_lost_frames + 1;
            return;
        }
        ++stored_line.lost_count;
        if (stored_line.lost_count > cfg_.max_lost_frames) stored_line.valid = false;
    }

    DynamicRoiConfig cfg_;
    std::array<ImageLineFit, 2> stored_lines_;
    std::array<SideDebugInfo, 2> debug_info_;
};

std::vector<Eigen::Vector3d> imagePointsToVehiclePoints(const std::vector<cv::Point2f>& pixels_f,
                                                        const cv::Mat& inverse_camera_matrix,
                                                        const tf2::Transform& vehicle_T_camera) {
    if (pixels_f.empty()) return {};
    const auto tf_points = pixelsToPoints(pixels_f, inverse_camera_matrix, vehicle_T_camera);

    std::vector<Eigen::Vector3d> vehicle_points;
    vehicle_points.reserve(tf_points.size());
    for (const auto& point : tf_points) vehicle_points.emplace_back(point.x(), point.y(), point.z());
    return vehicle_points;
}

std::vector<Eigen::Vector3d> pixelsToVehiclePoints(const std::vector<cv::Point>& pixels,
                                                   const cv::Mat& inverse_camera_matrix,
                                                   const tf2::Transform& vehicle_T_camera) {
    if (pixels.empty()) return {};

    std::vector<cv::Point2f> pixels_f;
    pixels_f.reserve(pixels.size());
    for (const auto& pixel : pixels) pixels_f.emplace_back(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
    return imagePointsToVehiclePoints(pixels_f, inverse_camera_matrix, vehicle_T_camera);
}

std::vector<Eigen::Vector3d> cropVehiclePoints(const std::vector<Eigen::Vector3d>& points,
                                               const DynamicRoiConfig& cfg) {
    std::vector<Eigen::Vector3d> cropped;
    for (const auto& point : points) {
        if (point.x() >= cfg.xmin && point.x() <= cfg.xmax && point.y() >= cfg.ymin && point.y() <= cfg.ymax)
            cropped.emplace_back(point);
    }
    return cropped;
}

std::vector<Eigen::Vector3d> sortedUniqueFiniteByX(const std::vector<Eigen::Vector3d>& points) {
    std::vector<Eigen::Vector3d> sorted;
    sorted.reserve(points.size());
    for (const auto& point : points) {
        if (std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z())) sorted.emplace_back(point);
    }
    std::sort(sorted.begin(), sorted.end(), [](const auto& lhs, const auto& rhs) { return lhs.x() < rhs.x(); });

    std::vector<Eigen::Vector3d> unique;
    unique.reserve(sorted.size());
    for (const auto& point : sorted) {
        if (unique.empty() || std::abs(point.x() - unique.back().x()) > 1e-9) unique.emplace_back(point);
    }
    return unique;
}

double interpolateYAtX(const std::vector<Eigen::Vector3d>& sorted_points, const double x) {
    if (sorted_points.empty()) return 0.0;
    if (x <= sorted_points.front().x()) return sorted_points.front().y();
    if (x >= sorted_points.back().x()) return sorted_points.back().y();

    const auto upper = std::lower_bound(sorted_points.begin(), sorted_points.end(), x,
                                        [](const Eigen::Vector3d& point, const double value) {
                                            return point.x() < value;
                                        });
    if (upper == sorted_points.begin()) return upper->y();
    if (upper == sorted_points.end()) return sorted_points.back().y();

    const auto& right = *upper;
    const auto& left = *(upper - 1);
    const double dx = right.x() - left.x();
    if (std::abs(dx) < 1e-9) return left.y();
    const double ratio = (x - left.x()) / dx;
    return left.y() + ratio * (right.y() - left.y());
}

std::vector<Eigen::Vector3d> fitAndResampleVehicleLine(const std::vector<Eigen::Vector3d>& points,
                                                       const DynamicRoiConfig& cfg) {
    if (points.size() < 2 || cfg.spacing <= 0.0) return {};

    const auto sorted = sortedUniqueFiniteByX(points);
    if (sorted.size() < 2 || sorted.back().x() - sorted.front().x() < cfg.min_projected_line_span_m) return {};

    Eigen::MatrixXd basis(static_cast<int>(sorted.size()), 2);
    Eigen::VectorXd y_values(static_cast<int>(sorted.size()));
    for (int i = 0; i < static_cast<int>(sorted.size()); ++i) {
        const double x = sorted[static_cast<std::size_t>(i)].x();
        basis(i, 0) = 1.0;
        basis(i, 1) = x;
        y_values(i) = sorted[static_cast<std::size_t>(i)].y();
    }

    const Eigen::Vector2d coeff = basis.colPivHouseholderQr().solve(y_values);
    if (!std::isfinite(coeff[0]) || !std::isfinite(coeff[1])) return {};

    const double first_x = std::ceil(sorted.front().x() / cfg.spacing) * cfg.spacing;
    const double last_x = std::floor(sorted.back().x() / cfg.spacing) * cfg.spacing;
    if (last_x < first_x) return {};

    std::vector<Eigen::Vector3d> resampled;
    for (double x = first_x; x <= last_x + 0.5 * cfg.spacing; x += cfg.spacing) {
        const double y = coeff[0] + coeff[1] * x;
        if (std::isfinite(y)) resampled.emplace_back(x, y, 0.0);
    }
    if (static_cast<int>(resampled.size()) < cfg.min_published_points) return {};
    return resampled;
}

std::vector<Eigen::Vector3d> makeCenterFromSides(const std::vector<Eigen::Vector3d>& left_points,
                                                 const std::vector<Eigen::Vector3d>& right_points) {
    const auto left = sortedUniqueFiniteByX(left_points);
    const auto right = sortedUniqueFiniteByX(right_points);
    if (left.empty() || right.empty()) return {};

    const double x_start = std::max(left.front().x(), right.front().x());
    const double x_stop = std::min(left.back().x(), right.back().x());
    if (x_stop < x_start) return {};

    std::vector<Eigen::Vector3d> center_points;
    center_points.reserve(left.size());
    for (const auto& left_point : left) {
        const double x = left_point.x();
        if (x < x_start || x > x_stop) continue;
        const double right_y = interpolateYAtX(right, x);
        center_points.emplace_back(x, 0.5 * (left_point.y() + right_y), 0.0);
    }
    return center_points;
}

cv::Scalar sideColor(const int side) {
    if (side == LEFT) return cv::Scalar(0, 255, 0);
    if (side == RIGHT) return cv::Scalar(0, 0, 255);
    return cv::Scalar(255, 255, 0);
}

cv::Mat imageToBgr(const cv::Mat& image) {
    cv::Mat image_u8;
    if (image.depth() != CV_8U) {
        image.convertTo(image_u8, CV_8U);
    } else {
        image_u8 = image;
    }

    cv::Mat bgr;
    if (image_u8.channels() == 1) {
        cv::cvtColor(image_u8, bgr, cv::COLOR_GRAY2BGR);
    } else if (image_u8.channels() == 4) {
        cv::cvtColor(image_u8, bgr, cv::COLOR_BGRA2BGR);
    } else {
        bgr = image_u8.clone();
    }
    return bgr;
}

void publishBgrImage(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher,
                     const builtin_interfaces::msg::Time& timestamp, const cv::Mat& image) {
    if (!publisher) return;

    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = timestamp;
    cv_img.encoding = "bgr8";
    cv_img.image = image;

    sensor_msgs::msg::Image msg;
    cv_img.toImageMsg(msg);
    publisher->publish(std::move(msg));
}

void blendPolygon(cv::Mat& image, const std::vector<cv::Point>& polygon, const cv::Scalar& color, const double alpha) {
    if (polygon.size() < 3 || alpha <= 0.0) return;

    cv::Mat overlay = image.clone();
    const std::vector<std::vector<cv::Point>> polygons = {polygon};
    cv::fillPoly(overlay, polygons, color);
    cv::addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, image);
    cv::polylines(image, polygons, true, color, 2, cv::LINE_AA);
}

void publishDynamicRoiVisualization(const cv::Mat& source_mask, const builtin_interfaces::msg::Time& timestamp,
                                    const FrameResult& frame_result, const DynamicRoiProcessor& processor) {
    cv::Mat visualization = imageToBgr(source_mask);
    blendPolygon(visualization, frame_result.global_roi_polygon, cv::Scalar(255, 255, 0), 0.18);

    const auto& debug_info = processor.debugInfo();
    for (const int side : {LEFT, RIGHT}) blendPolygon(visualization, debug_info[side].roi_polygon, sideColor(side), 0.35);

    publishBgrImage(g_dynamic_roi_image_pub, timestamp, visualization);
}

void publishLinearFitVisualization(const cv::Mat& source_mask, const builtin_interfaces::msg::Time& timestamp,
                                   const FrameResult& frame_result) {
    cv::Mat visualization = imageToBgr(source_mask);
    const int rows = visualization.rows;
    const int cols = visualization.cols;

    for (const int side : {LEFT, RIGHT}) {
        const auto& side_result = frame_result.side_results[side];
        const auto& pixels = side_result.pixels;
        const auto color = sideColor(side);

        if (!pixels.empty()) {
            const std::size_t step = std::max<std::size_t>(1, pixels.size() / 2000 + 1);
            for (std::size_t i = 0; i < pixels.size(); i += step) {
                const auto& pixel = pixels[i];
                if (pixel.x >= 0 && pixel.x < cols && pixel.y >= 0 && pixel.y < rows) {
                    visualization.at<cv::Vec3b>(pixel.y, pixel.x) =
                        cv::Vec3b(static_cast<uchar>(color[0]), static_cast<uchar>(color[1]), static_cast<uchar>(color[2]));
                }
            }
        }

        if (!side_result.current_line.valid || pixels.empty()) continue;

        const auto minmax_v =
            std::minmax_element(pixels.begin(), pixels.end(),
                                [](const cv::Point& lhs, const cv::Point& rhs) { return lhs.y < rhs.y; });
        const int v_min = clampInt(minmax_v.first->y, 0, rows - 1);
        const int v_max = clampInt(minmax_v.second->y, 0, rows - 1);
        const int u_min =
            clampInt(static_cast<int>(std::lround(side_result.current_line.a * v_min + side_result.current_line.b)), 0,
                     cols - 1);
        const int u_max =
            clampInt(static_cast<int>(std::lround(side_result.current_line.a * v_max + side_result.current_line.b)), 0,
                     cols - 1);
        cv::line(visualization, cv::Point(u_min, v_min), cv::Point(u_max, v_max), color, 3, cv::LINE_AA);
    }

    publishBgrImage(g_linear_fit_image_pub, timestamp, visualization);
}

cv::Point vehiclePointToPixel(const Eigen::Vector3d& point, const DynamicRoiConfig& cfg, const int width,
                              const int height) {
    constexpr int margin_left = 72;
    constexpr int margin_right = 34;
    constexpr int margin_top = 36;
    constexpr int margin_bottom = 52;

    const int inner_w = std::max(1, width - margin_left - margin_right);
    const int inner_h = std::max(1, height - margin_top - margin_bottom);
    const double px = margin_left + (cfg.ymax - point.y()) / (cfg.ymax - cfg.ymin) * inner_w;
    const double py = height - margin_bottom - (point.x() - cfg.xmin) / (cfg.xmax - cfg.xmin) * inner_h;
    return cv::Point(clampInt(static_cast<int>(std::lround(px)), 0, width - 1),
                     clampInt(static_cast<int>(std::lround(py)), 0, height - 1));
}

void drawVehiclePolyline(cv::Mat& image, const std::vector<Eigen::Vector3d>& points, const DynamicRoiConfig& cfg,
                         const cv::Scalar& color) {
    if (points.empty()) return;

    std::vector<cv::Point> image_points;
    image_points.reserve(points.size());
    for (const auto& point : points) image_points.emplace_back(vehiclePointToPixel(point, cfg, image.cols, image.rows));

    if (image_points.size() >= 2) {
        const std::vector<std::vector<cv::Point>> polylines = {image_points};
        cv::polylines(image, polylines, false, color, 3, cv::LINE_AA);
    }
    for (const auto& point : image_points) cv::circle(image, point, 3, color, -1, cv::LINE_AA);
}

cv::Mat makeVehicleFitVisualization(const LaneLines& lane_lines, const DynamicRoiConfig& cfg) {
    constexpr int width = 720;
    constexpr int height = 520;
    constexpr int margin_left = 72;
    constexpr int margin_right = 34;
    constexpr int margin_top = 36;
    constexpr int margin_bottom = 52;

    cv::Mat image(height, width, CV_8UC3, cv::Scalar(248, 248, 248));
    cv::rectangle(image, cv::Point(margin_left, margin_top), cv::Point(width - margin_right, height - margin_bottom),
                  cv::Scalar(40, 40, 40), 2);

    for (double x = std::ceil(cfg.xmin); x <= std::floor(cfg.xmax); x += 1.0) {
        const cv::Point p1 = vehiclePointToPixel(Eigen::Vector3d(x, cfg.ymin, 0.0), cfg, width, height);
        const cv::Point p2 = vehiclePointToPixel(Eigen::Vector3d(x, cfg.ymax, 0.0), cfg, width, height);
        cv::line(image, p1, p2, cv::Scalar(222, 222, 222), 1, cv::LINE_AA);
    }
    for (double y = std::ceil(cfg.ymin); y <= std::floor(cfg.ymax); y += 1.0) {
        const cv::Point p1 = vehiclePointToPixel(Eigen::Vector3d(cfg.xmin, y, 0.0), cfg, width, height);
        const cv::Point p2 = vehiclePointToPixel(Eigen::Vector3d(cfg.xmax, y, 0.0), cfg, width, height);
        cv::line(image, p1, p2, cv::Scalar(222, 222, 222), 1, cv::LINE_AA);
    }

    drawVehiclePolyline(image, lane_lines.left.points, cfg, sideColor(LEFT));
    drawVehiclePolyline(image, lane_lines.right.points, cfg, sideColor(RIGHT));
    drawVehiclePolyline(image, lane_lines.center.points, cfg, sideColor(CENTER));

    cv::putText(image, "vehicle frame published output", cv::Point(18, 26), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                cv::Scalar(40, 40, 40), 2, cv::LINE_AA);
    cv::putText(image, "x forward", cv::Point(width / 2 - 48, height - 16), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                cv::Scalar(70, 70, 70), 1, cv::LINE_AA);
    cv::putText(image, "y left", cv::Point(8, height / 2), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(70, 70, 70), 1,
                cv::LINE_AA);
    return image;
}

void publishVehicleFitVisualization(const builtin_interfaces::msg::Time& timestamp, const LaneLines& lane_lines,
                                    const DynamicRoiConfig& cfg) {
    publishBgrImage(g_vehicle_fit_image_pub, timestamp, makeVehicleFitVisualization(lane_lines, cfg));
}

cv::Mat visualizeLanePixels(const cv::Mat& binary_mask, const LaneLines& lane_lines) {
    constexpr uchar GRAY = 127;
    static const cv::Vec3b BLUE(255, 127, 0);
    static const cv::Vec3b GREEN(127, 255, 0);
    constexpr int MARKER_SIZE = 2;

    cv::Mat binary = toBinaryMask(binary_mask);
    cv::Mat visualization;
    cv::cvtColor(binary * GRAY, visualization, cv::COLOR_GRAY2BGR);

    for (const auto& pixel : lane_lines.left.pixels)
        cv::drawMarker(visualization, pixel, GREEN, cv::MARKER_SQUARE, MARKER_SIZE);
    for (const auto& pixel : lane_lines.right.pixels)
        cv::drawMarker(visualization, pixel, GREEN, cv::MARKER_SQUARE, MARKER_SIZE);
    for (const auto& pixel : lane_lines.center.pixels)
        cv::drawMarker(visualization, pixel, BLUE, cv::MARKER_SQUARE, MARKER_SIZE);
    return visualization;
}

DynamicRoiConfig makeDynamicRoiConfig(const int min_area, const int tolerance, const double xmin, const double xmax,
                                      const double ymin, const double ymax, const double spacing) {
    DynamicRoiConfig cfg;
    cfg.min_area = min_area;
    cfg.tolerance = tolerance;
    cfg.xmin = xmin;
    cfg.xmax = xmax;
    cfg.ymin = ymin;
    cfg.ymax = ymax;
    cfg.spacing = spacing;
    return cfg;
}

}  // namespace

LaneLinePublisher::LaneLinePublisher() : Node("lane_line_publisher") {
    initMembers();
    initConnections();
    RCLCPP_INFO(get_logger(), "Launched %s", get_name());
}

void LaneLinePublisher::initMembers() {
    debug_ = getRosParameter<bool>(this, "debug");
    vehicle_frame_id_ = getRosParameter<std::string>(this, "vehicle_frame_id");
    xmin_ = getRosParameter<double>(this, "lane_line_publisher.roi.xmin");
    xmax_ = getRosParameter<double>(this, "lane_line_publisher.roi.xmax");
    ymin_ = getRosParameter<double>(this, "lane_line_publisher.roi.ymin");
    ymax_ = getRosParameter<double>(this, "lane_line_publisher.roi.ymax");
    spacing_ = getRosParameter<double>(this, "lane_line_publisher.spacing");
    const auto camera_frame_id = getRosParameter<std::string>(this, "camera_frame_id");
    const auto camera_name = getRosParameter<std::string>(this, "camera_name");
    const auto min_area = getRosParameter<int>(this, "lane_pixel_finder.min_area");
    const auto tolerance = getRosParameter<int>(this, "lane_pixel_finder.tolerance");

    getCameraParams(this, camera_name, camera_matrix_);
    camera_matrix_ = camera_matrix_.inv();
    vehicle_T_camera_ = getTf2Transform(this, vehicle_frame_id_, camera_frame_id);

    g_dynamic_roi_processor = std::make_unique<DynamicRoiProcessor>(
        makeDynamicRoiConfig(min_area, tolerance, xmin_, xmax_, ymin_, ymax_, spacing_));
}

void LaneLinePublisher::initConnections() {
    const auto queue_size = getRosParameter<int>(this, "lane_line_publisher.queue_size");
    mask_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "mask_image", queue_size, std::bind(&LaneLinePublisher::imageCallback, this, std::placeholders::_1));
    lane_line_pubs_ = {create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/left", queue_size),
                       create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/right", queue_size),
                       create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/center", queue_size)};

    g_dynamic_roi_image_pub = create_publisher<sensor_msgs::msg::Image>("dynamic_roi_image", queue_size);
    g_linear_fit_image_pub = create_publisher<sensor_msgs::msg::Image>("linear_fit_image", queue_size);
    g_vehicle_fit_image_pub = create_publisher<sensor_msgs::msg::Image>("vehicle_fit_image", queue_size);

    if (!debug_) return;

    annotated_mask_image_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_mask_image", queue_size);
    contour_point_pubs_ = {create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/left", queue_size),
                           create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/right", queue_size),
                           create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/center", queue_size)};
}

void LaneLinePublisher::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    RCLCPP_DEBUG(get_logger(), "Received image [%ix%i]", static_cast<int>(msg->width), static_cast<int>(msg->height));

    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    if (cv_img->image.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty image");
        return;
    }

    LaneLines lane_lines;
    findLaneLines(cv_img->image, msg->header.stamp, lane_lines);
    publishLaneLines(lane_lines, msg->header.stamp);
}

void LaneLinePublisher::findLaneLines(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                                      LaneLines& lane_lines) const {
    if (!g_dynamic_roi_processor) return;

    auto frame_result = g_dynamic_roi_processor->findLanePixels(mask);
    lane_lines.left.pixels = frame_result.side_results[LEFT].pixels;
    lane_lines.right.pixels = frame_result.side_results[RIGHT].pixels;

    publishDynamicRoiVisualization(mask, timestamp, frame_result, *g_dynamic_roi_processor);
    publishLinearFitVisualization(mask, timestamp, frame_result);
    if (debug_) publishAnnotatedMask(frame_result.cleaned_mask, timestamp, lane_lines);

    std::vector<std::vector<Eigen::Vector3d>> contour_points(NUM_LANE_LINES);
    const DynamicRoiConfig cfg = makeDynamicRoiConfig(0, 0, xmin_, xmax_, ymin_, ymax_, spacing_);

    std::array<bool, NUM_LANE_LINES> side_output_valid{};
    for (const int side : {LEFT, RIGHT}) {
        auto& side_result = frame_result.side_results[side];
        auto& output_line = side == LEFT ? lane_lines.left : lane_lines.right;
        if (side_result.status != "ok" || !side_result.current_line.valid) continue;

        const auto vehicle_points_before_crop =
            pixelsToVehiclePoints(side_result.pixels, camera_matrix_, vehicle_T_camera_);
        const auto vehicle_points = cropVehiclePoints(vehicle_points_before_crop, cfg);
        const auto published_points = fitAndResampleVehicleLine(vehicle_points, cfg);
        if (static_cast<int>(published_points.size()) < cfg.min_published_points) {
            side_result.status = "insufficient_vehicle_line_support";
            g_dynamic_roi_processor->registerProjectedOutputFailure(side);
            continue;
        }

        output_line.points = published_points;
        contour_points[side] = vehicle_points_before_crop;
        side_output_valid[side] = true;
    }

    if (side_output_valid[LEFT] && side_output_valid[RIGHT]) {
        lane_lines.center.points = makeCenterFromSides(lane_lines.left.points, lane_lines.right.points);
        contour_points[CENTER] = lane_lines.center.points;
    }

    publishVehicleFitVisualization(timestamp, lane_lines, cfg);
    if (debug_) publishContourPoints(contour_points, timestamp);
}

void LaneLinePublisher::publishAnnotatedMask(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                                             const LaneLines& lane_lines) const {
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = timestamp;
    cv_img.encoding = "bgr8";
    cv_img.image = visualizeLanePixels(mask, lane_lines);

    sensor_msgs::msg::Image msg;
    cv_img.toImageMsg(msg);
    annotated_mask_image_pub_->publish(std::move(msg));
}

void LaneLinePublisher::publishContourPoints(const std::vector<std::vector<Eigen::Vector3d>>& contour_points,
                                             const builtin_interfaces::msg::Time& timestamp) const {
    for (int i = 0; i < NUM_LANE_LINES; ++i) {
        const auto& points = contour_points[i];
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (const auto& point : points) {
            auto& p = cloud.points.emplace_back();
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
        }

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(cloud, pcl_msg);
        pcl_msg.header.stamp = timestamp;
        pcl_msg.header.frame_id = vehicle_frame_id_;
        contour_point_pubs_[i]->publish(pcl_msg);
    }
}

void LaneLinePublisher::publishLaneLines(const LaneLines& lane_lines,
                                         const builtin_interfaces::msg::Time& timestamp) const {
    const std::vector<const LaneLine*> lane_line_ptrs = {&lane_lines.left, &lane_lines.right, &lane_lines.center};

    for (int i = 0; i < NUM_LANE_LINES; ++i) {
        const auto& lane_line_points = lane_line_ptrs[i]->points;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (const auto& point : lane_line_points) {
            auto& p = cloud.points.emplace_back();
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
        }

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(cloud, pcl_msg);
        pcl_msg.header.stamp = timestamp;
        pcl_msg.header.frame_id = vehicle_frame_id_;
        lane_line_pubs_[i]->publish(pcl_msg);
    }
}

}  // namespace aiformula

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::LaneLinePublisher>());
    rclcpp::shutdown();
    return 0;
}
