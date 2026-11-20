#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <iterator>
#include <optional>
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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_cpp/camera.hpp"
#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/tf2_transform.hpp"

namespace aiformula::association {

using Line = std::vector<Eigen::Vector3d>;
using Boundaries = std::array<Line, 2>; // left, right; x forward, y left
struct Pose {
    double stamp, x, y, yaw;
};

class PoseHistory {
public:
    void add(Pose p) {
        auto it = std::lower_bound(samples_.begin(), samples_.end(), p.stamp,
                                   [](const Pose &q, double t) { return q.stamp < t; });
        if (it != samples_.end() && it->stamp == p.stamp)
            *it = p;
        else
            samples_.insert(it, p);
        while (samples_.back().stamp - samples_.front().stamp > 5.0 || samples_.size() > 1000)
            samples_.pop_front();
    }
    std::optional<Pose> at(double t, double max_nearest_age = 0.15) const {
        auto b = std::lower_bound(samples_.begin(), samples_.end(), t,
                                  [](const Pose &p, double s) { return p.stamp < s; });
        if (b != samples_.end() && b->stamp == t)
            return *b;
        if (b != samples_.begin() && b != samples_.end()) {
            const auto &a = *std::prev(b);
            // Low-rate bags can bracket an image by over a second. Interpolate
            // only across physically continuous poses and modest heading change.
            const double dt = b->stamp - a.stamp;
            const double turn = std::remainder(b->yaw - a.yaw, 2. * std::acos(-1.));
            if (dt <= 1.6 && std::hypot(b->x - a.x, b->y - a.y) <= 5. * dt + 0.2 && std::abs(turn) <= 0.65) {
                const double f = (t - a.stamp) / (b->stamp - a.stamp);
                return Pose{t, a.x + f * (b->x - a.x), a.y + f * (b->y - a.y),
                            a.yaw + f * std::remainder(b->yaw - a.yaw, 2.0 * std::acos(-1.0))};
            }
        }
        const Pose *p = b == samples_.end() ? nullptr : &*b;
        if (b != samples_.begin() && (!p || t - std::prev(b)->stamp < p->stamp - t))
            p = &*std::prev(b);
        if (p && std::abs(p->stamp - t) <= max_nearest_age)
            return *p;
        return std::nullopt;
    }
    void clear() { samples_.clear(); }

private:
    std::deque<Pose> samples_;
};

inline Line predict(const Line &previous, const Pose &a, const Pose &b) {
    Line out;
    const double c = std::cos(a.yaw - b.yaw), s = std::sin(a.yaw - b.yaw);
    const double dx = a.x - b.x, dy = a.y - b.y;
    const double tx = std::cos(b.yaw) * dx + std::sin(b.yaw) * dy, ty = -std::sin(b.yaw) * dx + std::cos(b.yaw) * dy;
    for (const auto &p : previous) {
        const double x = c * p.x() - s * p.y() + tx, y = s * p.x() + c * p.y() + ty;
        if (!out.empty() && x <= out.back().x())
            return {};
        out.emplace_back(x, y, 0.);
    }
    return out;
}

struct Separation {
    int count = 0;
    double median = 0, low = 0, high = 0, midpoint = 0;
};

// Signed local-normal separation at several shared, observed forward distances.
inline Separation compare(const Line &a, const Line &b, double xmax = std::numeric_limits<double>::infinity()) {
    if (a.size() < 2 || b.size() < 2)
        return {};
    const double first = std::max({a.front().x(), b.front().x(), 0.5});
    const double last = std::min({a.back().x(), b.back().x(), xmax});
    if (last - first < 0.95)
        return {};
    std::vector<double> distances;
    double midpoint = 0.;
    size_t i = 1, j = 1;
    for (double x = first; x <= last + 1e-6; x += 0.5) {
        while (i + 1 < a.size() && a[i].x() < x)
            ++i;
        while (j + 1 < b.size() && b[j].x() < x)
            ++j;
        const double ma = (a[i].y() - a[i - 1].y()) / (a[i].x() - a[i - 1].x());
        const double mb = (b[j].y() - b[j - 1].y()) / (b[j].x() - b[j - 1].x());
        const double ya = a[i].y() + ma * (x - a[i].x()), yb = b[j].y() + mb * (x - b[j].x());
        const double norm = std::hypot(1., 0.5 * (ma + mb));
        if (distances.empty())
            midpoint = (ya + yb) / (2. * norm);
        distances.push_back((ya - yb) / norm);
    }
    std::sort(distances.begin(), distances.end());
    return {static_cast<int>(distances.size()), distances[distances.size() / 2], distances.front(), distances.back(),
            midpoint};
}

struct Config {
    double min_width = 2.1, expected_width = 3.5, max_width = 4.9, match_distance = 0.85;
};
struct Selection {
    int left = -1, right = -1;
};

inline Selection select(const std::vector<Line> &lines, const Boundaries &expected, const Config &cfg) {
    const size_t n = lines.size();
    constexpr double inf = std::numeric_limits<double>::infinity();
    std::vector<std::array<double, 2>> error(n, {inf, inf});
    std::vector<std::vector<Separation>> gaps(n, std::vector<Separation>(n));
    for (size_t i = 0; i < n; ++i) {
        for (size_t s = 0; s < 2; ++s) {
            const auto d = compare(lines[i], expected[s]);
            if (d.count >= 3)
                error[i][s] = std::max(std::abs(d.low), std::abs(d.high));
        }
        for (size_t j = 0; j < n; ++j)
            if (i != j)
                gaps[i][j] = compare(lines[i], lines[j]);
    }
    auto near_parallel = [&](size_t outer, size_t inner) {
        const auto &d = gaps[outer][inner];
        return d.count >= 3 && d.low > 0.14 && d.high < 0.5 * cfg.min_width && d.high - d.low < 0.5;
    };
    std::vector<std::array<bool, 2>> outer(n, {false, false});
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j) {
            if (near_parallel(i, j)) {
                outer[i][0] = true;
                outer[j][1] = true;
            }
        }
    double best = inf, runner = inf;
    Selection result;
    const bool have_prior = !expected[0].empty() || !expected[1].empty();
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j) {
            const auto &w = gaps[i][j];
            if (w.count < 3 || w.low < cfg.min_width || w.high > cfg.max_width || outer[i][0] || outer[j][1])
                continue;
            double cost = std::abs(w.median - cfg.expected_width) + 0.15 * (w.high - w.low);
            const double left_error = error[i][0], right_error = error[j][1];
            if (std::isfinite(left_error) || std::isfinite(right_error)) {
                if (std::isfinite(left_error)) {
                    if (left_error > cfg.match_distance)
                        continue;
                    cost += left_error;
                }
                if (std::isfinite(right_error)) {
                    if (right_error > cfg.match_distance)
                        continue;
                    cost += right_error;
                }
            } else {
                const double center_limit = (outer[i][1] || outer[j][0]) ? 0.58 : 0.46;
                if (std::abs(w.midpoint) > center_limit * w.median)
                    continue;
                cost += std::abs(w.midpoint);
            }
            if (cost < best) {
                runner = best;
                best = cost;
                result = {static_cast<int>(i), static_cast<int>(j)};
            } else
                runner = std::min(runner, cost);
        }
    if (result.left >= 0)
        return runner - best < 0.20 ? Selection{} : result;
    // No trusted pair: a single marking needs an unambiguous, motion-compensated identity.
    if (!have_prior)
        return {};
    for (int s = 0; s < 2; ++s) {
        double first = inf, second = inf;
        int id = -1;
        for (size_t i = 0; i < n; ++i) {
            if (error[i][s] > 0.65 * cfg.match_distance || error[i][1 - s] < error[i][s] + 0.45 || outer[i][s])
                continue;
            if (error[i][s] < first) {
                second = first;
                first = error[i][s];
                id = static_cast<int>(i);
            } else
                second = std::min(second, error[i][s]);
        }
        if (id >= 0 && second - first >= 0.20)
            (s == 0 ? result.left : result.right) = id;
    }
    // Two independently matched singles must never bypass the width test above.
    if (result.left >= 0 && result.right >= 0)
        return {};
    return result;
}
} // namespace aiformula::association

namespace aiformula::green_boundary {

enum class Surface { unknown, green, pavement };

// Each patch is voted in the camera's native RGB values. The relative and
// absolute margins distinguish faded green paint from gray asphalt and noise.
inline Surface surface(int r, int g, int b) {
    const int hi = std::max({r, g, b}), lo = std::min({r, g, b});
    if (lo < 28 || hi > 238)
        return Surface::unknown;
    if (g - std::max(r, b) >= std::max(8., 0.065 * g))
        return Surface::green;
    if (g - std::max(r, b) <= 4 && hi - lo <= std::max(22., 0.22 * hi))
        return Surface::pavement;
    return Surface::unknown;
}

struct Flank { double x, y, slope; };

inline Flank at(const association::Line &line, double x) {
    auto it = std::lower_bound(line.begin() + 1, line.end() - 1, x,
                               [](const auto &p, double value) { return p.x() < value; });
    const auto &prev = *std::prev(it);
    const double slope = (it->y() - prev.y()) / (it->x() - prev.x());
    return {x, prev.y() + slope * (x - prev.x()), slope};
}

template <class Sample>
Surface flankSurface(const Flank &f, int direction, const Sample &sample) {
    int green = 0, pavement = 0;
    for (double offset : {0.30, 0.60, 0.90}) {
        const double dy = direction * offset / std::hypot(1., f.slope);
        const auto s = sample(f.x - f.slope * dy, f.y + dy);
        green += s == Surface::green;
        pavement += s == Surface::pavement;
    }
    if (green >= 2) return Surface::green;
    if (pavement >= 2) return Surface::pavement;
    return Surface::unknown;
}

// A nearby double stripe is relevant only if it overlaps each boundary
// currently proposed by the original detector. A marking in another lane
// must never invalidate a usable single or a normal-width pair.
inline bool relevantDouble(const association::Line &a, const association::Line &b,
                           const association::Boundaries &original, const association::Separation &gap) {
    const double narrow = std::min(std::abs(gap.low), std::abs(gap.high));
    const double wide = std::max(std::abs(gap.low), std::abs(gap.high));
    if (gap.count < 3 || gap.low * gap.high <= 0 || narrow <= 0.14 ||
        wide >= 1.0 || wide - narrow >= 0.5) return false;
    for (const auto &side : original) {
        if (side.empty()) continue;
        const auto matches = [&](const association::Line &stripe) {
            const auto d = association::compare(side, stripe, 6.);
            return d.count >= 3 && std::max(std::abs(d.low), std::abs(d.high)) <= 0.55;
        };
        if (!matches(a) && !matches(b)) return false;
    }
    return true;
}

inline bool hasRelevantDouble(const std::vector<association::Line> &lines, const association::Boundaries &original) {
    for (size_t i = 0; i < lines.size(); ++i)
        for (size_t j = i + 1; j < lines.size(); ++j)
            if (relevantDouble(lines[i], lines[j], original, association::compare(lines[i], lines[j]))) return true;
    return false;
}

// Used only when geometry did not already find a complete lane. Multiple
// interpretations must agree on the actual stripe AND on its side.
template <class Sample>
association::Selection resolve(const std::vector<association::Line> &lines,
                               association::Selection geometric, const Sample &sample,
                               const association::Boundaries &original = {}) {
    if (geometric.left >= 0 && geometric.right >= 0)
        return geometric;
    association::Selection agreed;
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            const auto gap = association::compare(lines[i], lines[j]);
            if (!relevantDouble(lines[i], lines[j], original, gap)) continue;
            const int left = static_cast<int>(gap.median > 0 ? i : j);
            const int right = static_cast<int>(gap.median > 0 ? j : i);
            if ((geometric.left >= 0 && geometric.left != left && geometric.left != right) ||
                (geometric.right >= 0 && geometric.right != left && geometric.right != right))
                continue;
            const auto &l = lines[left], &r = lines[right];
            const double begin = std::max({1.5, l.front().x(), r.front().x()});
            const double end = std::min({6., l.back().x(), r.back().x()});
            int stations = 0, green_left = 0, green_right = 0;
            for (double x = begin; x <= end + 1e-6; x += 0.5) {
                const auto a = flankSurface(at(l, x), 1, sample);
                const auto b = flankSurface(at(r, x), -1, sample);
                ++stations;
                green_left += a == Surface::green && b == Surface::pavement;
                green_right += b == Surface::green && a == Surface::pavement;
            }
            const int required = std::max(3, (2 * stations + 2) / 3);
            association::Selection vote;
            if (green_left >= required && green_right == 0)
                vote.left = right;
            else if (green_right >= required && green_left == 0)
                vote.right = left;
            else
                continue;
            if ((agreed.left >= 0 || agreed.right >= 0) &&
                (agreed.left != vote.left || agreed.right != vote.right))
                return geometric;
            agreed = vote;
        }
    }
    return agreed.left >= 0 || agreed.right >= 0 ? agreed : geometric;
}

} // namespace aiformula::green_boundary

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

namespace { struct ImageLineFit; struct FrameResult; struct DynamicRoiConfig; }

class LaneLinePublisher : public rclcpp::Node {
public:
    LaneLinePublisher();
    ~LaneLinePublisher() override = default;

private:
    void initMembers();
    void initConnections();
    void subscribeColor(bool reliable);
    void matchColorQoS();
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    void cacheCalibration(CameraInfo::ConstSharedPtr info);
    void selectCalibration(const sensor_msgs::msg::Image& mask);
    rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
    std::deque<CameraInfo::ConstSharedPtr> calibrations_;
    CameraInfo::ConstSharedPtr active_calibration_;
    cv::Mat configured_camera_matrix_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    using ColorImage = sensor_msgs::msg::Image::ConstSharedPtr;
    void cacheColor(ColorImage msg);
    std::function<green_boundary::Surface(double, double)> pavementSampler(
        const cv::Size &size, double stamp) const;
    void findLaneLines(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp, bool calibrated_frame,
                       LaneLines& lane_lines) const;
    void resolveDoubleMarking(const cv::Mat& mask, bool calibrated_frame, double t,
        const std::optional<association::Pose>& pose, const std::array<ImageLineFit, 2>& prior_tracking,
        FrameResult& frame_result, const DynamicRoiConfig& cfg, LaneLines& lane_lines,
        std::vector<association::Line>& contour_points) const;
    void publishAnnotatedMask(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                              const LaneLines& lane_lines) const;
    void publishContourPoints(const std::vector<std::vector<Eigen::Vector3d>>& contour_points,
                              const builtin_interfaces::msg::Time& timestamp) const;
    void publishLaneLines(const LaneLines& lane_lines, const builtin_interfaces::msg::Time& timestamp) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_image_sub_, color_image_sub_;
    rclcpp::TimerBase::SharedPtr color_qos_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dynamic_roi_image_pub_, linear_fit_image_pub_,
        vehicle_fit_image_pub_;
    std::deque<ColorImage> colors_;
    size_t color_bytes_ = 0;
    std::string camera_frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_mask_image_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lane_line_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> contour_point_pubs_;

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    association::PoseHistory odometry_;
    std::string odometry_frame_;
    mutable std::array<std::optional<association::Pose>, 2> anchor_pose_;
    mutable association::Boundaries anchor_;
    mutable std::array<double, 2> anchor_stamp_{};
    mutable double last_mask_stamp_ = -1.;
    double newest_odom_ = -1.;
    association::Config association_config_;
    double max_prediction_age_ = 1.5;
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
    double min_line_angle_deg = 8.0;
    double max_line_angle_deg = 172.0;
    double min_abs_line_slope = 0.15;
    double max_abs_line_slope = 5.50;
    double global_roi_top_row_ratio = 0.58;
    double global_roi_rect_top_ratio = 0.70;
    double global_roi_top_left_ratio = 0.30;
    double global_roi_top_right_ratio = 0.70;
    double min_lane_spacing_far = 40.0;
    double min_lane_spacing_near = 140.0;
    double xmin = 0.0;
    double xmax = 12.0;
    double ymin = -5.0;
    double ymax = 5.0;
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
    std::vector<cv::Point> pixels;
    ImageLineFit current_line;
    double angle_deg = 0.0;
    double row_coverage_ratio = 0.0;
    int lost_count = 0;
};

struct FrameResult {
    cv::Mat cleaned_mask;
    std::vector<cv::Point> global_roi_polygon;
    std::array<SideResult, NUM_LANE_LINES> side_results;
};

class DynamicRoiProcessor;

std::unique_ptr<DynamicRoiProcessor> g_dynamic_roi_processor;

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

double scoreInitComponent(const cv::Size& image_size, const std::vector<cv::Point>& pixels, const int side,
                          const double center_u) {
    if (pixels.size() < 3) return std::numeric_limits<double>::infinity();

    ImageLineFit line;
    double angle_deg = 0.0;
    if (!fitImageLineUv(pixels, line, angle_deg)) return std::numeric_limits<double>::infinity();
    if (hasInvalidSideGeometry(side, line, angle_deg, DynamicRoiConfig{}))
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

std::vector<cv::Point> selectBestComponent(const cv::Mat& cleaned_mask, const cv::Mat& roi_mask, const int side,
                                           const ImageLineFit* reference_line) {
    cv::Mat masked;
    cv::bitwise_and(cleaned_mask, roi_mask, masked);
    auto components = componentPixels(masked, 3);
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
                score = scoreInitComponent(cleaned_mask.size(), pixels, side, center_u);
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
    return {selectBestComponent(cleaned_mask, roi_mask, side, nullptr), roi_polygon};
}

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> extractTrackingPixels(const cv::Mat& cleaned_mask,
                                                                                const ImageLineFit& previous_line,
                                                                                const int side,
                                                                                const ImageLineFit& opposite_line,
                                                                                const DynamicRoiConfig& cfg) {
    auto [roi_mask, roi_polygon] = buildTrackingRoi(cleaned_mask.size(), previous_line, side, opposite_line, cfg);
    return {selectBestComponent(cleaned_mask, roi_mask, side, &previous_line), roi_polygon};
}

double rowCoverageRatio(const std::vector<cv::Point>& pixels, const int rows, const DynamicRoiConfig& cfg) {
    const int roi_height = std::max(1, rows - topRow(rows, cfg));
    if (pixels.empty()) return 0.0;
    return static_cast<double>(uniqueRowCount(pixels)) / static_cast<double>(roi_height);
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
    const double top_threshold = std::max(3.0 * marginAtRow(v_top, rows, cols, cfg), 0.30 * cols);
    const double bottom_threshold = std::max(3.0 * marginAtRow(v_bottom, rows, cols, cfg), 0.30 * cols);
    return top_jump > top_threshold && bottom_jump > bottom_threshold;
}

bool hasInsufficientLaneSpacing(const int side, const ImageLineFit& current_line, const ImageLineFit& opposite_line,
                                const int rows, const int cols, const DynamicRoiConfig& cfg) {
    if (!current_line.valid || !opposite_line.valid) return false;

    const ImageLineFit& left_line = side == LEFT ? current_line : opposite_line;
    const ImageLineFit& right_line = side == LEFT ? opposite_line : current_line;

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
        return frame;
    }

    const std::array<SideDebugInfo, 2>& debugInfo() const { return debug_info_; }

    void finish(const FrameResult &frame, const std::array<bool, 2> &track) {
        for (int side : {LEFT, RIGHT}) {
            const auto &result = frame.side_results[side];
            if (track[side] && result.status == "ok")
                updateStoredLine(stored_lines_[side], result.current_line);
            else
                registerDetectionFailure(stored_lines_[side], true);
        }
    }
    const std::array<ImageLineFit, 2>& trackingState() const { return stored_lines_; }
    void restoreTracking(const std::array<ImageLineFit, 2>& state) { stored_lines_ = state; }

private:
    SideResult processSide(const cv::Mat& cleaned_mask, const int side) {
        ImageLineFit& stored_line = stored_lines_[side];
        ImageLineFit& opposite_line = stored_lines_[side == LEFT ? RIGHT : LEFT];
        const bool use_tracking_roi = stored_line.valid && stored_line.lost_count <= cfg_.max_lost_frames;

        auto [extracted_pixels, roi_polygon] =
            use_tracking_roi ? extractTrackingPixels(cleaned_mask, stored_line, side, opposite_line, cfg_)
                             : extractInitPixels(cleaned_mask, side, opposite_line, cfg_);

        debug_info_[side] = SideDebugInfo{use_tracking_roi, stored_line, roi_polygon, extracted_pixels};

        SideResult result;
        result.roi_mode = use_tracking_roi ? "tracking" : "init";
        result.pixels = extracted_pixels;
        result.row_coverage_ratio = rowCoverageRatio(result.pixels, cleaned_mask.rows, cfg_);

        if (result.row_coverage_ratio < cfg_.min_row_coverage_ratio && use_tracking_roi) {
            auto fallback = extractInitPixels(cleaned_mask, side, opposite_line, cfg_);
            extracted_pixels = fallback.first;
            roi_polygon = fallback.second;
            debug_info_[side] = SideDebugInfo{false, stored_line, roi_polygon, extracted_pixels};
            result = SideResult{};
            result.roi_mode = "init";
            result.pixels = extracted_pixels;
            result.row_coverage_ratio = rowCoverageRatio(result.pixels, cleaned_mask.rows, cfg_);
        }

        if (result.row_coverage_ratio < cfg_.min_row_coverage_ratio) {
            result.status = "insufficient_row_coverage";
            registerDetectionFailure(stored_line, true);
            result.lost_count = stored_line.lost_count;
            return result;
        }

        ImageLineFit current_line;
        double angle_deg = 0.0;
        const bool fit_ok = fitImageLineUv(result.pixels, current_line, angle_deg);
        result.current_line = current_line;
        result.angle_deg = angle_deg;

        const bool reject_geometry = fit_ok && hasInvalidSideGeometry(side, current_line, angle_deg, cfg_);
        const bool reject_jump =
            fit_ok && hasExcessiveJump(stored_line, current_line, cleaned_mask.rows, cleaned_mask.cols, cfg_);
        const bool reject_spacing =
            fit_ok && hasInsufficientLaneSpacing(side, current_line, opposite_line, cleaned_mask.rows,
                                                 cleaned_mask.cols, cfg_);

        if (!fit_ok) {
            result.status = "too_few_or_degenerate_pixels";
            registerDetectionFailure(stored_line, false);
            result.lost_count = stored_line.lost_count;
            return result;
        }
        if (reject_geometry) {
            result.status = "rejected_side_geometry";
            registerDetectionFailure(stored_line, true);
            result.lost_count = stored_line.lost_count;
            return result;
        }
        if (reject_jump) {
            result.status = "rejected_excessive_jump";
            registerDetectionFailure(stored_line, false);
            result.lost_count = stored_line.lost_count;
            return result;
        }
        if (reject_spacing) {
            result.status = "rejected_lane_spacing";
            registerDetectionFailure(stored_line, true);
            result.lost_count = stored_line.lost_count;
            return result;
        }

        updateStoredLine(stored_line, current_line);
        result.status = "ok";
        result.lost_count = stored_line.lost_count;
        return result;
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

std::vector<Eigen::Vector3d> pixelsToVehiclePoints(const std::vector<cv::Point>& pixels,
                                                   const cv::Mat& inverse_camera_matrix,
                                                   const tf2::Transform& vehicle_T_camera) {
    if (pixels.empty()) return {};

    std::vector<cv::Point2f> pixels_f;
    pixels_f.reserve(pixels.size());
    for (const auto& pixel : pixels) pixels_f.emplace_back(static_cast<float>(pixel.x), static_cast<float>(pixel.y));

    const auto tf_points = pixelsToPoints(pixels_f, inverse_camera_matrix, vehicle_T_camera);
    std::vector<Eigen::Vector3d> vehicle_points;
    vehicle_points.reserve(tf_points.size());
    for (const auto& point : tf_points) vehicle_points.emplace_back(point.x(), point.y(), point.z());
    return vehicle_points;
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

std::vector<Eigen::Vector3d> fitQuadraticVehicleLine(const std::vector<Eigen::Vector3d>& points,
                                                     const DynamicRoiConfig& cfg) {
    if (points.size() < 3 || cfg.xmax <= cfg.xmin || cfg.spacing <= 0.0) return {};

    Eigen::MatrixXd design(points.size(), 3);
    Eigen::VectorXd y(points.size());
    int row = 0;
    for (const auto& point : points) {
        if (!std::isfinite(point.x()) || !std::isfinite(point.y())) continue;
        design(row, 0) = 1.0;
        design(row, 1) = point.x();
        design(row, 2) = point.x() * point.x();
        y(row) = point.y();
        ++row;
    }
    if (row < 3) return {};

    design.conservativeResize(row, 3);
    y.conservativeResize(row);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
    if (qr.rank() < 3) return {};
    const Eigen::Vector3d coeff = qr.solve(y);

    std::vector<Eigen::Vector3d> fitted;
    for (double x = cfg.xmin; x <= cfg.xmax + 1e-9; x += cfg.spacing) {
        const double fitted_y = coeff(0) + coeff(1) * x + coeff(2) * x * x;
        fitted.emplace_back(x, fitted_y, 0.0);
    }
    return fitted;
}

std::vector<Eigen::Vector3d> makeCenterFromSides(const std::vector<Eigen::Vector3d>& left_points,
                                                 const std::vector<Eigen::Vector3d>& right_points) {
    std::vector<Eigen::Vector3d> center_points;
    size_t l = 0, r = 0;
    while (l < left_points.size() && r < right_points.size()) {
        const auto &left = left_points[l];
        const auto &right = right_points[r];
        if (left.x() < right.x() - 1e-6)
            ++l;
        else if (right.x() < left.x() - 1e-6)
            ++r;
        else {
            center_points.emplace_back(0.5 * (left + right));
            ++l;
            ++r;
        }
    }
    return center_points;
}

// Follow separate white runs up the image. Connected components can join both
// stripes near the horizon; one run in a row may extend only one path.
std::vector<std::vector<cv::Point>> stripePaths(const cv::Mat &mask) {
    std::vector<std::vector<cv::Point>> paths;
    std::vector<size_t> active;
    const int gap = std::max(4, mask.rows / 45);
    for (int y = mask.rows - 1; y >= mask.rows / 2; y -= 2) {
        active.erase(
            std::remove_if(active.begin(), active.end(), [&](size_t k) { return paths[k].back().y - y > gap; }),
            active.end());
        const auto *row = mask.ptr<uchar>(y);
        for (int x = 0; x < mask.cols;) {
            if (!row[x]) {
                ++x;
                continue;
            }
            int begin = x;
            while (x < mask.cols && row[x])
                ++x;
            if (x - begin > mask.cols / 6)
                continue;
            const int u = (begin + x - 1) / 2;
            size_t best = paths.size();
            double distance = std::numeric_limits<double>::infinity();
            for (size_t k : active) {
                const auto &path = paths[k];
                const auto &last = path.back();
                const int dv = last.y - y;
                if (dv <= 0)
                    continue;
                const auto &earlier = path[path.size() > 4 ? path.size() - 5 : 0];
                double slope = earlier.y == last.y ? 0. : double(last.x - earlier.x) / (last.y - earlier.y);
                const double e = std::abs(u - (last.x - dv * slope));
                if (e < distance && e <= std::max(4., 0.007 * mask.cols + dv * 1.6)) {
                    best = k;
                    distance = e;
                }
            }
            if (best == paths.size()) {
                active.push_back(best);
                paths.emplace_back();
            }
            paths[best].emplace_back(u, y);
        }
    }
    paths.erase(std::remove_if(
                    paths.begin(), paths.end(),
                    [&](const auto &path) { return path.size() < static_cast<size_t>(std::max(5, mask.rows / 45)); }),
                paths.end());
    std::stable_sort(paths.begin(), paths.end(), [](const auto &a, const auto &b) { return a.size() > b.size(); });
    return paths;
}

struct Marking {
    std::vector<cv::Point> pixels;
    ImageLineFit image_line;
    association::Line raw, points;
};
void addMarking(std::vector<Marking> &out, std::vector<cv::Point> pixels, const DynamicRoiConfig &cfg,
                const cv::Mat &inverse_k, const tf2::Transform &transform) {
    ImageLineFit fit;
    double angle = 0;
    if (!fitImageLineUv(pixels, fit, angle) || isNearlyHorizontal(angle, cfg))
        return;
    auto raw = pixelsToVehiclePoints(pixels, inverse_k, transform);
    const auto crop = cropVehiclePoints(raw, cfg);
    if (crop.size() < 5)
        return;
    const auto limits =
        std::minmax_element(crop.begin(), crop.end(), [](const auto &a, const auto &b) { return a.x() < b.x(); });
    if (limits.second->x() - limits.first->x() < 0.95)
        return;
    auto points = fitQuadraticVehicleLine(crop, cfg);
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&](const auto &p) {
                                    return p.x() < limits.first->x() - 1e-6 || p.x() > limits.second->x() + 1e-6;
                                }),
                 points.end());
    if (points.size() < 3)
        return;
    for (auto &other : out) {
        const auto delta = association::compare(points, other.points);
        if (delta.count >= 3 && std::max(std::abs(delta.high), std::abs(delta.low)) < 0.22) {
            if (points.size() > other.points.size())
                other = {std::move(pixels), fit, std::move(raw), std::move(points)};
            return;
        }
    }
    out.push_back({std::move(pixels), fit, std::move(raw), std::move(points)});
}
// Search the predicted boundary itself, including pixels just outside a stale
// image ROI. A gap between paint dashes does not interrupt the ground corridor.
void searchPrediction(const association::Line &expected, const cv::Mat &binary, const cv::Mat &inverse_k,
                      const tf2::Transform &vehicle_T_camera, const DynamicRoiConfig &cfg, double radius,
                      std::vector<Marking> &candidates, bool join_dashes = false) {
    if (expected.size() < 3)
        return;
    const auto camera_T_vehicle = vehicle_T_camera.inverse();
    const cv::Mat intrinsic = inverse_k.inv();
    std::vector<cv::Point> low, high;
    for (size_t i = 0; i < expected.size(); ++i) {
        const auto &p = expected[i];
        if (p.x() < 0.6 || p.x() > cfg.xmax)
            continue;
        const auto delta = expected[std::min(i + 1, expected.size() - 1)] - expected[i == 0 ? 0 : i - 1];
        const double length = std::hypot(delta.x(), delta.y());
        if (length < 1e-6)
            continue;
        const double nx = -radius * delta.y() / length, ny = radius * delta.x() / length;
        cv::Point2f a, b;
        if (!pointToPixel({p.x() + nx, p.y() + ny, 0}, intrinsic, camera_T_vehicle, a) ||
            !pointToPixel({p.x() - nx, p.y() - ny, 0}, intrinsic, camera_T_vehicle, b))
            continue;
        low.emplace_back(cvRound(a.x), cvRound(a.y));
        high.emplace_back(cvRound(b.x), cvRound(b.y));
    }
    if (low.size() < 2)
        return;
    low.insert(low.end(), high.rbegin(), high.rend());
    cv::Mat corridor = cv::Mat::zeros(binary.size(), CV_8U);
    cv::fillPoly(corridor, std::vector<std::vector<cv::Point>>{low}, 255);
    cv::bitwise_and(corridor, binary, corridor);
    if (join_dashes) {
        // The outer stripe and the color vote constrain this narrow corridor.
        // Join disconnected inner paint through actual mask observations only.
        std::vector<cv::Point> samples, row_points;
        for (int y = binary.rows - 1; y >= binary.rows / 2; y -= 2) {
            cv::findNonZero(corridor.row(y), row_points);
            if (!row_points.empty())
                samples.emplace_back(row_points[row_points.size() / 2].x, y);
        }
        addMarking(candidates, std::move(samples), cfg, inverse_k, vehicle_T_camera);
        return;
    }
    for (auto &path : stripePaths(corridor)) {
        addMarking(candidates, std::move(path), cfg, inverse_k, vehicle_T_camera);
    }
}

// Bridge gaps in a dashed inner stripe only where the same-side outer stripe
// was observed. Raw contour points still contain measured inner paint only.
void extendDashedInner(Marking &inner, const association::Line &guide,
                       const association::Line &outer, const association::Line &outer_raw) {
    const auto offset = association::compare(inner.points, guide);
    if (offset.count < 4 || std::max(std::abs(offset.low), std::abs(offset.high)) > 0.15) return;
    association::Line fit;
    for (const auto &p : guide) {
        if (p.x() >= inner.points.front().x() && p.x() <= inner.points.back().x()) {
            fit.emplace_back(p.x(), green_boundary::at(inner.points, p.x()).y, 0.);
            continue;
        }
        const auto witnessed = [&](const Eigen::Vector3d &q) {
            if (std::abs(q.x() - p.x()) > 0.35) return false;
            const auto f = green_boundary::at(outer, q.x());
            return std::abs(q.y() - f.y) / std::hypot(1., f.slope) <= 0.20;
        };
        if (std::any_of(outer_raw.begin(), outer_raw.end(), witnessed))
            fit.push_back(p);
        else if (!fit.empty() && p.x() > inner.points.back().x())
            break;
        else
            fit.clear();
    }
    if (fit.size() > inner.points.size()) inner.points = std::move(fit);
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

void publishDynamicRoiVisualization(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher,
                                    const cv::Mat& source_mask, const builtin_interfaces::msg::Time& timestamp,
                                    const FrameResult& frame_result, const DynamicRoiProcessor& processor) {
    cv::Mat visualization = imageToBgr(source_mask);
    blendPolygon(visualization, frame_result.global_roi_polygon, cv::Scalar(255, 255, 0), 0.18);

    const auto& debug_info = processor.debugInfo();
    for (const int side : {LEFT, RIGHT}) {
        blendPolygon(visualization, debug_info[side].roi_polygon, sideColor(side), 0.20);
        for (const auto &p : frame_result.side_results[side].pixels)
            if (cv::Rect(0, 0, visualization.cols, visualization.rows).contains(p))
                cv::circle(visualization, p, 1, sideColor(side), -1);
    }

    publishBgrImage(publisher, timestamp, visualization);
}

void publishLinearFitVisualization(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher,
                                   const cv::Mat& source_mask, const builtin_interfaces::msg::Time& timestamp,
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

    publishBgrImage(publisher, timestamp, visualization);
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

    cv::putText(image, "vehicle frame fitted output", cv::Point(18, 26), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                cv::Scalar(40, 40, 40), 2, cv::LINE_AA);
    cv::putText(image, "x forward", cv::Point(width / 2 - 48, height - 16), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                cv::Scalar(70, 70, 70), 1, cv::LINE_AA);
    cv::putText(image, "y left", cv::Point(8, height / 2), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(70, 70, 70), 1,
                cv::LINE_AA);
    return image;
}

void publishVehicleFitVisualization(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher,
                                    const builtin_interfaces::msg::Time& timestamp, const LaneLines& lane_lines,
                                    const DynamicRoiConfig& cfg) {
    publishBgrImage(publisher, timestamp, makeVehicleFitVisualization(lane_lines, cfg));
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
    camera_frame_id_ = getRosParameter<std::string>(this, "camera_frame_id");
    const auto camera_name = getRosParameter<std::string>(this, "camera_name");
    const auto min_area = getRosParameter<int>(this, "lane_pixel_finder.min_area");
    const auto tolerance = getRosParameter<int>(this, "lane_pixel_finder.tolerance");

    association_config_.min_width = declare_parameter<double>("lane_line_publisher.min_width_m", 2.1);
    association_config_.expected_width = declare_parameter<double>("lane_line_publisher.expected_width_m", 3.5);
    association_config_.max_width = declare_parameter<double>("lane_line_publisher.max_width_m", 4.9);
    association_config_.match_distance = declare_parameter<double>("lane_line_publisher.match_distance_m", 0.85);
    max_prediction_age_ = declare_parameter<double>("lane_line_publisher.history_sec", 1.5);
    if (!(association_config_.min_width > 0 && association_config_.min_width < association_config_.expected_width &&
          association_config_.expected_width < association_config_.max_width &&
          association_config_.match_distance > 0 && max_prediction_age_ > 0 && spacing_ > 0))
        throw std::invalid_argument("Invalid lane association parameters");

    getCameraParams(this, camera_name, camera_matrix_);
    camera_matrix_ = camera_matrix_.inv();
    configured_camera_matrix_ = camera_matrix_;
    // A cold vehicle can bring up static TF after perception. Wait for the
    // calibrated transform before creating the mask subscription.
    tf2_ros::Buffer tf_buffer(get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    const auto transform = tf_buffer.lookupTransform(vehicle_frame_id_, camera_frame_id_,
        tf2::TimePointZero, tf2::durationFromSec(8.0)).transform;
    const auto &q = transform.rotation;
    const auto &p = transform.translation;
    vehicle_T_camera_ = tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(p.x, p.y, p.z));

    g_dynamic_roi_processor = std::make_unique<DynamicRoiProcessor>(
        makeDynamicRoiConfig(min_area, tolerance, xmin_, xmax_, ymin_, ymax_, spacing_));
}

void LaneLinePublisher::subscribeColor(bool reliable) {
    rclcpp::QoS qos(rclcpp::KeepLast(2));
    if (!reliable) qos.best_effort();
    color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "color_image", qos,
        std::bind(&LaneLinePublisher::cacheColor, this, std::placeholders::_1));
}

void LaneLinePublisher::matchColorQoS() {
    const auto publishers = get_publishers_info_by_topic(color_image_sub_->get_topic_name());
    if (publishers.empty()) return;
    const bool reliable = std::all_of(publishers.begin(), publishers.end(), [](const auto &p) {
        return p.qos_profile().get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    });
    const auto current = color_image_sub_->get_actual_qos().get_rmw_qos_profile().reliability;
    if (reliable != (current == RMW_QOS_POLICY_RELIABILITY_RELIABLE)) subscribeColor(reliable);
}

void LaneLinePublisher::initConnections() {
    const auto queue_size = getRosParameter<int>(this, "lane_line_publisher.queue_size");
    // The vehicle publishes reliable images. Keep retransmission for large, fragmented
    // frames, and still connect if a best-effort camera starts or replaces it.
    subscribeColor(true);
    matchColorQoS();
    color_qos_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                        std::bind(&LaneLinePublisher::matchColorQoS, this));
    camera_info_sub_ = create_subscription<CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        std::bind(&LaneLinePublisher::cacheCalibration, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS().keep_last(100),
        std::bind(&LaneLinePublisher::odometryCallback, this, std::placeholders::_1));
    mask_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "mask_image", queue_size, std::bind(&LaneLinePublisher::imageCallback, this, std::placeholders::_1));
    lane_line_pubs_ = {create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/left", queue_size),
                       create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/right", queue_size),
                       create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/center", queue_size)};

    dynamic_roi_image_pub_ = create_publisher<sensor_msgs::msg::Image>("dynamic_roi_image", queue_size);
    linear_fit_image_pub_ = create_publisher<sensor_msgs::msg::Image>("linear_fit_image", queue_size);
    vehicle_fit_image_pub_ = create_publisher<sensor_msgs::msg::Image>("vehicle_fit_image", queue_size);

    if (!debug_) return;

    annotated_mask_image_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_mask_image", queue_size);
    contour_point_pubs_ = {create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/left", queue_size),
                           create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/right", queue_size),
                           create_publisher<sensor_msgs::msg::PointCloud2>("contour_points/center", queue_size)};
}

void LaneLinePublisher::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (msg->child_frame_id != vehicle_frame_id_ || msg->header.frame_id.empty())
        return;
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    const double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!(norm > 0.99 && norm < 1.01) || !std::isfinite(p.x) || !std::isfinite(p.y))
        return;
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    if (msg->header.frame_id != odometry_frame_ || t < newest_odom_ - 5.) {
        odometry_.clear();
        anchor_pose_ = {};
        odometry_frame_ = msg->header.frame_id;
        newest_odom_ = t;
    }
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), norm - 2 * (q.y * q.y + q.z * q.z));
    newest_odom_ = std::max(t, newest_odom_);
    odometry_.add({t, p.x, p.y, yaw});
}

void LaneLinePublisher::cacheCalibration(CameraInfo::ConstSharedPtr info) {
    // For the left rectified image, P maps camera-frame points into pixels.
    // The projection below assumes a full-frame, unbinned pinhole image.
    const auto &p = info->p;
    const auto &roi = info->roi;
    if (info->header.frame_id != camera_frame_id_ || info->width == 0 || info->height == 0 ||
        !(info->k[0] > 0.) || info->binning_x > 1 || info->binning_y > 1 ||
        roi.x_offset || roi.y_offset || (roi.width && roi.width != info->width) ||
        (roi.height && roi.height != info->height) ||
        !std::all_of(p.begin(), p.end(), [](double value) { return std::isfinite(value); }) ||
        p[0] <= 0. || p[5] <= 0. || p[2] < 0. || p[2] >= info->width ||
        p[6] < 0. || p[6] >= info->height || p[1] || p[3] || p[4] || p[7] ||
        p[8] || p[9] || p[10] != 1. || p[11]) return;

    // CameraInfo repeats every exposure. Retain changes with their first stamp,
    // so a delayed mask can still use the calibration in force at capture.
    const auto stamp = rclcpp::Time(info->header.stamp);
    const auto it = std::lower_bound(calibrations_.begin(), calibrations_.end(), stamp,
        [](const auto &entry, const auto &t) { return rclcpp::Time(entry->header.stamp) < t; });
    if (it != calibrations_.end() && rclcpp::Time((*it)->header.stamp) == stamp) {
        *it = std::move(info);
    } else {
        if (it != calibrations_.begin()) {
            const auto &prev = *std::prev(it);
            if (prev->p == p && prev->width == info->width && prev->height == info->height) return;
        }
        calibrations_.insert(it, std::move(info));
    }
    if (calibrations_.size() > 16) calibrations_.pop_front();
}

void LaneLinePublisher::selectCalibration(const sensor_msgs::msg::Image& mask) {
    CameraInfo::ConstSharedPtr selected;
    const auto stamp = rclcpp::Time(mask.header.stamp);
    if (mask.header.frame_id == camera_frame_id_) {
        for (auto it = calibrations_.rbegin(); it != calibrations_.rend(); ++it) {
            const auto &info = *it;
            if (rclcpp::Time(info->header.stamp) > stamp) continue;
            if (info->width == mask.width && info->height == mask.height) selected = info;
            break;
        }
    }
    if (selected == active_calibration_) return;
    cv::Mat inverse = configured_camera_matrix_;
    if (selected) {
        const auto &p = selected->p;
        cv::Mat k = (cv::Mat_<float>(3, 3) << p[0], 0., p[2], 0., p[5], p[6], 0., 0., 1.);
        inverse = k.inv();
    }
    active_calibration_ = selected;
    if (cv::norm(camera_matrix_, inverse, cv::NORM_INF) == 0.) return;
    camera_matrix_ = inverse;
    anchor_pose_ = {};
    g_dynamic_roi_processor->restoreTracking({});
}

void LaneLinePublisher::cacheColor(ColorImage msg) {
    if (msg->header.frame_id != camera_frame_id_ ||
        (msg->encoding != "bgr8" && msg->encoding != "rgb8" &&
         msg->encoding != "bgra8" && msg->encoding != "rgba8"))
        return;
    color_bytes_ += msg->data.size();
    colors_.push_back(std::move(msg));
    while (colors_.size() > 32 || color_bytes_ > 32 * 1024 * 1024) {
        color_bytes_ -= colors_.front()->data.size();
        colors_.pop_front();
    }
}

std::function<green_boundary::Surface(double, double)> LaneLinePublisher::pavementSampler(
        const cv::Size &size, double t) const {
    ColorImage closest;
    double skew = 0.15; // Only frames already received, with at most 150 ms of capture skew.
    for (const auto &msg : colors_) {
        const double dt = std::abs(rclcpp::Time(msg->header.stamp).seconds() - t);
        if (dt <= skew && msg->width == static_cast<unsigned>(size.width) &&
            msg->height == static_cast<unsigned>(size.height)) {
            closest = msg;
            skew = dt;
        }
    }
    if (!closest) return {};
    // For different exposures, express mask-frame ground samples in the color
    // frame using two stamped odometry poses. Never wait on camera or odometry.
    double tx = 0., ty = 0., c = 1., s = 0.;
    if (skew > 0.025) {
        const auto a = odometry_.at(t, 0.025);
        const auto b = odometry_.at(rclcpp::Time(closest->header.stamp).seconds(), 0.025);
        if (!a || !b) return {};
        const double dx = a->x - b->x, dy = a->y - b->y;
        tx = std::cos(b->yaw) * dx + std::sin(b->yaw) * dy;
        ty = -std::sin(b->yaw) * dx + std::cos(b->yaw) * dy;
        c = std::cos(a->yaw - b->yaw);
        s = std::sin(a->yaw - b->yaw);
    }
    // Read the original layout in place; only sparse pavement patches are used.
    const int channels = closest->encoding.size() == 5 ? 4 : 3;
    const bool rgb = closest->encoding.front() == 'r';
    if (closest->step < closest->width * channels ||
        closest->data.size() < static_cast<size_t>(closest->step) * closest->height)
        return {};
    const cv::Mat intrinsic = camera_matrix_.inv();
    const auto camera_T_vehicle = vehicle_T_camera_.inverse();
    return [=](double x, double y) {
        using green_boundary::Surface;
        const auto p = camera_T_vehicle * tf2::Vector3(tx + c*x - s*y, ty + s*x + c*y, 0.);
        if (p.z() <= 0.) return Surface::unknown;
        const int u = cvRound(intrinsic.at<float>(0, 0) * p.x()/p.z() + intrinsic.at<float>(0, 2));
        const int v = cvRound(intrinsic.at<float>(1, 1) * p.y()/p.z() + intrinsic.at<float>(1, 2));
        if (u < 2 || v < 2 || u + 2 >= size.width || v + 2 >= size.height)
            return Surface::unknown;
        int green = 0, pavement = 0;
        for (int row = v - 2; row <= v + 2; ++row) {
            auto pixel = closest->data.data() + static_cast<size_t>(row) * closest->step + (u - 2) * channels;
            for (int col = 0; col < 5; ++col, pixel += channels) {
                const auto cls = green_boundary::surface(pixel[rgb ? 0 : 2], pixel[1], pixel[rgb ? 2 : 0]);
                green += cls == Surface::green;
                pavement += cls == Surface::pavement;
            }
        }
        if (green >= 16) return Surface::green;
        if (pavement >= 16) return Surface::pavement;
        return Surface::unknown;
    };
}

void LaneLinePublisher::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Received image [%ix%i]", static_cast<int>(msg->width), static_cast<int>(msg->height));

    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    if (cv_img->image.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty image");
        return;
    }

    selectCalibration(*msg);
    LaneLines lane_lines;
    findLaneLines(cv_img->image, msg->header.stamp, msg->header.frame_id == camera_frame_id_, lane_lines);
    publishLaneLines(lane_lines, msg->header.stamp);
}

void LaneLinePublisher::findLaneLines(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                                      bool calibrated_frame, LaneLines& lane_lines) const {
    if (!g_dynamic_roi_processor) return;

    const double t = rclcpp::Time(timestamp).seconds();
    if (t < last_mask_stamp_) g_dynamic_roi_processor->restoreTracking({});
    if (t <= last_mask_stamp_ || t - last_mask_stamp_ > max_prediction_age_) anchor_pose_ = {};
    last_mask_stamp_ = t;
    const auto prior_tracking = g_dynamic_roi_processor->trackingState();
    auto frame_result = g_dynamic_roi_processor->findLanePixels(mask);
    const auto cfg = makeDynamicRoiConfig(0, 0, xmin_, xmax_, ymin_, ymax_, spacing_);
    std::vector<std::vector<Eigen::Vector3d>> contour_points(NUM_LANE_LINES);
    for (int side : {LEFT, RIGHT}) {
        auto &lane = side == LEFT ? lane_lines.left : lane_lines.right;
        lane.pixels = frame_result.side_results[side].pixels;
        contour_points[side] = pixelsToVehiclePoints(lane.pixels, camera_matrix_, vehicle_T_camera_);
        lane.points = fitQuadraticVehicleLine(cropVehiclePoints(contour_points[side], cfg), cfg);
    }
    const auto pose = odometry_.at(t);
    const auto &left = lane_lines.left.points, &right = lane_lines.right.points;
    const auto separation = association::compare(left, right, 6.);
    const bool complete_lane = separation.count >= 3 && separation.low >= association_config_.min_width &&
                               separation.high <= association_config_.max_width;
    if (pose && complete_lane) {
        anchor_ = {left, right};
        anchor_pose_ = {pose, pose};
        anchor_stamp_ = {t, t};
    }
    if (!complete_lane)
        resolveDoubleMarking(mask, calibrated_frame, t, pose, prior_tracking, frame_result, cfg, lane_lines, contour_points);
    lane_lines.center.points = makeCenterFromSides(lane_lines.left.points, lane_lines.right.points);
    contour_points[CENTER] = lane_lines.center.points;
    publishDynamicRoiVisualization(dynamic_roi_image_pub_, mask, timestamp, frame_result, *g_dynamic_roi_processor);
    publishLinearFitVisualization(linear_fit_image_pub_, mask, timestamp, frame_result);
    if (debug_) publishAnnotatedMask(frame_result.cleaned_mask, timestamp, lane_lines);
    publishVehicleFitVisualization(vehicle_fit_image_pub_, timestamp, lane_lines, cfg);
    if (debug_) publishContourPoints(contour_points, timestamp);
}

void LaneLinePublisher::resolveDoubleMarking(const cv::Mat& mask, bool calibrated_frame, double t,
        const std::optional<association::Pose>& pose, const std::array<ImageLineFit, 2>& prior_tracking,
        FrameResult& frame_result, const DynamicRoiConfig& cfg, LaneLines& lane_lines,
        std::vector<association::Line>& contour_points) const {
    std::vector<Marking> candidates;
    // The global mask retains stripes on the far side of a double marking even
    // when both image ROIs have collapsed onto the same physical boundary.
    for (auto &path : stripePaths(frame_result.cleaned_mask)) {
        addMarking(candidates, std::move(path), cfg, camera_matrix_, vehicle_T_camera_);
    }
    for (int side : {LEFT, RIGHT}) {
        const auto &proposal = frame_result.side_results[side];
        if (proposal.status == "ok")
            addMarking(candidates, proposal.pixels, cfg, camera_matrix_, vehicle_T_camera_);
    }
    association::Boundaries predicted;
    for (int s : {LEFT, RIGHT}) {
        if (pose && anchor_pose_[s] && t >= anchor_stamp_[s] && t - anchor_stamp_[s] <= max_prediction_age_)
            predicted[s] = association::predict(anchor_[s], *anchor_pose_[s], *pose);
    }
    const cv::Mat binary = toBinaryMask(mask);
    auto recovery = cfg;
    const double reach = association_config_.expected_width + association_config_.match_distance;
    recovery.ymin = std::min(recovery.ymin, -reach);
    recovery.ymax = std::max(recovery.ymax, reach);
    if (!predicted[LEFT].empty() || !predicted[RIGHT].empty()) {
        for (int side : {LEFT, RIGHT})
            searchPrediction(predicted[side], binary, camera_matrix_, vehicle_T_camera_, recovery,
                             association_config_.match_distance, candidates);
    }
    for (auto &path : stripePaths(binary))
        addMarking(candidates, std::move(path), recovery, camera_matrix_, vehicle_T_camera_);
    std::vector<association::Line> lines;
    for (const auto &candidate : candidates) lines.push_back(candidate.points);
    const association::Boundaries original{lane_lines.left.points, lane_lines.right.points};
    if (!green_boundary::hasRelevantDouble(lines, original)) return;
    const auto geometric = association::select(lines, predicted, association_config_);
    auto selection = geometric;
    association::Selection color_side;
    if (calibrated_frame && !colors_.empty()) {
        const auto sample = pavementSampler(mask.size(), t);
        if (sample) color_side = green_boundary::resolve(lines, {}, sample, original);
    }
    const bool colored = color_side.left >= 0 || color_side.right >= 0;
    if (colored && (selection.left < 0 || selection.right < 0)) selection = color_side;
    // An existing single already drives lane_points' one-side fallback. Only
    // positive identity evidence can justify replacing it. Keep the original
    // extent if it already follows the confirmed inner stripe on the same side.
    if (original[LEFT].empty() != original[RIGHT].empty()) {
        if (!colored) return;
        const int side = original[LEFT].empty() ? RIGHT : LEFT;
        const int inner = side == LEFT ? color_side.left : color_side.right;
        if (inner >= 0 && (selection.left < 0 || selection.right < 0)) {
            const auto d = association::compare(original[side], lines[inner], 6.);
            if (d.count >= 3 && std::max(std::abs(d.low), std::abs(d.high)) < 0.22) return;
            // A distant dash may give a shorter cloud than the original
            // (outer) stripe. Use the measured offset to search for more paint.
            if (d.count >= 3 && std::abs(d.median) < 1.0 && lines[inner].size() < original[side].size()) {
                auto guide = original[side];
                for (auto &p : guide) {
                    const double slope = green_boundary::at(original[side], p.x()).slope;
                    p.y() -= d.median * std::hypot(1., slope);
                }
                std::vector<Marking> recovered;
                searchPrediction(guide, binary, camera_matrix_, vehicle_T_camera_, recovery,
                                 std::min(0.25, 0.35 * std::abs(d.median)), recovered, true);
                if (!recovered.empty() && recovered[0].points.size() > lines[inner].size()) {
                    const auto agreement = association::compare(recovered[0].points, lines[inner], 6.);
                    const auto follows = association::compare(recovered[0].points, guide);
                    if (agreement.count >= 3 && follows.count >= 3 &&
                        std::max(std::abs(agreement.low), std::abs(agreement.high)) <= 0.20 &&
                        std::max(std::abs(follows.low), std::abs(follows.high)) <= 0.25)
                        candidates[inner] = std::move(recovered[0]);
                }
                const double signed_gap = side == LEFT ? d.median : -d.median;
                if (signed_gap > 0.22 && d.high - d.low <= 0.15) {
                    // The tracking ROI may end before the visible outer stripe.
                    // Recheck its continuation in the full mask before using it.
                    std::vector<Marking> outer_paint;
                    searchPrediction(original[side], binary, camera_matrix_, vehicle_T_camera_,
                                     recovery, 0.20, outer_paint, true);
                    auto support = contour_points[side];
                    if (!outer_paint.empty())
                        support.insert(support.end(), outer_paint[0].raw.begin(), outer_paint[0].raw.end());
                    extendDashedInner(candidates[inner], guide, original[side], support);
                }
            }
        }
    }
    g_dynamic_roi_processor->restoreTracking(prior_tracking);
    const std::array<bool, 2> track{{selection.left == geometric.left, selection.right == geometric.right}};
    for (int side : {LEFT, RIGHT}) {
        auto &result = frame_result.side_results[side];
        result = SideResult{};
        auto &output = side == LEFT ? lane_lines.left : lane_lines.right;
        output = LaneLine{};
        contour_points[side].clear();
        const int id = side == LEFT ? selection.left : selection.right;
        if (id < 0)
            continue;
        auto &marking = candidates[id];
        output.points = marking.points;
        output.pixels = std::move(marking.pixels);
        contour_points[side] = std::move(marking.raw);
        result.current_line = marking.image_line;
        result.pixels = output.pixels;
        result.status = "ok";
    }
    if (pose) {
        for (int s : {LEFT, RIGHT}) {
            const auto &measured = s == LEFT ? lane_lines.left.points : lane_lines.right.points;
            if (!track[s] || measured.empty())
                continue;
            anchor_[s] = measured;
            anchor_pose_[s] = pose;
            anchor_stamp_[s] = t;
        }
    }
    g_dynamic_roi_processor->finish(frame_result, track);
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
    // The stock vehicle launch remaps the mask and clouds. Supply its two new
    // sensor inputs here; any explicit ROS remaps in argv take precedence.
    std::vector<const char*> args(argv, argv + argc);
    args.insert(args.end(), {"--ros-args",
        "-r", "color_image:=/aiformula_sensing/zed_node/left_image/undistorted",
        "-r", "camera_info:=/aiformula_sensing/zed_node/left/camera_info",
        "-r", "odom:=/aiformula_sensing/gyro_odometry_publisher/odom"});
    rclcpp::init(static_cast<int>(args.size()), args.data());
    try {
        rclcpp::spin(std::make_shared<aiformula::LaneLinePublisher>());
    } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("lane_line_publisher"), "Camera transform unavailable: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
