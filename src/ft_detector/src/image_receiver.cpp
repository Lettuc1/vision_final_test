// =============================================================================
//  image_receiver.cpp  --  traditional RoboMaster armour detector
//
//  Pipeline
//  --------
//    compressed image (image_transport) + camera_info
//        -> HSV colour mask (red wraps around hue 0, blue)
//        -> merge in the over-exposed white core of the LED
//        -> close + fill holes  (fixes the "hollow light bar" problem)
//        -> connected components -> light-bar geometry / brightness checks
//        -> symmetric bar pairing -> big/small classification
//        -> armour-number evidence (top-hat) -> score -> NMS
//        -> cv::solvePnP (four light-bar endpoints, definition 1)
//        -> /ft/detection, /ft/debug/image, /ft/debug/mask, TF
//
//  Why the old version failed on the bags
//  --------------------------------------
//  * A strong LED clips to white in the centre.  A strict HSV mask therefore
//    produces two thin coloured strips or a hollow ring.  Sampling the colour
//    at the centre (old code) then returns black -> every real bar is thrown
//    away.  Here the white core is added back only next to coloured pixels and
//    the external contour is filled.
//  * Floor/floor-wax reflections have the same hue but are diffuse: their
//    90th-percentile V is much lower and they contain no white-clipped core.
//    A real LED has a white (red) or extremely saturated (blue) core.  Both
//    are checked, together with strict symmetric-pairing and plate-interior
//    tests, so reflections no longer pair into a fake armour.
//  * Big/small is decided from the gap / bar-length *ratio* (scale invariant),
//    not from a hard-coded pixel distance.
//
//  Everything that affects behaviour is read from YAML and (for the required
//  subset) exposed through dynamic_reconfigure.  No GUI windows are opened.
// =============================================================================

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>
#include <ft_detector/ArmorReconfigureConfig.h>
#include <ft_detection/DetectionFrame.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace ft_armor {

constexpr double kPi = 3.14159265358979323846;

// =============================================================================
// Parameters
// =============================================================================
struct Params {
    // topics / transport
    std::string image_topic            = "/hk_camera/image_raw";
    std::string camera_info_topic      = "/hk_camera/camera_info";
    std::string image_transport        = "compressed";
    int         queue_size             = 2;
    std::string detection_topic        = "/ft/detection";
    std::string debug_image_topic      = "/ft/debug/image";
    std::string debug_mask_topic       = "/ft/debug/mask";
    bool        publish_mask           = true;
    bool        publish_tf             = true;
    bool        use_dynamic_reconfigure = true;

    // camera fallback (only used if the bag has no camera_info)
    std::string camera_frame_fallback  = "camera_optical_frame";
    double      fallback_fx            = 1000.0;
    double      fallback_fy            = 1000.0;
    double      fallback_cx            = 720.0;
    double      fallback_cy            = 540.0;
    std::vector<double> fallback_distortion{0.0, 0.0, 0.0, 0.0, 0.0};

    // HSV - red (two hue windows because red wraps around 0)
    int red_h_min1 = 0;
    int red_h_max1 = 20;
    int red_h_min2 = 160;
    int red_h_max2 = 180;
    int red_s_min  = 100;
    int red_s_max  = 255;
    int red_v_min  = 130;
    int red_v_max  = 255;

    // HSV - blue
    int blue_h_min = 90;
    int blue_h_max = 140;
    int blue_s_min = 100;
    int blue_s_max = 255;
    int blue_v_min = 130;
    int blue_v_max = 255;

    // over-exposed LED core (white pixels close to coloured pixels)
    int white_s_max = 110;
    int white_v_min = 215;
    int white_dilate = 5;

    // light-bar geometry
    int    min_contour_area = 40;
    double min_bar_length   = 8.0;
    double min_bar_width    = 2.0;
    double min_bar_ratio    = 1.2;
    double max_bar_ratio    = 12.0;
    double max_bar_tilt     = 40.0;   // deg from vertical

    // light-bar photometry
    int    min_bar_p90_v   = 225;
    double min_bar_mean_v  = 70.0;
    double min_white_ratio = 0.02;    // red: white-clipped core
    double min_sat_ratio   = 0.30;    // blue: saturated-blue core
    int    sat_s_min       = 150;
    int    sat_v_min       = 150;

    // pairing
    int    max_pair_distance          = 1100;
    double max_pair_angle             = 30.0;  // deg between bar axes (perspective)
    double max_pair_tilt              = 12.0;  // deg of centre line
    double max_pair_center_dy_ratio   = 0.35;  // |dy| / mean bar length
    double max_endpoint_diff_ratio    = 0.20;
    double max_length_ratio_diff      = 0.20;
    double min_width_ratio            = 0.55;
    double min_pair_gap_factor        = 0.60;
    // Fragments of one physical light bar can be split into two components by
    // an over-exposed white core.  These thresholds merge such duplicates.
    double bar_merge_center_factor    = 1.5;
    double bar_merge_length_factor    = 0.30;
    double bar_merge_overlap          = 0.35;
    double max_armor_overlap_ratio    = 0.35;
    double pair_centrality_weight     = 2.0;
    double pair_centrality_y_band     = 2.0;
    bool   reject_floor_reflection     = true;
    double reflection_min_vertical_factor = 0.60;

    // plate interior
    double max_inner_mean_v          = 170.0;
    double max_inner_colored_ratio   = 0.15;

    // physical dimensions (definition 1: four light-bar endpoints)
    double armor_small_width    = 120.0;  // mm, light-bar centre distance
    double armor_large_width    = 235.0;  // mm
    double lightbar_length      = 55.0;   // mm
    double armor_width_tolerance = 0.65;       // small armour lower factor
    double armor_width_upper_tolerance = 0.30; // small armour upper factor
    double armor_large_gap_lower_factor = 0.85; // large armour lower factor
    double armor_large_gap_upper_factor = 1.10; // large armour upper factor
    double max_large_box_aspect = 4.70;         // width / mean bar height
    double wide_large_aspect_min = 4.00;        // require number evidence above this
    double min_number_score_wide_large = 0.08;
    double min_number_score_large = 0.05;

    // PnP sanity
    double max_reproj_error = 20.0;  // px RMSE
    double min_depth        = 0.10;  // m
    double max_depth        = 50.0;  // m

    // number evidence (top-hat)
    int    number_tophat_kernel     = 31;
    double number_extend            = 0.85;
    double number_inset             = 0.05;
    int    number_s_max             = 170;
    int    number_thr_percentile    = 70;
    double number_min_area_ratio    = 0.002;
    double number_max_area_ratio    = 0.45;
};

template <typename T>
T readParam(ros::NodeHandle& nh, const std::string& key, const T& fallback) {
    T value;
    if (nh.getParam(key, value)) {
        return value;
    }
    return fallback;
}

Params loadParams(ros::NodeHandle& pnh) {
    Params p;

    p.image_topic       = readParam<std::string>(pnh, "image_topic", p.image_topic);
    p.camera_info_topic = readParam<std::string>(pnh, "camera_info_topic", p.camera_info_topic);
    p.image_transport   = readParam<std::string>(pnh, "image_transport", p.image_transport);
    p.queue_size        = readParam<int>(pnh, "queue_size", p.queue_size);
    p.detection_topic   = readParam<std::string>(pnh, "detection_topic", p.detection_topic);
    p.debug_image_topic = readParam<std::string>(pnh, "debug_image_topic", p.debug_image_topic);
    p.debug_mask_topic  = readParam<std::string>(pnh, "debug_mask_topic", p.debug_mask_topic);
    p.publish_mask      = readParam<bool>(pnh, "publish_mask", p.publish_mask);
    p.publish_tf        = readParam<bool>(pnh, "publish_tf", p.publish_tf);
    p.use_dynamic_reconfigure =
        readParam<bool>(pnh, "use_dynamic_reconfigure", p.use_dynamic_reconfigure);

    p.camera_frame_fallback =
        readParam<std::string>(pnh, "camera_frame_fallback", p.camera_frame_fallback);
    p.fallback_fx = readParam<double>(pnh, "fallback_fx", p.fallback_fx);
    p.fallback_fy = readParam<double>(pnh, "fallback_fy", p.fallback_fy);
    p.fallback_cx = readParam<double>(pnh, "fallback_cx", p.fallback_cx);
    p.fallback_cy = readParam<double>(pnh, "fallback_cy", p.fallback_cy);
    p.fallback_distortion =
        readParam<std::vector<double>>(pnh, "fallback_distortion", p.fallback_distortion);

    p.red_h_min1 = readParam<int>(pnh, "red_h_min1", p.red_h_min1);
    p.red_h_max1 = readParam<int>(pnh, "red_h_max1", p.red_h_max1);
    p.red_h_min2 = readParam<int>(pnh, "red_h_min2", p.red_h_min2);
    p.red_h_max2 = readParam<int>(pnh, "red_h_max2", p.red_h_max2);
    p.red_s_min  = readParam<int>(pnh, "red_s_min", p.red_s_min);
    p.red_s_max  = readParam<int>(pnh, "red_s_max", p.red_s_max);
    p.red_v_min  = readParam<int>(pnh, "red_v_min", p.red_v_min);
    p.red_v_max  = readParam<int>(pnh, "red_v_max", p.red_v_max);

    p.blue_h_min = readParam<int>(pnh, "blue_h_min", p.blue_h_min);
    p.blue_h_max = readParam<int>(pnh, "blue_h_max", p.blue_h_max);
    p.blue_s_min = readParam<int>(pnh, "blue_s_min", p.blue_s_min);
    p.blue_s_max = readParam<int>(pnh, "blue_s_max", p.blue_s_max);
    p.blue_v_min = readParam<int>(pnh, "blue_v_min", p.blue_v_min);
    p.blue_v_max = readParam<int>(pnh, "blue_v_max", p.blue_v_max);

    p.white_s_max = readParam<int>(pnh, "white_s_max", p.white_s_max);
    p.white_v_min = readParam<int>(pnh, "white_v_min", p.white_v_min);
    p.white_dilate = readParam<int>(pnh, "white_dilate", p.white_dilate);

    p.min_contour_area = readParam<int>(pnh, "min_contour_area", p.min_contour_area);
    p.min_bar_length   = readParam<double>(pnh, "min_bar_length", p.min_bar_length);
    p.min_bar_width    = readParam<double>(pnh, "min_bar_width", p.min_bar_width);
    p.min_bar_ratio    = readParam<double>(pnh, "min_bar_ratio", p.min_bar_ratio);
    p.max_bar_ratio    = readParam<double>(pnh, "max_bar_ratio", p.max_bar_ratio);
    p.max_bar_tilt     = readParam<double>(pnh, "max_bar_tilt", p.max_bar_tilt);

    p.min_bar_p90_v   = readParam<int>(pnh, "min_bar_p90_v", p.min_bar_p90_v);
    p.min_bar_mean_v  = readParam<double>(pnh, "min_bar_mean_v", p.min_bar_mean_v);
    p.min_white_ratio = readParam<double>(pnh, "min_white_ratio", p.min_white_ratio);
    p.min_sat_ratio   = readParam<double>(pnh, "min_sat_ratio", p.min_sat_ratio);
    p.sat_s_min       = readParam<int>(pnh, "sat_s_min", p.sat_s_min);
    p.sat_v_min       = readParam<int>(pnh, "sat_v_min", p.sat_v_min);

    p.max_pair_distance       = readParam<int>(pnh, "max_pair_distance", p.max_pair_distance);
    p.max_pair_angle          = readParam<double>(pnh, "max_pair_angle", p.max_pair_angle);
    p.max_pair_tilt           = readParam<double>(pnh, "max_pair_tilt", p.max_pair_tilt);
    p.max_pair_center_dy_ratio = readParam<double>(pnh, "max_pair_center_dy_ratio", p.max_pair_center_dy_ratio);
    p.max_endpoint_diff_ratio = readParam<double>(pnh, "max_endpoint_diff_ratio", p.max_endpoint_diff_ratio);
    p.max_length_ratio_diff   = readParam<double>(pnh, "max_length_ratio_diff", p.max_length_ratio_diff);
    p.min_width_ratio         = readParam<double>(pnh, "min_width_ratio", p.min_width_ratio);
    p.min_pair_gap_factor     = readParam<double>(pnh, "min_pair_gap_factor", p.min_pair_gap_factor);
    p.bar_merge_center_factor = readParam<double>(pnh, "bar_merge_center_factor", p.bar_merge_center_factor);
    p.bar_merge_length_factor = readParam<double>(pnh, "bar_merge_length_factor", p.bar_merge_length_factor);
    p.bar_merge_overlap       = readParam<double>(pnh, "bar_merge_overlap", p.bar_merge_overlap);
    p.max_armor_overlap_ratio = readParam<double>(pnh, "max_armor_overlap_ratio", p.max_armor_overlap_ratio);
    p.pair_centrality_weight = readParam<double>(pnh, "pair_centrality_weight", p.pair_centrality_weight);
    p.pair_centrality_y_band = readParam<double>(pnh, "pair_centrality_y_band", p.pair_centrality_y_band);
    p.reject_floor_reflection = readParam<bool>(pnh, "reject_floor_reflection", p.reject_floor_reflection);
    p.reflection_min_vertical_factor = readParam<double>(
        pnh, "reflection_min_vertical_factor", p.reflection_min_vertical_factor);

    p.max_inner_mean_v        = readParam<double>(pnh, "max_inner_mean_v", p.max_inner_mean_v);
    p.max_inner_colored_ratio = readParam<double>(pnh, "max_inner_colored_ratio", p.max_inner_colored_ratio);

    p.armor_small_width     = readParam<double>(pnh, "armor_small_width", p.armor_small_width);
    p.armor_large_width     = readParam<double>(pnh, "armor_large_width", p.armor_large_width);
    p.lightbar_length       = readParam<double>(pnh, "lightbar_length", p.lightbar_length);
    p.armor_width_tolerance = readParam<double>(pnh, "armor_width_tolerance", p.armor_width_tolerance);
    p.armor_width_upper_tolerance = readParam<double>(
        pnh, "armor_width_upper_tolerance", p.armor_width_upper_tolerance);
    p.armor_large_gap_lower_factor = readParam<double>(
        pnh, "armor_large_gap_lower_factor", p.armor_large_gap_lower_factor);
    p.armor_large_gap_upper_factor = readParam<double>(
        pnh, "armor_large_gap_upper_factor", p.armor_large_gap_upper_factor);
    p.max_large_box_aspect = readParam<double>(
        pnh, "max_large_box_aspect", p.max_large_box_aspect);
    p.wide_large_aspect_min = readParam<double>(
        pnh, "wide_large_aspect_min", p.wide_large_aspect_min);
    p.min_number_score_wide_large = readParam<double>(
        pnh, "min_number_score_wide_large", p.min_number_score_wide_large);
    p.min_number_score_large = readParam<double>(
        pnh, "min_number_score_large", p.min_number_score_large);

    p.max_reproj_error = readParam<double>(pnh, "max_reproj_error", p.max_reproj_error);
    p.min_depth        = readParam<double>(pnh, "min_depth", p.min_depth);
    p.max_depth        = readParam<double>(pnh, "max_depth", p.max_depth);

    p.number_tophat_kernel  = readParam<int>(pnh, "number_tophat_kernel", p.number_tophat_kernel);
    p.number_extend         = readParam<double>(pnh, "number_extend", p.number_extend);
    p.number_inset          = readParam<double>(pnh, "number_inset", p.number_inset);
    p.number_s_max          = readParam<int>(pnh, "number_s_max", p.number_s_max);
    p.number_thr_percentile = readParam<int>(pnh, "number_thr_percentile", p.number_thr_percentile);
    p.number_min_area_ratio = readParam<double>(pnh, "number_min_area_ratio", p.number_min_area_ratio);
    p.number_max_area_ratio = readParam<double>(pnh, "number_max_area_ratio", p.number_max_area_ratio);

    // Sanity fixes for values that must be odd / positive.
    if (p.queue_size <= 0) p.queue_size = 1;
    if (p.fallback_distortion.empty()) {
        p.fallback_distortion.assign(5, 0.0);
    }
    if (p.white_dilate < 1) p.white_dilate = 1;
    if (p.white_dilate % 2 == 0) ++p.white_dilate;
    if (p.number_tophat_kernel < 3) p.number_tophat_kernel = 3;
    if (p.number_tophat_kernel % 2 == 0) ++p.number_tophat_kernel;
    if (p.lightbar_length <= 0.0) p.lightbar_length = 55.0;
    if (p.armor_small_width <= 0.0) p.armor_small_width = 120.0;
    if (p.armor_large_width <= 0.0) p.armor_large_width = 235.0;
    if (p.armor_width_tolerance < 0.0) p.armor_width_tolerance = 0.0;
    if (p.armor_width_tolerance > 0.95) p.armor_width_tolerance = 0.95;

    return p;
}

// =============================================================================
// Data structures
// =============================================================================
struct LightBar {
    bool is_red = false;
    cv::Point2f center;
    cv::RotatedRect rect;
    float length = 0.0f;
    float width  = 0.0f;
    cv::Point2f axis;          // unit vector along the bar, pointing down
    std::vector<cv::Point2f> corners;
    int    area = 0;
    double p90_v = 0.0;
    double mean_v = 0.0;
    double white_ratio = 0.0;
    double sat_ratio = 0.0;
    double tilt = 0.0;         // deg from vertical
};

struct PairGeometry {
    cv::Point2f center;
    cv::Point2f P;             // unit vector from left bar to right bar
    cv::Point2f N;             // unit vector pointing down (N.y > 0)
    float mean_len = 0.0f;

    float left_inner = 0.0f;
    float right_inner = 0.0f;
    float top = 0.0f;
    float bottom = 0.0f;

    float top_left = 0.0f;
    float top_right = 0.0f;
    float bottom_left = 0.0f;
    float bottom_right = 0.0f;

    float inner_w = 0.0f;
    float inner_h = 0.0f;
    float top_diff = 0.0f;
    float bottom_diff = 0.0f;
    float pair_tilt = 0.0f;    // deg
};

struct PairCandidate {
    int left = -1;
    int right = -1;
    bool is_large = false;

    double gap = 0.0;
    double mean_len = 0.0;
    double gap_ratio = 0.0;
    double ratio_error = 0.0;

    double number_score = 0.0;
    double centrality = 0.5;
    double inner_mean_v = 0.0;
    double colored_ratio = 0.0;
    double score = 0.0;

    cv::Point2f center;
    PairGeometry geom;
};

struct Detection {
    int id = 0;
    bool is_red = false;
    bool is_large = false;
    std::array<cv::Point2f, 4> corners;  // TL, TR, BR, BL (image pixels)
    cv::Vec3d rvec{0, 0, 0};
    cv::Vec3d tvec{0, 0, 0};
    double reproj_error = 0.0;
    double score = 0.0;
    double number_score = 0.0;
    cv::Point2f center;
};

struct DetectResult {
    cv::Mat red_mask;
    cv::Mat blue_mask;
    std::vector<LightBar> bars;
    std::vector<PairCandidate> candidates;
    std::vector<PairCandidate> selected;
};

// =============================================================================
// Small helpers
// =============================================================================
inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double pointDistance(const cv::Point2f& a, const cv::Point2f& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}


double percentileFromHist(const std::vector<int>& hist, int total, double pct) {
    if (total <= 0) return 0.0;
    if (pct < 0.0) pct = 0.0;
    if (pct > 1.0) pct = 1.0;
    const int target = std::max(1, static_cast<int>(std::ceil(pct * total)));
    int acc = 0;
    for (int i = 0; i < 256; ++i) {
        acc += hist[i];
        if (acc >= target) return static_cast<double>(i);
    }
    return 255.0;
}

cv::Mat makeEllipseKernel(int size) {
    if (size < 1) size = 1;
    if (size % 2 == 0) ++size;
    return cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
}

// =============================================================================
// Colour mask + hollow-lightbar repair
// =============================================================================
cv::Mat buildColorMask(const cv::Mat& hsv, bool is_red, const Params& p,
                       const cv::Mat& kernel3, const cv::Mat& kernelWhite) {
    cv::Mat chroma;
    if (is_red) {
        cv::Mat r1, r2;
        cv::inRange(hsv, cv::Scalar(p.red_h_min1, p.red_s_min, p.red_v_min),
                    cv::Scalar(p.red_h_max1, p.red_s_max, p.red_v_max), r1);
        cv::inRange(hsv, cv::Scalar(p.red_h_min2, p.red_s_min, p.red_v_min),
                    cv::Scalar(p.red_h_max2, p.red_s_max, p.red_v_max), r2);
        cv::bitwise_or(r1, r2, chroma);
    } else {
        cv::inRange(hsv, cv::Scalar(p.blue_h_min, p.blue_s_min, p.blue_v_min),
                    cv::Scalar(p.blue_h_max, p.blue_s_max, p.blue_v_max), chroma);
    }

    // The LED centre clips to white; it is only re-added where it touches the
    // coloured halo of a bar.  This avoids turning white walls/numbers into
    // light bars.
    cv::Mat near;
    cv::dilate(chroma, near, kernelWhite);
    cv::Mat white;
    cv::inRange(hsv, cv::Scalar(0, 0, p.white_v_min),
                cv::Scalar(180, p.white_s_max, 255), white);
    cv::Mat core;
    cv::bitwise_and(white, near, core);

    cv::Mat solid;
    cv::bitwise_or(chroma, core, solid);
    cv::morphologyEx(solid, solid, cv::MORPH_CLOSE, kernel3);

    // Fill the hole: findContours(RETR_EXTERNAL)+drawContours(FILLED) turns a
    // ring into a solid bar without growing its outer boundary.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(solid.clone(), contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    cv::Mat filled = cv::Mat::zeros(solid.size(), CV_8U);
    if (!contours.empty()) {
        cv::drawContours(filled, contours, -1, cv::Scalar(255), cv::FILLED);
    }
    return filled;
}

// =============================================================================
// Light-bar extraction
// =============================================================================
void extractBars(const cv::Mat& hsv, const cv::Mat& mask, bool is_red,
                 const Params& p, std::vector<LightBar>& out) {
    if (mask.empty()) return;

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats,
                                                   centroids, 8, CV_32S);
    for (int i = 1; i < n; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < p.min_contour_area) continue;

        cv::Mat comp;
        cv::compare(labels, i, comp, cv::CMP_EQ);  // 255 where this label
        std::vector<cv::Point> points;
        cv::findNonZero(comp, points);
        if (points.size() < 5) continue;

        cv::RotatedRect rect = cv::minAreaRect(points);
        const float w = rect.size.width;
        const float h = rect.size.height;
        const float length = std::max(w, h);
        const float width  = std::min(w, h);
        if (length < p.min_bar_length || width < p.min_bar_width) continue;

        const double ratio = length / std::max(1.0e-3, static_cast<double>(width));
        if (ratio < p.min_bar_ratio || ratio > p.max_bar_ratio) continue;

        // Long-axis direction.  A fitted line is less sensitive to the
        // corner-ordering changes of RotatedRect than rect.angle.
        cv::Vec4f line;
        cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
        cv::Point2f axis(line[0], line[1]);
        const float norm = std::sqrt(axis.x * axis.x + axis.y * axis.y);
        if (norm < 1.0e-6f) continue;
        axis.x /= norm;
        axis.y /= norm;
        if (axis.y < 0.0f) {       // point "down" consistently
            axis.x = -axis.x;
            axis.y = -axis.y;
        }

        const double tilt = std::asin(std::min(1.0, std::fabs(static_cast<double>(axis.x)))) *
                            180.0 / kPi;
        if (tilt > p.max_bar_tilt) continue;

        // Photometry over the actual component (not the rectangle, which
        // would include too much dark background).
        std::vector<int> hist_v(256, 0);
        int white_like = 0;
        int saturated  = 0;
        long long sum_v = 0;
        for (const cv::Point& pt : points) {
            const cv::Vec3b px = hsv.at<cv::Vec3b>(pt.y, pt.x);
            const int H = px[0];
            const int S = px[1];
            const int V = px[2];
            (void)H;
            hist_v[V] += 1;
            sum_v += V;
            if (S < p.white_s_max && V > p.white_v_min) ++white_like;
            if (S > p.sat_s_min && V > p.sat_v_min) ++saturated;
        }
        const int count = static_cast<int>(points.size());
        const double p90_v = percentileFromHist(hist_v, count, 0.90);
        const double mean_v = static_cast<double>(sum_v) / std::max(1, count);
        const double white_ratio = static_cast<double>(white_like) / count;
        const double sat_ratio = static_cast<double>(saturated) / count;

        if (p90_v < p.min_bar_p90_v) continue;
        if (mean_v < p.min_bar_mean_v) continue;
        // A real LED has either the white-clipped core (red in these bags) or
        // an extremely saturated core (blue).  Diffuse floor glare has neither.
        if (!(white_ratio >= p.min_white_ratio || sat_ratio >= p.min_sat_ratio)) {
            continue;
        }

        LightBar bar;
        bar.is_red = is_red;
        bar.center = rect.center;
        bar.rect = rect;
        bar.length = length;
        bar.width = width;
        bar.axis = axis;
        bar.area = area;
        bar.p90_v = p90_v;
        bar.mean_v = mean_v;
        bar.white_ratio = white_ratio;
        bar.sat_ratio = sat_ratio;
        bar.tilt = tilt;

        cv::Point2f corners[4];
        rect.points(corners);
        bar.corners.assign(corners, corners + 4);
        out.push_back(bar);
    }
}

// =============================================================================
// Light-bar duplicate / fragment handling
// =============================================================================
bool samePhysicalBar(const LightBar& a, const LightBar& b, const Params& p) {
    if (a.is_red != b.is_red) return false;

    const double d = pointDistance(a.center, b.center);
    const double max_w = std::max(static_cast<double>(a.width),
                                  static_cast<double>(b.width));
    const double min_l = std::min(static_cast<double>(a.length),
                                  static_cast<double>(b.length));
    if (d < p.bar_merge_center_factor * max_w) return true;
    if (d < p.bar_merge_length_factor * min_l) return true;

    // Overlap coefficient (intersection / smaller area) is better than IoU
    // here: one fragment can be much smaller than the full bar.
    const cv::Rect ra = a.rect.boundingRect();
    const cv::Rect rb = b.rect.boundingRect();
    const cv::Rect inter = ra & rb;
    if (inter.width > 0 && inter.height > 0) {
        const double inter_area = static_cast<double>(inter.width) * inter.height;
        const double small_area = static_cast<double>(
            std::min(ra.width * ra.height, rb.width * rb.height));
        if (small_area > 0.0 && inter_area / small_area > p.bar_merge_overlap) {
            return true;
        }
    }
    return false;
}

std::vector<LightBar> mergeDuplicateBars(const std::vector<LightBar>& bars,
                                         const Params& p) {
    std::vector<bool> removed(bars.size(), false);
    auto better = [](const LightBar& a, const LightBar& b) {
        if (a.area != b.area) return a.area > b.area;
        if (std::fabs(a.length - b.length) > 1.0f) return a.length > b.length;
        return a.p90_v > b.p90_v;
    };

    for (size_t i = 0; i < bars.size(); ++i) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < bars.size(); ++j) {
            if (removed[j]) continue;
            if (!samePhysicalBar(bars[i], bars[j], p)) continue;
            if (better(bars[j], bars[i])) {
                removed[i] = true;
                break;  // bars[i] is the weaker fragment; move on
            }
            removed[j] = true;
        }
    }

    std::vector<LightBar> merged;
    merged.reserve(bars.size());
    for (size_t i = 0; i < bars.size(); ++i) {
        if (!removed[i]) merged.push_back(bars[i]);
    }
    return merged;
}

// =============================================================================
// Pair geometry
// =============================================================================
bool buildPairGeometry(const LightBar& left, const LightBar& right,
                       const Params& p, PairCandidate& pc) {
    const cv::Point2f delta = right.center - left.center;
    const double gap = std::sqrt(static_cast<double>(delta.x * delta.x + delta.y * delta.y));
    if (gap < 1.0) return false;
    if (gap > p.max_pair_distance) return false;

    const double mean_len = 0.5 * (left.length + right.length);
    if (mean_len < 1.0) return false;
    if (gap < p.min_pair_gap_factor * mean_len) return false;

    // Length / width consistency.
    if (std::min(left.length, right.length) /
            std::max(left.length, right.length) < (1.0 - p.max_length_ratio_diff)) {
        return false;
    }
    if (std::min(left.width, right.width) /
            std::max(left.width, right.width) < p.min_width_ratio) {
        return false;
    }

    // Bar axes must be parallel.
    const double dot = std::fabs(static_cast<double>(left.axis.x * right.axis.x +
                                                     left.axis.y * right.axis.y));
    const double angle_diff = std::acos(clampd(dot, -1.0, 1.0)) * 180.0 / kPi;
    if (angle_diff > p.max_pair_angle) return false;

    PairGeometry g;
    g.center = 0.5f * (left.center + right.center);
    g.mean_len = static_cast<float>(mean_len);

    g.P = cv::Point2f(delta.x / static_cast<float>(gap),
                      delta.y / static_cast<float>(gap));
    g.N = cv::Point2f(-g.P.y, g.P.x);
    if (g.N.y < 0.0f) {
        g.N.x = -g.N.x;
        g.N.y = -g.N.y;
    }

    auto projP = [&](const cv::Point2f& pt) {
        return pt.x * g.P.x + pt.y * g.P.y;
    };
    auto projN = [&](const cv::Point2f& pt) {
        return pt.x * g.N.x + pt.y * g.N.y;
    };

    float left_min_p = std::numeric_limits<float>::max();
    float left_max_p = -std::numeric_limits<float>::max();
    float right_min_p = std::numeric_limits<float>::max();
    float right_max_p = -std::numeric_limits<float>::max();
    float left_min_n = std::numeric_limits<float>::max();
    float left_max_n = -std::numeric_limits<float>::max();
    float right_min_n = std::numeric_limits<float>::max();
    float right_max_n = -std::numeric_limits<float>::max();

    for (const auto& c : left.corners) {
        const float up = projP(c);
        const float vn = projN(c);
        left_min_p = std::min(left_min_p, up);
        left_max_p = std::max(left_max_p, up);
        left_min_n = std::min(left_min_n, vn);
        left_max_n = std::max(left_max_n, vn);
    }
    for (const auto& c : right.corners) {
        const float up = projP(c);
        const float vn = projN(c);
        right_min_p = std::min(right_min_p, up);
        right_max_p = std::max(right_max_p, up);
        right_min_n = std::min(right_min_n, vn);
        right_max_n = std::max(right_max_n, vn);
    }

    g.left_inner  = left_max_p;
    g.right_inner = right_min_p;
    g.top_left = left_min_n;
    g.top_right = right_min_n;
    g.bottom_left = left_max_n;
    g.bottom_right = right_max_n;

    g.top = std::max(g.top_left, g.top_right);
    g.bottom = std::min(g.bottom_left, g.bottom_right);
    g.inner_w = g.right_inner - g.left_inner;
    g.inner_h = g.bottom - g.top;
    if (g.inner_w <= 2.0f || g.inner_h <= 2.0f) return false;

    g.top_diff = std::fabs(g.top_left - g.top_right);
    g.bottom_diff = std::fabs(g.bottom_left - g.bottom_right);

    // Do NOT allow an arbitrary vertical offset.  In red_4 the side plate and
    // the front plate have similar-looking bars; joining the right bar of one
    // plate to the left bar of the other produces a visibly tilted "armour"
    // with the two bars at clearly different heights.  All real front plates
    // in these bags have their two bars at almost the same image height.
    if (g.top_diff > p.max_endpoint_diff_ratio * g.mean_len ||
        g.bottom_diff > p.max_endpoint_diff_ratio * g.mean_len) {
        return false;
    }
    // Extra rigidity check for perspective: the top and bottom offsets must
    // also agree with each other.
    if (std::fabs(g.top_diff - g.bottom_diff) >
        p.max_endpoint_diff_ratio * g.mean_len) {
        return false;
    }

    const double center_dy_ratio =
        std::fabs(static_cast<double>(delta.y)) / mean_len;
    if (center_dy_ratio > p.max_pair_center_dy_ratio) return false;

    g.pair_tilt = std::atan2(std::fabs(static_cast<double>(delta.y)),
                             std::fabs(static_cast<double>(delta.x))) *
                  180.0 / kPi;
    if (g.pair_tilt > p.max_pair_tilt) return false;

    // Big / small classification is scale invariant: compare the gap between
    // bar centres with the bar length.
    const double small_exp = p.armor_small_width / p.lightbar_length;
    const double large_exp = p.armor_large_width / p.lightbar_length;
    const double gap_ratio = gap / mean_len;

    // The upper bound must be much tighter than the lower bound.  Projecting a
    // plate with yaw only makes the gap *smaller*; a gap much wider than W/L is
    // therefore not a real plate, it is two bars from different plates (the
    // blue_guard wide-box case, gap/bar-length ≈ 6.8).  Short/partially
    // detected bars can make the ratio slightly larger, but not 1.6x.
    const bool small_ok =
        gap_ratio >= small_exp * p.armor_width_tolerance &&
        gap_ratio <= small_exp * (1.0 + p.armor_width_upper_tolerance);
    // Large armour uses its own, tighter band.  In red_1&3, a side bar of the
    // left robot joined to the adjacent front bar gives ratio ≈3.2 (too small
    // for a large plate), while a left side bar joined to the right vehicle's
    // side bar gives ratio ≈5.0 (too large).  Both are rejected here, leaving
    // a gap between the small and large bands intentionally.
    const bool large_ok =
        gap_ratio >= large_exp * p.armor_large_gap_lower_factor &&
        gap_ratio <= large_exp * p.armor_large_gap_upper_factor;
    if (!small_ok && !large_ok) return false;

    const double err_small = std::fabs(gap_ratio - small_exp) / small_exp;
    const double err_large = std::fabs(gap_ratio - large_exp) / large_exp;
    bool is_large;
    if (small_ok && large_ok) {
        is_large = (err_large < err_small);
    } else {
        is_large = large_ok;
    }

    pc.left = -1;
    pc.right = -1;
    pc.is_large = is_large;
    pc.gap = gap;
    pc.mean_len = mean_len;
    pc.gap_ratio = gap_ratio;
    pc.ratio_error = std::min(err_small, err_large);
    pc.center = g.center;
    pc.geom = g;
    return true;
}

// =============================================================================
// Plate-interior evidence (number + brightness + colour leak)
// =============================================================================
void computeRegionEvidence(const cv::Mat& hsv, const cv::Mat& s_channel,
                           const cv::Mat& v_channel, const cv::Mat& top_hat,
                           const PairCandidate& pc, const Params& p,
                           double& number_score, double& inner_mean_v,
                           double& colored_ratio) {
    number_score = 0.0;
    inner_mean_v = 0.0;
    colored_ratio = 0.0;

    const PairGeometry& g = pc.geom;
    // left_inner/right_inner/top/bottom are absolute projections on P/N.
    // Convert them to a frame centred on the pair before rebuilding points.
    const float pair_center_u = g.center.dot(g.P);
    const float pair_center_v = g.center.dot(g.N);
    const float u0_raw = g.left_inner - pair_center_u;
    const float u1_raw = g.right_inner - pair_center_u;
    const float du = static_cast<float>(p.number_inset) * (u1_raw - u0_raw);
    const float u0 = u0_raw + du;
    const float u1 = u1_raw - du;
    const float v0 = (g.top - pair_center_v) -
                     static_cast<float>(p.number_extend) * g.mean_len;
    const float v1 = (g.bottom - pair_center_v) +
                     static_cast<float>(p.number_extend) * g.mean_len;
    if (u1 <= u0 || v1 <= v0) return;

    const cv::Point2f q[4] = {
        g.center + g.P * u0 + g.N * v0,
        g.center + g.P * u1 + g.N * v0,
        g.center + g.P * u1 + g.N * v1,
        g.center + g.P * u0 + g.N * v1
    };

    cv::Rect roi = cv::boundingRect(std::vector<cv::Point2f>(q, q + 4));
    roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
    if (roi.width < 3 || roi.height < 3) return;
    cv::Mat local_mask = cv::Mat::zeros(roi.size(), CV_8U);
    std::vector<cv::Point> q_local;
    q_local.reserve(4);
    for (int k = 0; k < 4; ++k) {
        q_local.emplace_back(cv::Point(cvRound(q[k].x) - roi.x,
                                       cvRound(q[k].y) - roi.y));
    }
    cv::fillConvexPoly(local_mask, q_local, cv::Scalar(255));

    const int region_w = std::max(1, static_cast<int>(std::lround(u1 - u0)));
    const int region_h = std::max(1, static_cast<int>(std::lround(v1 - v0)));

    const int total_pixels = cv::countNonZero(local_mask);
    if (total_pixels < 20) return;
    std::vector<int> hist_top(256, 0);
    long long sum_v = 0;
    int colored = 0;
    for (int y = 0; y < roi.height; ++y) {
        const uchar* mrow = local_mask.ptr<uchar>(y);
        for (int x = 0; x < roi.width; ++x) {
            if (!mrow[x]) continue;
            const int gx = roi.x + x;
            const int gy = roi.y + y;
            const int V = v_channel.at<uchar>(gy, gx);
            const int S = s_channel.at<uchar>(gy, gx);
            sum_v += V;
            hist_top[top_hat.at<uchar>(gy, gx)] += 1;
            if (S > p.sat_s_min && V > p.sat_v_min) ++colored;
        }
    }
    inner_mean_v = static_cast<double>(sum_v) / total_pixels;
    colored_ratio = static_cast<double>(colored) / total_pixels;

    const double thr = std::max(
        8.0, percentileFromHist(hist_top, total_pixels,
                                static_cast<double>(p.number_thr_percentile) / 100.0));
    const double median_top = percentileFromHist(hist_top, total_pixels, 0.5);
    // Candidate = bright achromatic structure inside the plate, only inside
    // the filled polygon.  The number is compact and does not span the full
    // plate; a floor line or a side panel usually does.
    cv::Mat top_roi = top_hat(roi);
    cv::Mat s_roi = s_channel(roi);
    cv::Mat cand, low_s;
    // Use threshold (not compare with a double scalar) so the 8U top-hat
    // image is compared against an unambiguous saturation value.
    cv::threshold(top_roi, cand, thr, 255, cv::THRESH_BINARY);
    cv::threshold(s_roi, low_s, p.number_s_max, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(cand, low_s, cand);
    cv::bitwise_and(cand, local_mask, cand);

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(cand, labels, stats,
                                                   centroids, 8, CV_32S);
    for (int i = 1; i < n; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        const double area_ratio = static_cast<double>(area) / total_pixels;
        if (area_ratio < p.number_min_area_ratio ||
            area_ratio > p.number_max_area_ratio) {
            continue;
        }
        const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (w > 0.95 * region_w || h > 0.90 * region_h) continue;

        // p90 of the top-hat response inside the component.
        cv::Mat comp_mask;
        cv::compare(labels, i, comp_mask, cv::CMP_EQ);
        std::vector<cv::Point> comp_points;
        cv::findNonZero(comp_mask, comp_points);
        if (comp_points.size() < 5) continue;
        std::vector<int> comp_hist(256, 0);
        for (const auto& cp : comp_points) {
            comp_hist[top_roi.at<uchar>(cp)] += 1;
        }
        const double p90_comp =
            percentileFromHist(comp_hist, static_cast<int>(comp_points.size()), 0.90);
        const double contrast = p90_comp - median_top;
        if (contrast < 5.0) continue;

        const cv::Point2f centroid(static_cast<float>(centroids.at<double>(i, 0) + roi.x),
                                   static_cast<float>(centroids.at<double>(i, 1) + roi.y));
        const float u = (centroid - g.center).dot(g.P);
        const float v = (centroid - g.center).dot(g.N);
        const float region_center_u = (u0 + u1) * 0.5f;
        const float region_center_v = (v0 + v1) * 0.5f;
        const float cu = 1.0f - std::fabs(u - region_center_u) /
                                    std::max(1.0f, (u1 - u0) * 0.5f);
        const float cv = 1.0f - std::fabs(v - region_center_v) /
                                    std::max(1.0f, (v1 - v0) * 0.5f);
        const float center_score = clampf(std::min(cu, cv), 0.0f, 1.0f);

        const double score = std::min(1.0, area_ratio / 0.03) *
                             std::min(1.0, contrast / 30.0) *
                             (0.15 + 0.85 * center_score);
        number_score = std::max(number_score, score);
    }
}

// =============================================================================
// Pair search / scoring / NMS
// =============================================================================
double pairCentrality(const PairCandidate& pc, const std::vector<LightBar>& bars,
                     const Params& p) {
    const LightBar& l = bars[pc.left];
    const LightBar& r = bars[pc.right];
    const double y_center = 0.5 * (static_cast<double>(l.center.y) + r.center.y);
    const double band = p.pair_centrality_y_band * pc.mean_len;

    bool has_left = false;
    bool has_right = false;
    double nearest_left = -1.0e9;
    double nearest_right = 1.0e9;
    for (size_t k = 0; k < bars.size(); ++k) {
        if (static_cast<int>(k) == pc.left || static_cast<int>(k) == pc.right) continue;
        const LightBar& b = bars[k];
        if (b.is_red != l.is_red) continue;
        if (std::fabs(static_cast<double>(b.center.y) - y_center) > band) continue;
        if (b.center.x < l.center.x) {
            if (static_cast<double>(b.center.x) > nearest_left) {
                nearest_left = b.center.x;
                has_left = true;
            }
        } else if (b.center.x > r.center.x) {
            if (static_cast<double>(b.center.x) < nearest_right) {
                nearest_right = b.center.x;
                has_right = true;
            }
        }
    }

    // A plate between two neighbouring plates is the central pair: the free
    // gap on its left and right is almost the same.  This prefers the true
    // front plate over a pair made from one front bar and one side bar.
    if (has_left && has_right) {
        const double gl = static_cast<double>(l.center.x) - nearest_left;
        const double gr = nearest_right - static_cast<double>(r.center.x);
        if (gl + gr > 1.0) {
            return clampd(1.0 - std::fabs(gl - gr) / (gl + gr), 0.0, 1.0);
        }
        return 0.5;
    }
    if (has_left || has_right) return 0.25;
    return 0.5;
}

std::vector<PairCandidate> findPairCandidates(
        const cv::Mat& hsv, const cv::Mat& s_channel, const cv::Mat& v_channel,
        const cv::Mat& top_hat, const std::vector<LightBar>& bars,
        const Params& p) {
    std::vector<PairCandidate> out;
    const size_t n = bars.size();
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (bars[i].is_red != bars[j].is_red) continue;

            const LightBar* left = &bars[i];
            const LightBar* right = &bars[j];
            int left_idx = static_cast<int>(i);
            int right_idx = static_cast<int>(j);
            if (left->center.x > right->center.x) {
                std::swap(left, right);
                std::swap(left_idx, right_idx);
            }

            PairCandidate pc;
            if (!buildPairGeometry(*left, *right, p, pc)) continue;
            pc.left = left_idx;
            pc.right = right_idx;

            // Reject a pair whose interval contains a third bar of the same
            // colour: it would be a cross-pair between two different plates.
            bool third_inside = false;
            const float line_v = pc.geom.center.dot(pc.geom.N);
            for (size_t k = 0; k < n; ++k) {
                if (static_cast<int>(k) == left_idx || static_cast<int>(k) == right_idx) continue;
                if (bars[k].is_red != left->is_red) continue;
                const float u = bars[k].center.dot(pc.geom.P);
                const float v = bars[k].center.dot(pc.geom.N);
                if (u > pc.geom.left_inner && u < pc.geom.right_inner &&
                    std::fabs(v - line_v) < 0.55f * pc.geom.mean_len) {
                    third_inside = true;
                    break;
                }
            }
            if (third_inside) continue;

            double number_score = 0.0;
            double inner_mean = 0.0;
            double colored_ratio = 0.0;
            computeRegionEvidence(hsv, s_channel, v_channel, top_hat, pc, p,
                                  number_score, inner_mean, colored_ratio);

            if (inner_mean > p.max_inner_mean_v) continue;
            if (colored_ratio > p.max_inner_colored_ratio) continue;

            if (pc.is_large) {
                // A real large plate always has a big number; a cross pair
                // between two plates/vehicles usually has empty background in
                // its interior and a near-zero number score.  Keep the gate
                // low enough for a dark but visible number, high enough for
                // empty background.
                if (number_score < p.min_number_score_large) continue;
                // Use the actual projected quadrilateral aspect, not only the
                // bar-centre distance: a pair made from the two facing side
                // bars of two different vehicles has aspect ≈4.8-5.0 while the
                // real large plate is ≈4.5.  A large plate with this width must
                // also contain the big armour number; otherwise it is two
                // unrelated bars with empty background between them.
                const cv::Point2f lt = left->center - left->axis * (left->length * 0.5f);
                const cv::Point2f lb = left->center + left->axis * (left->length * 0.5f);
                const cv::Point2f rt = right->center - right->axis * (right->length * 0.5f);
                const cv::Point2f rb = right->center + right->axis * (right->length * 0.5f);
                const double edge_w = 0.5 * (pointDistance(lt, rt) + pointDistance(lb, rb));
                const double edge_h = 0.5 * (pointDistance(lt, lb) + pointDistance(rt, rb));
                const double box_aspect = edge_w / std::max(1.0, edge_h);
                if (box_aspect > p.max_large_box_aspect) continue;
                if (box_aspect > p.wide_large_aspect_min &&
                    number_score < p.min_number_score_wide_large) {
                    continue;
                }
            }

            pc.number_score = number_score;
            pc.inner_mean_v = inner_mean;
            pc.colored_ratio = colored_ratio;
            pc.centrality = pairCentrality(pc, bars, p);

            const double endpoint_mismatch =
                static_cast<double>(std::max(pc.geom.top_diff, pc.geom.bottom_diff));
            const double sym = 1.0 - std::min(
                1.0, endpoint_mismatch /
                         std::max(1.0, 0.35 * static_cast<double>(pc.geom.mean_len)));
            const double barq = 0.5 * (left->p90_v + right->p90_v) / 255.0;
            const double dark = std::max(0.0, 1.0 - inner_mean / 220.0);

            pc.score = 1.2 * (1.0 - std::min(1.0, pc.ratio_error)) +
                       0.5 * sym +
                       0.4 * barq +
                       1.5 * pc.number_score +
                       p.pair_centrality_weight * pc.centrality +
                       0.1 * dark;
            out.push_back(pc);
        }
    }
    return out;
}

double armorBoxOverlap(const PairCandidate& a, const PairCandidate& b,
                       const std::vector<LightBar>& bars) {
    if (a.left < 0 || a.right < 0 || b.left < 0 || b.right < 0) return 0.0;
    auto box_of = [&](const PairCandidate& pc, cv::Rect2f& box) {
        const LightBar& l = bars[pc.left];
        const LightBar& r = bars[pc.right];
        const cv::Point2f pts[4] = {
            l.center - l.axis * (l.length * 0.5f),
            l.center + l.axis * (l.length * 0.5f),
            r.center - r.axis * (r.length * 0.5f),
            r.center + r.axis * (r.length * 0.5f)
        };
        float x0 = pts[0].x, x1 = pts[0].x, y0 = pts[0].y, y1 = pts[0].y;
        for (int k = 1; k < 4; ++k) {
            x0 = std::min(x0, pts[k].x); x1 = std::max(x1, pts[k].x);
            y0 = std::min(y0, pts[k].y); y1 = std::max(y1, pts[k].y);
        }
        const float pad = 0.5f * std::max(l.width, r.width);
        box = cv::Rect2f(x0 - pad, y0 - pad,
                         (x1 - x0) + 2.0f * pad, (y1 - y0) + 2.0f * pad);
    };

    cv::Rect2f ra, rb;
    box_of(a, ra);
    box_of(b, rb);
    const float ix0 = std::max(ra.x, rb.x);
    const float iy0 = std::max(ra.y, rb.y);
    const float ix1 = std::min(ra.x + ra.width, rb.x + rb.width);
    const float iy1 = std::min(ra.y + ra.height, rb.y + rb.height);
    if (ix1 <= ix0 || iy1 <= iy0) return 0.0;
    const double inter = static_cast<double>(ix1 - ix0) * (iy1 - iy0);
    const double small = std::min(static_cast<double>(ra.width) * ra.height,
                                  static_cast<double>(rb.width) * rb.height);
    return small > 0.0 ? inter / small : 0.0;
}

bool isLikelyFloorReflection(const PairCandidate& below,
                             const PairCandidate& above,
                             const std::vector<LightBar>& bars,
                             const Params& p) {
    if (below.left < 0 || below.right < 0 || above.left < 0 || above.right < 0) {
        return false;
    }
    if (bars[below.left].is_red != bars[above.left].is_red) return false;
    if (below.center.y <=
        above.center.y + p.reflection_min_vertical_factor * above.mean_len) {
        return false;
    }
    // The reflected plate has almost the same horizontal position and width as
    // the real plate above it, but is much lower in the image (on the floor).
    if (std::fabs(below.center.x - above.center.x) >
        0.70 * std::max(1.0, static_cast<double>(above.geom.inner_w))) {
        return false;
    }
    const double width_ratio = below.geom.inner_w / std::max(1.0f, above.geom.inner_w);
    if (width_ratio < 0.55 || width_ratio > 1.80) return false;
    const double len_ratio = below.mean_len / std::max(1.0, above.mean_len);
    if (len_ratio < 0.45 || len_ratio > 2.20) return false;
    return true;
}

std::vector<PairCandidate> selectPairs(const std::vector<PairCandidate>& candidates,
                                       const std::vector<LightBar>& bars,
                                       const Params& p) {
    std::vector<int> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return candidates[a].score > candidates[b].score;
    });

    std::vector<bool> used(bars.size(), false);
    std::vector<PairCandidate> chosen;
    for (int idx : order) {
        const PairCandidate& pc = candidates[idx];
        if (pc.left < 0 || pc.right < 0) continue;
        if (used[pc.left] || used[pc.right]) continue;

        // Even after fragment merging, never let one physical bar be used by
        // two armours.  This is what caused the "flash" box connecting two
        // adjacent plates in red_4.
        bool shares_bar = false;
        for (size_t u = 0; u < bars.size() && !shares_bar; ++u) {
            if (!used[u]) continue;
            if (samePhysicalBar(bars[pc.left], bars[u], p) ||
                samePhysicalBar(bars[pc.right], bars[u], p)) {
                shares_bar = true;
            }
        }
        if (shares_bar) continue;

        bool too_close = false;
        for (const auto& c : chosen) {
            const float d = static_cast<float>(pointDistance(pc.center, c.center));
            if (d < 0.30f * std::min(pc.geom.mean_len, c.geom.mean_len)) {
                too_close = true;
                break;
            }
        }
        if (too_close) continue;

        // Two detections must not cover the same image region.  A cross-pair
        // involving a neighbouring plate often overlaps the true armour.
        bool overlaps = false;
        for (const auto& c : chosen) {
            if (armorBoxOverlap(pc, c, bars) > p.max_armor_overlap_ratio) {
                overlaps = true;
                break;
            }
        }
        if (overlaps) continue;

        used[pc.left] = true;
        used[pc.right] = true;
        chosen.push_back(pc);
    }

    // A floor/wax reflection appears below the real plate with nearly the same
    // x position and width.  Remove that lower duplicate; it is the remaining
    // "flash" source in red_4 after the pairing fixes.
    if (p.reject_floor_reflection && chosen.size() > 1) {
        std::vector<bool> drop(chosen.size(), false);
        for (size_t i = 0; i < chosen.size(); ++i) {
            for (size_t j = 0; j < chosen.size(); ++j) {
                if (i == j) continue;
                if (isLikelyFloorReflection(chosen[i], chosen[j], bars, p)) {
                    drop[i] = true;
                    break;
                }
            }
        }
        std::vector<PairCandidate> filtered;
        filtered.reserve(chosen.size());
        for (size_t i = 0; i < chosen.size(); ++i) {
            if (!drop[i]) filtered.push_back(chosen[i]);
        }
        chosen.swap(filtered);
    }

    // Stable per-frame order: left to right.
    std::sort(chosen.begin(), chosen.end(),
              [](const PairCandidate& a, const PairCandidate& b) {
                  return a.center.x < b.center.x;
              });
    return chosen;
}

// =============================================================================
// PnP
// =============================================================================
bool solveArmorPose(const PairCandidate& pc, const std::vector<LightBar>& bars,
                    const cv::Mat& K, const cv::Mat& D, const Params& p,
                    Detection& det) {
    if (pc.left < 0 || pc.right < 0) return false;
    const LightBar& left = bars[pc.left];
    const LightBar& right = bars[pc.right];

    const double width_m = (pc.is_large ? p.armor_large_width : p.armor_small_width) / 1000.0;
    const double length_m = p.lightbar_length / 1000.0;
    const double half_w = width_m * 0.5;
    const double half_l = length_m * 0.5;

    // Definition 1: four endpoints of the two light bars.
    // Object frame: x right, y down, z into the plate (right-handed, camera
    // optical convention x-right / y-down / z-forward).
    std::vector<cv::Point3f> object_points = {
        cv::Point3f(static_cast<float>(-half_w), static_cast<float>(-half_l), 0.0f),  // TL
        cv::Point3f(static_cast<float>( half_w), static_cast<float>(-half_l), 0.0f),  // TR
        cv::Point3f(static_cast<float>( half_w), static_cast<float>( half_l), 0.0f),  // BR
        cv::Point3f(static_cast<float>(-half_w), static_cast<float>( half_l), 0.0f)   // BL
    };

    const cv::Point2f left_top    = left.center  - left.axis  * (left.length * 0.5f);
    const cv::Point2f left_bottom = left.center  + left.axis  * (left.length * 0.5f);
    const cv::Point2f right_top   = right.center - right.axis * (right.length * 0.5f);
    const cv::Point2f right_bottom= right.center + right.axis * (right.length * 0.5f);

    std::vector<cv::Point2f> image_points = {
        left_top, right_top, right_bottom, left_bottom
    };

    auto evaluate = [&](const cv::Mat& rvec, const cv::Mat& tvec, double& rmse) -> bool {
        if (rvec.empty() || tvec.empty()) return false;
        const double z = tvec.at<double>(2);
        if (!std::isfinite(z) || z < p.min_depth || z > p.max_depth) return false;
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points, rvec, tvec, K, D, projected);
        if (projected.size() != image_points.size()) return false;
        double sum = 0.0;
        for (size_t i = 0; i < projected.size(); ++i) {
            const cv::Point2f d = projected[i] - image_points[i];
            sum += d.x * d.x + d.y * d.y;
        }
        rmse = std::sqrt(sum / static_cast<double>(projected.size()));
        return std::isfinite(rmse);
    };

    cv::Mat best_rvec, best_tvec;
    double best_rmse = std::numeric_limits<double>::max();

    cv::Mat rvec, tvec;
    try {
        if (cv::solvePnP(object_points, image_points, K, D, rvec, tvec, false,
                         cv::SOLVEPNP_IPPE)) {
            double rmse = 0.0;
            if (evaluate(rvec, tvec, rmse) && rmse < best_rmse) {
                best_rmse = rmse;
                best_rvec = rvec.clone();
                best_tvec = tvec.clone();
            }
        }
    } catch (const cv::Exception&) {
        // Fall through to the iterative solver.
    }

    try {
        if (cv::solvePnP(object_points, image_points, K, D, rvec, tvec, false,
                         cv::SOLVEPNP_ITERATIVE)) {
            double rmse = 0.0;
            if (evaluate(rvec, tvec, rmse) && rmse < best_rmse) {
                best_rmse = rmse;
                best_rvec = rvec.clone();
                best_tvec = tvec.clone();
            }
        }
    } catch (const cv::Exception&) {
        // Ignore; the other solution may already be valid.
    }

    if (best_rvec.empty() || best_tvec.empty() || best_rmse > p.max_reproj_error) {
        return false;
    }
    for (int k = 0; k < 3; ++k) {
        if (!std::isfinite(best_rvec.at<double>(k)) ||
            !std::isfinite(best_tvec.at<double>(k))) {
            return false;
        }
    }

    det.is_red = left.is_red;
    det.is_large = pc.is_large;
    det.center = pc.center;
    det.corners = {left_top, right_top, right_bottom, left_bottom};
    det.rvec = cv::Vec3d(best_rvec.at<double>(0),
                         best_rvec.at<double>(1),
                         best_rvec.at<double>(2));
    det.tvec = cv::Vec3d(best_tvec.at<double>(0),
                         best_tvec.at<double>(1),
                         best_tvec.at<double>(2));
    det.reproj_error = best_rmse;
    det.score = pc.score;
    det.number_score = pc.number_score;
    return true;
}

geometry_msgs::Quaternion rotationToQuaternion(const cv::Vec3d& rvec) {
    cv::Mat R;
    const cv::Mat rvec_mat = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);
    cv::Rodrigues(rvec_mat, R);
    tf2::Matrix3x3 m(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                     R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                     R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    tf2::Quaternion q;
    m.getRotation(q);
    q.normalize();
    geometry_msgs::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
}

// =============================================================================
// Detection entry point
// =============================================================================
DetectResult detectFrame(const cv::Mat& bgr, const Params& p) {
    DetectResult result;
    if (bgr.empty()) return result;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);  // 0=H, 1=S, 2=V

    const cv::Mat kernel3 = makeEllipseKernel(3);
    const cv::Mat kernel_white = makeEllipseKernel(p.white_dilate);

    result.red_mask  = buildColorMask(hsv, true,  p, kernel3, kernel_white);
    result.blue_mask = buildColorMask(hsv, false, p, kernel3, kernel_white);

    extractBars(hsv, result.red_mask,  true,  p, result.bars);
    extractBars(hsv, result.blue_mask, false, p, result.bars);


    // One top-hat image serves all pair candidates.
    cv::Mat top_hat;
    cv::morphologyEx(hsv_channels[2], top_hat, cv::MORPH_TOPHAT,
                     cv::getStructuringElement(
                         cv::MORPH_RECT,
                         cv::Size(p.number_tophat_kernel, p.number_tophat_kernel)));

    result.candidates = findPairCandidates(hsv, hsv_channels[1], hsv_channels[2],
                                           top_hat, result.bars, p);
    result.selected = selectPairs(result.candidates, result.bars, p);
    return result;
}

// =============================================================================
// ROS node
// =============================================================================
class ImageReceiverNode {
public:
    explicit ImageReceiverNode(ros::NodeHandle& nh)
        : nh_(nh), pnh_("~"), it_(nh_) {
        params_ = loadParams(pnh_);

        // Fallback camera from YAML.  It is overwritten by camera_info.
        camera_K_ = cv::Mat::eye(3, 3, CV_64F);
        camera_K_.at<double>(0, 0) = params_.fallback_fx;
        camera_K_.at<double>(1, 1) = params_.fallback_fy;
        camera_K_.at<double>(0, 2) = params_.fallback_cx;
        camera_K_.at<double>(1, 2) = params_.fallback_cy;
        camera_D_ = cv::Mat(1, static_cast<int>(params_.fallback_distortion.size()), CV_64F);
        for (size_t i = 0; i < params_.fallback_distortion.size(); ++i) {
            camera_D_.at<double>(0, static_cast<int>(i)) = params_.fallback_distortion[i];
        }
        camera_frame_ = params_.camera_frame_fallback;

        detection_pub_ = nh_.advertise<ft_detection::DetectionFrame>(
            params_.detection_topic, 10);
        debug_image_pub_ = nh_.advertise<sensor_msgs::Image>(
            params_.debug_image_topic, 1);
        debug_mask_pub_ = nh_.advertise<sensor_msgs::Image>(
            params_.debug_mask_topic, 1);

        tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

        camera_info_sub_ = nh_.subscribe(
            params_.camera_info_topic, 2,
            &ImageReceiverNode::cameraInfoCallback, this);

        image_transport::TransportHints hints(
            params_.image_transport, ros::TransportHints(), pnh_);
        image_sub_ = it_.subscribe(
            params_.image_topic, params_.queue_size,
            &ImageReceiverNode::imageCallback, this, hints);

        if (params_.use_dynamic_reconfigure) {
            dyn_server_.reset(
                new dynamic_reconfigure::Server<ft_detector::ArmorReconfigureConfig>(
                    dyn_mutex_, pnh_));
            dynamic_reconfigure::Server<ft_detector::ArmorReconfigureConfig>::CallbackType cb =
                boost::bind(&ImageReceiverNode::dynamicCallback, this, _1, _2);
            dyn_server_->setCallback(cb);
            syncDynamicConfig();
        }

        ROS_INFO_STREAM("armor detector started"
                        << " | image=" << params_.image_topic
                        << " (" << params_.image_transport << ")"
                        << " | camera_info=" << params_.camera_info_topic
                        << " | detection=" << params_.detection_topic
                        << " | dynamic_reconfigure="
                        << (params_.use_dynamic_reconfigure ? "on" : "off"));
    }

private:
    // ---------------------------------------------------------------------
    // Dynamic reconfigure
    // ---------------------------------------------------------------------
    void dynamicCallback(ft_detector::ArmorReconfigureConfig& cfg, uint32_t /*level*/) {
        std::lock_guard<std::mutex> lock(params_mutex_);
        params_.red_h_min1 = cfg.red_h_min1;
        params_.red_h_max1 = cfg.red_h_max1;
        params_.red_h_min2 = cfg.red_h_min2;
        params_.red_h_max2 = cfg.red_h_max2;
        params_.red_s_min  = cfg.red_s_min;
        params_.red_s_max  = cfg.red_s_max;
        params_.red_v_min  = cfg.red_v_min;
        params_.red_v_max  = cfg.red_v_max;

        params_.blue_h_min = cfg.blue_h_min;
        params_.blue_h_max = cfg.blue_h_max;
        params_.blue_s_min = cfg.blue_s_min;
        params_.blue_s_max = cfg.blue_s_max;
        params_.blue_v_min = cfg.blue_v_min;
        params_.blue_v_max = cfg.blue_v_max;

        params_.white_s_max = cfg.white_s_max;
        params_.white_v_min = cfg.white_v_min;

        params_.min_contour_area = cfg.min_contour_area;
        params_.min_bar_length   = cfg.min_bar_length;
        params_.min_bar_ratio    = cfg.min_bar_ratio;
        params_.max_bar_ratio    = cfg.max_bar_ratio;
        params_.max_bar_tilt     = cfg.max_bar_tilt;
        params_.min_bar_p90_v    = cfg.min_bar_p90_v;
        params_.min_white_ratio  = cfg.min_white_ratio;
        params_.min_sat_ratio    = cfg.min_sat_ratio;

        params_.max_pair_distance       = cfg.max_pair_distance;
        params_.max_pair_angle          = cfg.max_pair_angle;
        params_.max_pair_tilt           = cfg.max_pair_tilt;
        params_.max_pair_center_dy_ratio = cfg.max_pair_center_dy_ratio;
        params_.max_endpoint_diff_ratio = cfg.max_endpoint_diff_ratio;
        params_.max_length_ratio_diff   = cfg.max_length_ratio_diff;
        params_.min_width_ratio         = cfg.min_width_ratio;
        params_.armor_width_tolerance   = cfg.armor_width_tolerance;
        params_.armor_width_upper_tolerance = cfg.armor_width_upper_tolerance;
        params_.armor_large_gap_lower_factor = cfg.armor_large_gap_lower_factor;
        params_.armor_large_gap_upper_factor = cfg.armor_large_gap_upper_factor;
        params_.max_large_box_aspect = cfg.max_large_box_aspect;
        params_.wide_large_aspect_min = cfg.wide_large_aspect_min;
        params_.min_number_score_wide_large = cfg.min_number_score_wide_large;
        params_.min_number_score_large = cfg.min_number_score_large;

        params_.max_inner_mean_v        = cfg.max_inner_mean_v;
        params_.max_inner_colored_ratio = cfg.max_inner_colored_ratio;

        params_.max_reproj_error = cfg.max_reproj_error;
        params_.min_depth        = cfg.min_depth;
        params_.max_depth        = cfg.max_depth;

        params_.publish_mask = cfg.publish_mask;
        params_.publish_tf   = cfg.publish_tf;
    }

    void syncDynamicConfig() {
        if (!dyn_server_) return;
        ft_detector::ArmorReconfigureConfig cfg;
        {
            std::lock_guard<std::mutex> lock(params_mutex_);
            cfg.red_h_min1 = params_.red_h_min1;
            cfg.red_h_max1 = params_.red_h_max1;
            cfg.red_h_min2 = params_.red_h_min2;
            cfg.red_h_max2 = params_.red_h_max2;
            cfg.red_s_min  = params_.red_s_min;
            cfg.red_s_max  = params_.red_s_max;
            cfg.red_v_min  = params_.red_v_min;
            cfg.red_v_max  = params_.red_v_max;

            cfg.blue_h_min = params_.blue_h_min;
            cfg.blue_h_max = params_.blue_h_max;
            cfg.blue_s_min = params_.blue_s_min;
            cfg.blue_s_max = params_.blue_s_max;
            cfg.blue_v_min = params_.blue_v_min;
            cfg.blue_v_max = params_.blue_v_max;

            cfg.white_s_max = params_.white_s_max;
            cfg.white_v_min = params_.white_v_min;

            cfg.min_contour_area = params_.min_contour_area;
            cfg.min_bar_length   = params_.min_bar_length;
            cfg.min_bar_ratio    = params_.min_bar_ratio;
            cfg.max_bar_ratio    = params_.max_bar_ratio;
            cfg.max_bar_tilt     = params_.max_bar_tilt;
            cfg.min_bar_p90_v    = params_.min_bar_p90_v;
            cfg.min_white_ratio  = params_.min_white_ratio;
            cfg.min_sat_ratio    = params_.min_sat_ratio;

            cfg.max_pair_distance       = params_.max_pair_distance;
            cfg.max_pair_angle          = params_.max_pair_angle;
            cfg.max_pair_tilt           = params_.max_pair_tilt;
            cfg.max_pair_center_dy_ratio = params_.max_pair_center_dy_ratio;
            cfg.max_endpoint_diff_ratio = params_.max_endpoint_diff_ratio;
            cfg.max_length_ratio_diff   = params_.max_length_ratio_diff;
            cfg.min_width_ratio         = params_.min_width_ratio;
            cfg.armor_width_tolerance   = params_.armor_width_tolerance;
            cfg.armor_width_upper_tolerance = params_.armor_width_upper_tolerance;
            cfg.armor_large_gap_lower_factor = params_.armor_large_gap_lower_factor;
            cfg.armor_large_gap_upper_factor = params_.armor_large_gap_upper_factor;
            cfg.max_large_box_aspect = params_.max_large_box_aspect;
            cfg.wide_large_aspect_min = params_.wide_large_aspect_min;
            cfg.min_number_score_wide_large = params_.min_number_score_wide_large;
            cfg.min_number_score_large = params_.min_number_score_large;

            cfg.max_inner_mean_v        = params_.max_inner_mean_v;
            cfg.max_inner_colored_ratio = params_.max_inner_colored_ratio;

            cfg.max_reproj_error = params_.max_reproj_error;
            cfg.min_depth        = params_.min_depth;
            cfg.max_depth        = params_.max_depth;

            cfg.publish_mask = params_.publish_mask;
            cfg.publish_tf   = params_.publish_tf;
        }
        dyn_server_->updateConfig(cfg);
    }

    // ---------------------------------------------------------------------
    // Callbacks
    // ---------------------------------------------------------------------
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (msg->K[0] == 0.0 && msg->K[4] == 0.0) return;

        std::lock_guard<std::mutex> lock(camera_mutex_);
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                K.at<double>(r, c) = msg->K[static_cast<size_t>(r * 3 + c)];
            }
        }
        cv::Mat D;
        if (!msg->D.empty()) {
            D = cv::Mat(1, static_cast<int>(msg->D.size()), CV_64F);
            for (size_t i = 0; i < msg->D.size(); ++i) {
                D.at<double>(0, static_cast<int>(i)) = msg->D[i];
            }
        } else {
            D = cv::Mat::zeros(1, 5, CV_64F);
        }
        camera_K_ = K;
        camera_D_ = D;
        camera_width_ = msg->width;
        camera_height_ = msg->height;
        if (!msg->header.frame_id.empty()) {
            camera_frame_ = msg->header.frame_id;
        }
        has_camera_info_ = true;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR_STREAM_THROTTLE(2.0, "cv_bridge exception: " << e.what());
            return;
        }
        if (!cv_ptr || cv_ptr->image.empty()) return;

        cv::Mat img = cv_ptr->image;
        if (img.channels() == 1) {
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        } else if (img.channels() == 4) {
            cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
        }

        // Copy the parameter snapshot.  Dynamic reconfigure may run at any
        // time; a frame must use one consistent set.
        Params p;
        {
            std::lock_guard<std::mutex> lock(params_mutex_);
            p = params_;
        }

        // Camera info snapshot.  Keep the image timestamp untouched.
        cv::Mat K, D;
        std::string camera_frame;
        int cam_w = 0;
        int cam_h = 0;
        bool has_info = false;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            K = camera_K_.clone();
            D = camera_D_.clone();
            camera_frame = camera_frame_;
            cam_w = camera_width_;
            cam_h = camera_height_;
            has_info = has_camera_info_;
        }

        if (!has_info) {
            K = cv::Mat::eye(3, 3, CV_64F);
            K.at<double>(0, 0) = p.fallback_fx;
            K.at<double>(1, 1) = p.fallback_fy;
            K.at<double>(0, 2) = p.fallback_cx;
            K.at<double>(1, 2) = p.fallback_cy;
            D = cv::Mat(1, static_cast<int>(p.fallback_distortion.size()), CV_64F);
            for (size_t i = 0; i < p.fallback_distortion.size(); ++i) {
                D.at<double>(0, static_cast<int>(i)) = p.fallback_distortion[i];
            }
            camera_frame = msg->header.frame_id.empty() ? p.camera_frame_fallback
                                                        : msg->header.frame_id;
            ROS_WARN_STREAM_THROTTLE(5.0,
                "No camera_info received yet; using YAML fallback intrinsics.");
        } else if (cam_w > 0 && cam_h > 0 &&
                   (cam_w != img.cols || cam_h != img.rows)) {
            // camera_info may have been recorded at a different resolution.
            const double sx = static_cast<double>(img.cols) / cam_w;
            const double sy = static_cast<double>(img.rows) / cam_h;
            K.at<double>(0, 0) *= sx;
            K.at<double>(0, 2) *= sx;
            K.at<double>(1, 1) *= sy;
            K.at<double>(1, 2) *= sy;
        }

        std_msgs::Header header = msg->header;
        if (!camera_frame.empty()) {
            header.frame_id = camera_frame;
        } else {
            camera_frame = msg->header.frame_id;
        }

        // -------------------------------------------------------------
        // Detect
        // -------------------------------------------------------------
        DetectResult result = detectFrame(img, p);

        std::vector<Detection> detections;
        detections.reserve(result.selected.size());
        for (const auto& pair : result.selected) {
            Detection det;
            if (solveArmorPose(pair, result.bars, K, D, p, det)) {
                detections.push_back(det);
            }
        }

        // Stable, unique per-frame IDs, ordered left-to-right.
        std::sort(detections.begin(), detections.end(),
                  [](const Detection& a, const Detection& b) {
                      return a.center.x < b.center.x;
                  });
        for (size_t i = 0; i < detections.size(); ++i) {
            detections[i].id = static_cast<int>(i);
        }

        // -------------------------------------------------------------
        // Publish /ft/detection
        // -------------------------------------------------------------
        ft_detection::DetectionFrame out_msg;
        out_msg.header = header;
        out_msg.detections.reserve(detections.size());
        const double score_norm =
            1.2 + 0.5 + 0.4 + 1.5 + p.pair_centrality_weight + 0.1;
        for (const auto& det : detections) {
            if (det.tvec[2] < p.min_depth || det.tvec[2] > p.max_depth) continue;

            ft_detection::Detection d;
            d.id = det.id;
            d.is_red = det.is_red;
            d.is_large_armor = det.is_large;
            d.pose.position.x = det.tvec[0];
            d.pose.position.y = det.tvec[1];
            d.pose.position.z = det.tvec[2];
            d.pose.orientation = rotationToQuaternion(det.rvec);
            for (size_t k = 0; k < det.corners.size(); ++k) {
                d.corners[k].x = det.corners[k].x;
                d.corners[k].y = det.corners[k].y;
                d.corners[k].z = 0.0f;
            }
            d.confidence = static_cast<float>(
                clampd(det.score / std::max(1.0, score_norm), 0.0, 1.0));
            out_msg.detections.push_back(d);

            if (p.publish_tf && !camera_frame.empty()) {
                geometry_msgs::TransformStamped tf_msg;
                tf_msg.header = header;
                tf_msg.child_frame_id = "armor_" + std::to_string(det.id);
                tf_msg.transform.translation.x = det.tvec[0];
                tf_msg.transform.translation.y = det.tvec[1];
                tf_msg.transform.translation.z = det.tvec[2];
                tf_msg.transform.rotation = d.pose.orientation;
                tf_broadcaster_->sendTransform(tf_msg);
            }
        }
        detection_pub_.publish(out_msg);

        // -------------------------------------------------------------
        // Debug image
        // -------------------------------------------------------------
        cv::Mat debug = img.clone();
        drawDebug(debug, result, detections, K, D);
        cv_bridge::CvImage debug_cv;
        debug_cv.header = header;
        debug_cv.encoding = sensor_msgs::image_encodings::BGR8;
        debug_cv.image = debug;
        debug_image_pub_.publish(debug_cv.toImageMsg());

        if (p.publish_mask) {
            cv::Mat mask_out;
            cv::bitwise_or(result.red_mask, result.blue_mask, mask_out);
            cv_bridge::CvImage mask_cv;
            mask_cv.header = header;
            mask_cv.encoding = sensor_msgs::image_encodings::MONO8;
            mask_cv.image = mask_out;
            debug_mask_pub_.publish(mask_cv.toImageMsg());
        }

        ROS_INFO_STREAM_THROTTLE(2.0,
            "frame " << header.stamp
                     << " bars=" << result.bars.size()
                     << " pairs=" << result.candidates.size()
                     << " selected=" << result.selected.size()
                     << " detections=" << out_msg.detections.size());
    }

    // ---------------------------------------------------------------------
    // Debug drawing
    // ---------------------------------------------------------------------
    void drawDebug(cv::Mat& img, const DetectResult& result,
                   const std::vector<Detection>& detections,
                   const cv::Mat& K, const cv::Mat& D) const {
        // All detected bars (yellow dots).  Useful when tuning the mask.
        for (const auto& bar : result.bars) {
            cv::circle(img, bar.center, 2, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
        }

        // Selected pairs without a valid pose get a small grey note.
        for (const auto& pair : result.selected) {
            bool has_detection = false;
            for (const auto& det : detections) {
                if (pointDistance(det.center, pair.center) < 3.0) {
                    has_detection = true;
                    break;
                }
            }
            if (has_detection) continue;
            cv::putText(img, "pair / PnP failed",
                        cv::Point(static_cast<int>(pair.center.x) - 55,
                                  static_cast<int>(pair.center.y) - 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(160, 160, 160), 1,
                        cv::LINE_AA);
        }

        for (const auto& det : detections) {
            const cv::Scalar color = det.is_red ? cv::Scalar(0, 0, 255)
                                                : cv::Scalar(255, 0, 0);
            std::vector<cv::Point> poly;
            poly.reserve(4);
            for (const auto& c : det.corners) {
                poly.emplace_back(cv::Point(cvRound(c.x), cvRound(c.y)));
            }
            cv::polylines(img, poly, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

            for (const auto& c : det.corners) {
                cv::circle(img, c, 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            }

            // Projected armour axes: x=red, y=green, z=blue.  The object
            // origin is the centre between the two bars.
            const std::vector<cv::Point3f> axes = {
                cv::Point3f(0.0f, 0.0f, 0.0f),
                cv::Point3f(0.05f, 0.0f, 0.0f),
                cv::Point3f(0.0f, 0.05f, 0.0f),
                cv::Point3f(0.0f, 0.0f, 0.05f)
            };
            std::vector<cv::Point2f> axes_img;
            try {
                const cv::Mat rvec = (cv::Mat_<double>(3, 1) << det.rvec[0],
                                      det.rvec[1], det.rvec[2]);
                const cv::Mat tvec = (cv::Mat_<double>(3, 1) << det.tvec[0],
                                      det.tvec[1], det.tvec[2]);
                cv::projectPoints(axes, rvec, tvec, K, D, axes_img);
                if (axes_img.size() == 4) {
                    cv::arrowedLine(img, axes_img[0], axes_img[1],
                                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);
                    cv::arrowedLine(img, axes_img[0], axes_img[2],
                                    cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.2);
                    cv::arrowedLine(img, axes_img[0], axes_img[3],
                                    cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.2);
                }
            } catch (const cv::Exception&) {
                // Debug drawing must never take down the node.
            }

            char text[192];
            std::snprintf(text, sizeof(text), "#%d %s %s %.2fm n=%.2f",
                          det.id, det.is_red ? "RED" : "BLUE",
                          det.is_large ? "LARGE" : "SMALL",
                          det.tvec[2], det.number_score);
            const int tx = std::max(2, static_cast<int>(det.center.x) - 55);
            float top_y = static_cast<float>(img.rows);
            for (const auto& c : det.corners) top_y = std::min(top_y, c.y);
            const int ty = std::max(16, static_cast<int>(top_y) - 8);
            cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
                        0.50, color, 1, cv::LINE_AA);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber camera_info_sub_;

    ros::Publisher detection_pub_;
    ros::Publisher debug_image_pub_;
    ros::Publisher debug_mask_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    boost::recursive_mutex dyn_mutex_;
    std::unique_ptr<dynamic_reconfigure::Server<ft_detector::ArmorReconfigureConfig>>
        dyn_server_;

    mutable std::mutex params_mutex_;
    Params params_;

    std::mutex camera_mutex_;
    cv::Mat camera_K_;
    cv::Mat camera_D_;
    std::string camera_frame_;
    int camera_width_ = 0;
    int camera_height_ = 0;
    bool has_camera_info_ = false;
};

}  // namespace ft_armor

// =============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "image_receiver_node");
    ros::NodeHandle nh;
    try {
        ft_armor::ImageReceiverNode node(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("armor detector failed: " << e.what());
        return 1;
    }
    return 0;
}
