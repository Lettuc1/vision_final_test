// =============================================================================
// Single-target constant-velocity tracker (base part 6, no lead yet).
//
// Input : /ft/detection  (ft_detection/DetectionFrame)
// Output: /ft/track      (ft_tracker/Track)
//
// State : [x, y, z, vx, vy, vz] in the camera frame, metres / m/s
// Observation: [x, y, z] of the armour centre
//
// The node locks onto one detection.  While observations hit, it publishes the
// corrected state.  When observations are missing it keeps predicting and sets
// tracking=false.  A track that has not been seen for max_lost_time seconds is
// deleted and a later detection starts a new track id.
// =============================================================================

#include <ros/ros.h>
#include <ft_detection/DetectionFrame.h>
#include <ft_tracker/Track.h>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace ft_tracker_node {

struct Params {
    std::string detection_topic = "/ft/detection";
    std::string track_topic = "/ft/track";
    int queue_size = 50;

    double gate_distance = 0.5;       // metres
    double max_lost_time = 0.5;       // seconds
    double q_accel = 4.0;             // m/s^2 acceleration spectral density
    double r_position = 0.0025;       // m^2 (5 cm std)
    double initial_position_var = 0.1;
    double initial_velocity_var = 1.0;
    double min_dt = 0.001;
    double max_dt = 0.5;
};

class TrackerNode {
public:
    TrackerNode() : nh_(), pnh_("~") {
        loadParams();
        sub_ = nh_.subscribe(p_.detection_topic, p_.queue_size,
                             &TrackerNode::detectionCallback, this);
        pub_ = nh_.advertise<ft_tracker::Track>(p_.track_topic, p_.queue_size);
        ROS_INFO_STREAM("ft_tracker: listening " << p_.detection_topic
                        << " -> publishing " << p_.track_topic
                        << ", gate=" << p_.gate_distance
                        << " m, max_lost=" << p_.max_lost_time << " s");
    }

private:
    void loadParams() {
        pnh_.param<std::string>("detection_topic", p_.detection_topic, p_.detection_topic);
        pnh_.param<std::string>("track_topic", p_.track_topic, p_.track_topic);
        pnh_.param<int>("queue_size", p_.queue_size, p_.queue_size);
        pnh_.param<double>("gate_distance", p_.gate_distance, p_.gate_distance);
        pnh_.param<double>("max_lost_time", p_.max_lost_time, p_.max_lost_time);
        pnh_.param<double>("q_accel", p_.q_accel, p_.q_accel);
        pnh_.param<double>("r_position", p_.r_position, p_.r_position);
        pnh_.param<double>("initial_position_var", p_.initial_position_var, p_.initial_position_var);
        pnh_.param<double>("initial_velocity_var", p_.initial_velocity_var, p_.initial_velocity_var);
        pnh_.param<double>("min_dt", p_.min_dt, p_.min_dt);
        pnh_.param<double>("max_dt", p_.max_dt, p_.max_dt);

        if (p_.queue_size < 1) p_.queue_size = 1;
        if (p_.max_lost_time <= 0.0) p_.max_lost_time = 0.5;
        if (p_.gate_distance <= 0.0) p_.gate_distance = 0.5;
    }

    // Highest-confidence detection becomes the lock target on acquisition.
    int selectDetection(const ft_detection::DetectionFrame& frame) const {
        int best = -1;
        double best_score = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < frame.detections.size(); ++i) {
            const auto& d = frame.detections[i];
            double score = std::isfinite(d.confidence)
                               ? static_cast<double>(d.confidence) : 0.0;
            // Tiny area tie-break: a well-visible plate is preferred over a
            // one-pixel sliver with the same confidence.
            double area = 0.0;
            for (int k = 0; k < 4; ++k) {
                const auto& a = d.corners[k];
                const auto& b = d.corners[(k + 1) % 4];
                area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
            }
            score += 1.0e-8 * std::fabs(area);
            if (score > best_score) {
                best_score = score;
                best = static_cast<int>(i);
            }
        }
        return best;
    }

    void initFilter(double x, double y, double z) {
        kf_.init(6, 3, 0);

        cv::setIdentity(kf_.transitionMatrix);
        kf_.measurementMatrix.setTo(cv::Scalar(0));
        for (int i = 0; i < 3; ++i) {
            kf_.measurementMatrix.at<float>(i, i) = 1.0f;
        }

        kf_.processNoiseCov.setTo(cv::Scalar(0));
        kf_.measurementNoiseCov.setTo(cv::Scalar(0));
        for (int i = 0; i < 3; ++i) {
            kf_.measurementNoiseCov.at<float>(i, i) =
                static_cast<float>(p_.r_position);
        }

        kf_.statePost.setTo(cv::Scalar(0));
        kf_.statePost.at<float>(0, 0) = static_cast<float>(x);
        kf_.statePost.at<float>(1, 0) = static_cast<float>(y);
        kf_.statePost.at<float>(2, 0) = static_cast<float>(z);

        kf_.errorCovPost.setTo(cv::Scalar(0));
        for (int i = 0; i < 3; ++i) {
            kf_.errorCovPost.at<float>(i, i) =
                static_cast<float>(p_.initial_position_var);
            kf_.errorCovPost.at<float>(i + 3, i + 3) =
                static_cast<float>(p_.initial_velocity_var);
        }
        kf_ready_ = true;
    }

    void predict(double dt) {
        if (!kf_ready_) return;
        dt = std::max(p_.min_dt, std::min(p_.max_dt, dt));

        kf_.transitionMatrix.setTo(cv::Scalar(0));
        for (int i = 0; i < 6; ++i) {
            kf_.transitionMatrix.at<float>(i, i) = 1.0f;
        }
        for (int i = 0; i < 3; ++i) {
            kf_.transitionMatrix.at<float>(i, i + 3) = static_cast<float>(dt);
        }

        // White-acceleration model for each axis:
        // [position, velocity] Q block scaled by q_accel^2 and dt.
        const double q = p_.q_accel * p_.q_accel;
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        kf_.processNoiseCov.setTo(cv::Scalar(0));
        for (int i = 0; i < 3; ++i) {
            kf_.processNoiseCov.at<float>(i, i) =
                static_cast<float>(0.25 * dt4 * q);
            kf_.processNoiseCov.at<float>(i, i + 3) =
                static_cast<float>(0.5 * dt3 * q);
            kf_.processNoiseCov.at<float>(i + 3, i) =
                static_cast<float>(0.5 * dt3 * q);
            kf_.processNoiseCov.at<float>(i + 3, i + 3) =
                static_cast<float>(dt2 * q);
        }
        kf_.predict();
    }

    void correct(double x, double y, double z) {
        cv::Mat z_meas(3, 1, CV_32F);
        z_meas.at<float>(0, 0) = static_cast<float>(x);
        z_meas.at<float>(1, 0) = static_cast<float>(y);
        z_meas.at<float>(2, 0) = static_cast<float>(z);
        kf_.correct(z_meas);
    }

    void startTrack(const ft_detection::Detection& det, const ros::Time& stamp,
                    const std::string& frame_id) {
        initFilter(det.pose.position.x, det.pose.position.y, det.pose.position.z);
        has_track_ = true;
        track_id_ = next_id_++;
        last_seen_ = stamp;
        last_stamp_ = stamp;
        tracking_ = true;
        ROS_INFO_STREAM("ft_tracker: acquired track " << track_id_
                        << " at (" << det.pose.position.x << ", "
                        << det.pose.position.y << ", " << det.pose.position.z << ")");
        publish(stamp, frame_id, true, true);
    }

    void publish(const ros::Time& stamp, const std::string& frame_id,
                 bool tracking, bool use_post) {
        if (!has_track_ || !kf_ready_) return;

        const cv::Mat& state = use_post ? kf_.statePost : kf_.statePre;
        if (state.empty()) return;

        const double x = state.at<float>(0, 0);
        const double y = state.at<float>(1, 0);
        const double z = state.at<float>(2, 0);
        const double vx = state.at<float>(3, 0);
        const double vy = state.at<float>(4, 0);
        const double vz = state.at<float>(5, 0);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
            ROS_WARN("ft_tracker: non-finite state, dropping track");
            has_track_ = false;
            kf_ready_ = false;
            return;
        }

        ft_tracker::Track out;
        out.header.stamp = stamp;
        out.header.frame_id = frame_id;
        out.id = track_id_;
        out.tracking = tracking;
        out.position.x = x;
        out.position.y = y;
        out.position.z = z;
        out.velocity.x = vx;
        out.velocity.y = vy;
        out.velocity.z = vz;
        pub_.publish(out);
    }

    void detectionCallback(const ft_detection::DetectionFrame::ConstPtr& msg) {
        const ros::Time stamp = msg->header.stamp;
        if (stamp.isZero()) {
            ROS_WARN_THROTTLE(2.0, "ft_tracker: zero image stamp, ignored");
            return;
        }

        // A track that has not been observed for too long is deleted before
        // association, so a returning target starts a fresh track id.
        if (has_track_ && (stamp - last_seen_).toSec() > p_.max_lost_time) {
            ROS_INFO_STREAM("ft_tracker: track " << track_id_
                            << " lost for " << (stamp - last_seen_).toSec()
                            << " s, deleted");
            has_track_ = false;
            kf_ready_ = false;
        }

        bool predicted = false;
        if (has_track_ && !last_stamp_.isZero()) {
            double dt = (stamp - last_stamp_).toSec();
            if (dt >= p_.min_dt) {
                dt = std::min(dt, p_.max_dt);
                predict(dt);
                predicted = true;
            }
        }

        // Predict used by association: statePre when predict() ran, otherwise
        // the last posterior.
        double px = 0.0;
        double py = 0.0;
        double pz = 0.0;
        if (has_track_) {
            const cv::Mat& pred = (predicted && !kf_.statePre.empty())
                                      ? kf_.statePre : kf_.statePost;
            if (!pred.empty()) {
                px = pred.at<float>(0, 0);
                py = pred.at<float>(1, 0);
                pz = pred.at<float>(2, 0);
            }
        }

        int best = -1;
        double best_dist = std::numeric_limits<double>::max();
        if (has_track_) {
            for (size_t i = 0; i < msg->detections.size(); ++i) {
                const auto& pos = msg->detections[i].pose.position;
                const double dx = pos.x - px;
                const double dy = pos.y - py;
                const double dz = pos.z - pz;
                const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (d <= p_.gate_distance && d < best_dist) {
                    best_dist = d;
                    best = static_cast<int>(i);
                }
            }
        }

        if (has_track_ && best >= 0) {
            const auto& pos = msg->detections[best].pose.position;
            correct(pos.x, pos.y, pos.z);
            last_seen_ = stamp;
            last_stamp_ = stamp;
            tracking_ = true;
            publish(stamp, msg->header.frame_id, true, true);
            return;
        }

        if (has_track_) {
            // No matching observation: keep predicting.
            tracking_ = false;
            last_stamp_ = stamp;
            publish(stamp, msg->header.frame_id, false, !predicted);
            return;
        }

        // No active track: acquire the best detection, if any.
        if (!msg->detections.empty()) {
            const int idx = selectDetection(*msg);
            if (idx >= 0) {
                startTrack(msg->detections[idx], stamp, msg->header.frame_id);
            }
        } else {
            last_stamp_ = stamp;
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    Params p_;
    cv::KalmanFilter kf_;
    bool kf_ready_ = false;
    bool has_track_ = false;
    bool tracking_ = false;
    uint32_t next_id_ = 1;
    uint32_t track_id_ = 0;
    ros::Time last_stamp_;
    ros::Time last_seen_;
};

}  // namespace ft_tracker_node

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker_node");
    ft_tracker_node::TrackerNode node;
    ros::spin();
    return 0;
}
