#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

// ==========================================
// 全局变量：两个发布者
// ==========================================
ros::Publisher debug_pub;
ros::Publisher mask_pub;

// ==========================================
// 数据结构
// ==========================================
struct Lightbar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length;
    float width;
    float angle;
    int color; // 1: 红, 2: 蓝
};

struct Armor {
    cv::Point2f left_center;
    cv::Point2f right_center;
    cv::Point2f center;
    bool is_big;
    int color;
    int id;
};

// ==========================================
// 过滤函数：防止地胶反光被当成灯条
// ==========================================
bool check_lightbar_geometry(const Lightbar& bar) {
    float ratio = bar.length / bar.width;
    // 1. 长宽比限制：太胖或太细都不行
    if (ratio < 2.0 || ratio > 8.0) return false;
    // 2. 长度限制：太短的不要
    if (bar.length < 15.0) return false;
    // 3. 角度限制：灯条应该大致竖直，不能太歪
    if (std::abs(bar.angle) > 45.0) return false; 
    return true;
}

// ==========================================
// 核心识别函数
// ==========================================
std::vector<Armor> detect_armors(const cv::Mat& bgr_img, cv::Mat& out_mask) {
    std::vector<Armor> final_armors;
    std::vector<Lightbar> all_lightbars;

    cv::Mat hsv_img;
    cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat mask_red1, mask_red2, mask_red, mask_blue, mask_all;
    
    // ⚠️ 【调参重点 1】 HSV 阈值：调紧一点，防止背景发光干扰
    // 蓝色
    cv::inRange(hsv_img, cv::Scalar(100, 150, 150), cv::Scalar(124, 255, 255), mask_blue);
    // 红色（两段）
    cv::inRange(hsv_img, cv::Scalar(0, 180, 150), cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv_img, cv::Scalar(170, 180, 150), cv::Scalar(180, 255, 255), mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);
    
    // 合并红蓝二值图
    cv::bitwise_or(mask_red, mask_blue, mask_all);

    // 闭运算：把由于过曝断裂的灯条连起来
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_all, mask_all, cv::MORPH_CLOSE, kernel);

    // 把最终的 mask 给到输出参数，用于发布 debug 图
    out_mask = mask_all.clone();

    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_all, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < 30) continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);
        float w = rect.size.width;
        float h = rect.size.height;
        
        Lightbar bar;
        bar.rect = rect;
        bar.center = rect.center;
        bar.length = std::max(w, h);
        bar.width = std::min(w, h);

        // 角度统一化
        bar.angle = rect.angle;
        if (w < h) bar.angle += 90.0;
        if (bar.angle > 45.0) bar.angle -= 90.0;
        if (bar.angle < -45.0) bar.angle += 90.0;

        // 颜色判断
        if (mask_blue.at<uchar>(bar.center) > 0) bar.color = 2;
        else if (mask_red.at<uchar>(bar.center) > 0) bar.color = 1;
        else continue;

        if (!check_lightbar_geometry(bar)) continue;
        all_lightbars.push_back(bar);
    }

    if (all_lightbars.size() < 2) return final_armors;

    // 配对
    int armor_id = 0;
    for (size_t i = 0; i < all_lightbars.size(); ++i) {
        for (size_t j = i + 1; j < all_lightbars.size(); ++j) {
            Lightbar& left = all_lightbars[i];
            Lightbar& right = all_lightbars[j];

            if (left.center.x > right.center.x) std::swap(left, right);
            if (left.color != right.color) continue;

            float dy = std::abs(left.center.y - right.center.y);
            if (dy > std::max(left.length, right.length) * 0.8) continue;

            float len_ratio = left.length / right.length;
            if (len_ratio < 0.7 || len_ratio > 1.3) continue;

            float angle_diff = std::abs(left.angle - right.angle);
            if (angle_diff > 15.0) continue;

            float distance = cv::norm(left.center - right.center);
            if (distance < 20) continue; 

            // ⚠️ 【调参重点 2】 大小装甲板距离阈值
            bool is_big = (distance > 80.0);

            Armor armor;
            armor.left_center = left.center;
            armor.right_center = right.center;
            armor.center = (left.center + right.center) / 2.0f;
            armor.is_big = is_big;
            armor.color = left.color;
            armor.id = armor_id++;
            final_armors.push_back(armor);
        }
    }

    return final_armors;
}

// ==========================================
// ROS 回调函数
// ==========================================
void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;

        cv::Mat mask; // 准备一个空盆装 mask
        std::vector<Armor> armors = detect_armors(img, mask);

        // 画图
        for (const auto& armor : armors) {
            cv::circle(img, armor.left_center, 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(img, armor.right_center, 3, cv::Scalar(0, 255, 255), -1);

            cv::rectangle(img, 
                cv::Point(armor.left_center.x, armor.left_center.y - 15),
                cv::Point(armor.right_center.x, armor.right_center.y + 15),
                cv::Scalar(0, 255, 0), 2);

            std::string color_str = (armor.color == 1) ? "Red" : "Blue";
            std::string size_str = armor.is_big ? "Big" : "Small";
            std::string text = color_str + " " + size_str + " ID:" + std::to_string(armor.id);
            
            cv::putText(img, text, cv::Point(armor.center.x - 30, armor.center.y - 20), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }

        // 发布带框的彩图
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img;
        debug_pub.publish(out_msg.toImageMsg());

        // 发布二值化黑白图
        if (!mask.empty()) {
            cv_bridge::CvImage mask_msg;
            mask_msg.header = msg->header;
            mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
            mask_msg.image = mask;
            mask_pub.publish(mask_msg.toImageMsg());
        }

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_receiver_node");
    ros::NodeHandle nh;

    debug_pub = nh.advertise<sensor_msgs::Image>("/ft/debug/image", 1);
    mask_pub = nh.advertise<sensor_msgs::Image>("/ft/debug/mask", 1);

    ros::Subscriber sub = nh.subscribe("/hk_camera/image_raw/compressed", 1, imageCallback);
    
    ROS_INFO("Armor Detector Node Started!");
    ros::spin();
    return 0;
}