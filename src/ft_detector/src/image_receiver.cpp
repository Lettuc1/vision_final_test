#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

ros::Publisher debug_pub;
ros::Publisher mask_pub;

// ==========================================
// 1. 定义数据结构
// ==========================================
struct Lightbar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length; // 长边
    float width;  // 短边
    float angle;  // 统一化后的角度
    int color;    // 1: 红色, 2: 蓝色
};

struct Armor {
    cv::Point2f left_center;
    cv::Point2f right_center;
    cv::Point2f center;
    bool is_big; // true: 大装甲板, false: 小装甲板
    int color;
    int id;      // 目标ID
};

// ==========================================
// 2. 灯条几何过滤
// ==========================================
bool check_lightbar_geometry(const Lightbar& bar) {
    float ratio = bar.length / bar.width; // 长宽比
    // 过滤条件：太胖、太短、太歪的，直接扔掉！
    if (ratio < 1.5 || ratio > 10.0) return false;
    if (bar.length < 15.0) return false;
    if (std::abs(bar.angle) > 45.0) return false; 
    return true;
}

// ==========================================
// 3. 核心识别函数
// ==========================================
std::vector<Armor> detect_armors(const cv::Mat& bgr_img, cv::Mat& out_mask) {
    out_mask = mask_all;
    std::vector<Armor> final_armors;
    std::vector<Lightbar> all_lightbars;

    // --- 步骤 1：BGR 转 HSV ---
    cv::Mat hsv_img;
    cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);

    // --- 步骤 2：提取红色和蓝色掩码 ---
    cv::Mat mask_red, mask_blue, mask_red1, mask_red2;
    
    // 蓝色范围（可根据实际光照自行微调）
    cv::inRange(hsv_img, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), mask_blue);
    
    // 红色有两段（跨0度）
    cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv_img, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red); // 将两段红色合并

    // 把红蓝掩码合并，统一找轮廓
    cv::Mat mask_all;
    cv::bitwise_or(mask_red, mask_blue, mask_all);

    // 形态学操作（闭运算：填补灯条中间的过曝断裂处）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_all, mask_all, cv::MORPH_CLOSE, kernel);

    // --- 步骤 3：找轮廓并提取灯条 ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_all, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < 30) continue; // 面积太小直接忽略

        cv::RotatedRect rect = cv::minAreaRect(contour);
        float w = rect.size.width;
        float h = rect.size.height;
        
        Lightbar bar;
        bar.rect = rect;
        bar.center = rect.center;
        bar.length = std::max(w, h);
        bar.width = std::min(w, h);

        // ⚠️ 极其关键：角度统一化（学长提示的防跳变）
        // OpenCV 的 minAreaRect 返回的角度在 [0, 90) 或 [0, -90)
        bar.angle = rect.angle;
        if (w < h) bar.angle += 90.0;
        // 把角度映射到 [-45, 45] 之间，便于配对
        if (bar.angle > 45.0) bar.angle -= 90.0;
        if (bar.angle < -45.0) bar.angle += 90.0;

        // 判断颜色（看轮廓中心点对应的掩码图是红还是蓝）
        cv::Point2f center = rect.center;
        if (mask_blue.at<uchar>(center) > 0) bar.color = 2;
        else if (mask_red.at<uchar>(center) > 0) bar.color = 1;
        else continue;

        // 几何过滤（宁缺毋滥）
        if (!check_lightbar_geometry(bar)) continue;

        all_lightbars.push_back(bar);
    }

    if (all_lightbars.size() < 2) return final_armors;

    // --- 步骤 4：灯条配对 ---
    int armor_id = 0;
    for (size_t i = 0; i < all_lightbars.size(); ++i) {
        for (size_t j = i + 1; j < all_lightbars.size(); ++j) {
            Lightbar& left = all_lightbars[i];
            Lightbar& right = all_lightbars[j];

            // 保证 left 在左边
            if (left.center.x > right.center.x) std::swap(left, right);

            // 颜色必须一样
            if (left.color != right.color) continue;

            // 高度差不能太大（允许稍微倾斜的装甲板）
            float dy = std::abs(left.center.y - right.center.y);
            if (dy > std::max(left.length, right.length) * 0.8) continue;

            // 长度差不能太大
            float len_ratio = left.length / right.length;
            if (len_ratio < 0.7 || len_ratio > 1.3) continue;

            // 角度差不能太大（防止一正一反强行凑）
            float angle_diff = std::abs(left.angle - right.angle);
            if (angle_diff > 15.0) continue;

            // 距离计算
            float distance = cv::norm(left.center - right.center);
            if (distance < 20) continue; // 太近，可能是同一个灯条劈开了

            // --- 步骤 5：区分大/小装甲板 ---
            // 经验值：像素间距 > 80 算大，<=80 算小（根据分辨率可调）
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
// 4. ROS 回调函数
// ==========================================
void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        cv::Mat current_mask;
        // 调用我们写的识别函数
        std::vector<Armor> armors = detect_armors(img, current_mask);

        // --- 步骤 6：画图打标（替代 imshow） ---
        for (const auto& armor : armors) {
            // 画左右灯条中心点
            cv::circle(img, armor.left_center, 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(img, armor.right_center, 3, cv::Scalar(0, 255, 255), -1);

            // 画装甲板外框
            cv::rectangle(img, 
                cv::Point(armor.left_center.x, armor.left_center.y - 15),
                cv::Point(armor.right_center.x, armor.right_center.y + 15),
                cv::Scalar(0, 255, 0), 2);

            // 标记颜色、大小、ID
            std::string color_str = (armor.color == 1) ? "Red" : "Blue";
            std::string size_str = armor.is_big ? "Big" : "Small";
            std::string text = color_str + " " + size_str + " ID:" + std::to_string(armor.id);
            
            cv::putText(img, text, cv::Point(armor.center.x - 30, armor.center.y - 20), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }

        // 把画好的图转成 ROS 消息发出去
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header; // 必须用原图时间戳！
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img;
        debug_pub.publish(out_msg.toImageMsg());

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    if (!current_mask.empty()) {
    cv_bridge::CvImage mask_msg;
    mask_msg.header = msg->header; // 必须用原图时间戳......
    mask_msg.encoding = sensor_msgs::image_encodings::MONO8; // 黑白图格式
    mask_msg.image = current_mask;
    mask_pub.publish(mask_msg.toImageMsg());
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