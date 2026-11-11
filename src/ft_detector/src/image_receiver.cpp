#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat camera_matrix;
cv::Mat dist_coeffs;
bool camera_info_received = false;

ros::Publisher debug_pub;

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    camera_matrix = cv::Mat(3, 3, CV_64F, (void*)msg->K.data()).clone();
    dist_coeffs = cv::Mat(1, 5, CV_64F, (void*)msg->D.data()).clone();
    camera_info_received = true;
}
void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        cv::Mat image = cv_ptr->image;

        cv::line(image, cv::Point(image.cols / 2, image.rows / 2 - 20), 
                 cv::Point(image.cols / 2, image.rows / 2 + 20), cv::Scalar(0, 255, 0), 2);
        cv::line(image, cv::Point(image.cols / 2 - 20, image.rows / 2), 
                 cv::Point(image.cols / 2 + 20, image.rows / 2), cv::Scalar(0, 255, 0), 2);

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header; 
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = image;
        debug_pub.publish(out_msg.toImageMsg());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_receiver_node");
    ros::NodeHandle nh;
    
    debug_pub = nh.advertise<sensor_msgs::Image>("/ft/debug/image", 1);
    
    ros::Subscriber info_sub = nh.subscribe("/hk_camera/camera_info", 1, cameraInfoCallback);
    ros::Subscriber img_sub = nh.subscribe("/hk_camera/image_raw/compressed", 1, imageCallback);

    ROS_INFO("Image Receiver Node Started!");
    ros::spin();
    return 0;
}
