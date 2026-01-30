#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 显示图像
        cv::imshow("Camera Image", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_subscriber");
    ros::NodeHandle nh;

    // 订阅相机图像话题
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("camera/image", 1, imageCallback);

    // 创建一个显示图像的窗口
    cv::namedWindow("Camera Image", cv::WINDOW_NORMAL);

    // 循环监听ROS话题
    ros::spin();

    return 0;
}
