#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);
    
    // 指定相机参数和GStreamer管道字符串
    std::string pipeline = "v4l2src device=/dev/video0 ! video/x-raw, width=1920, height=1080, framerate=30/1 ! videoconvert ! appsink";

    // 创建OpenCV VideoCapture对象
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    double frameRate = cap.get(cv::CAP_PROP_FPS);
    
    // 检查相机是否成功打开
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }

    // 获取相机的宽度和高度
    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("Camera resolution: %dx%d", width, height);

    // 发布频率设定
    ros::Rate rate(30); // 设置为相机的帧率或适当值

    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame; // 从相机捕获一帧图像

        if (frame.empty()) {
            ROS_ERROR("Failed to capture frame.");
            break;
        }

        // 转换OpenCV图像到ROS图像消息
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "bgr8"; // 假设图像格式为BGR
        cv_image.image = frame;
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);

        // 发布图像消息
        pub.publish(ros_image);

        // 检查ROS是否需要退出
        ros::spinOnce();
        rate.sleep();
    }

    cap.release(); // 释放相机资源
    return 0;
}