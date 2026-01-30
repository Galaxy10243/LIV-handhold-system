#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;

    // 创建四个发布者，每个发布者对应一个相机
    ros::Publisher pub1 = nh.advertise<sensor_msgs::Image>("/camera2/image", 1);

    // 分别指定四个相机的参数和GStreamer管道字符串
    std::string pipeline1 = "v4l2src device=/dev/video2 ! image/jpeg, width=1920, height=1080, framerate=10/1 ! jpegdec ! videoconvert ! appsink sync=false";

    // 创建四个OpenCV VideoCapture对象，分别对应四个相机
    cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);

    if (!cap1.isOpened()){
        ROS_ERROR("Failed to open one of the cameras.");
        return -1;
    }

    // 获取相机的宽度和高度
    int width = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("Camera resolution: %dx%d", width, height);

    // 发布频率设定
    ros::Rate rate(10); // 设置为相机的帧率或适当值

    while (ros::ok()) {
        cv::Mat frame1;
        cap1 >> frame1; // 从第一个相机捕获一帧图像

        // 检查是否有相机捕获失败
        if (frame1.empty()){
            ROS_ERROR("Failed to capture frame from one of the cameras.");
            break;
        }
        // std::cout << "frame1 size: " << frame1.size() << std::endl;
        // 转换OpenCV图像到ROS图像消息
        cv_bridge::CvImage cv_image1;
        ros::Time captureTime = ros::Time::now();

        cv_image1.encoding = "bgr8"; // 假设图像格式为BGR
        cv_image1.image = frame1;
        sensor_msgs::Image ros_image1;
        cv_image1.toImageMsg(ros_image1);
	    ros_image1.header.stamp = captureTime;
        ros_image1.header.frame_id = "camera1";

        // 发布图像消息
        pub1.publish(ros_image1);

        // 检查ROS是否需要退出
        ros::spinOnce();
        rate.sleep();
    }

    // 释放相机资源
    cap1.release();

    return 0;
}
                                
