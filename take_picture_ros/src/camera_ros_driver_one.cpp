#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_camera_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Read parameters
    std::string topic_name;
    std::string pipeline1;
    int rate_hz;

    pnh.param<std::string>("topic_name", topic_name, "/image_rgb");
    pnh.param<std::string>("pipeline", pipeline1, "v4l2src device=/dev/video0 ! image/jpeg, width=1920, height=1080, framerate=60/1 ! jpegdec ! videoconvert ! appsink sync=false drop=true max-buffers=1");
    pnh.param<int>("rate_hz", rate_hz, 30);

    // 创建四个发布者，每个发布者对应一个相机
    ros::Publisher pub1 = nh.advertise<sensor_msgs::Image>(topic_name, 1);

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

    cap1.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // 发布频率设定，低于硬件采集频率
    ros::Rate rate(rate_hz);

    while (ros::ok()) {
        cv::Mat frame1;
        bool ok = cap1.read(frame1);

        if (!ok || frame1.empty()){
            ROS_WARN_THROTTLE(5.0, "No fresh frame available at publish time.");
        } else {
            cv_bridge::CvImage cv_image1;
            ros::Time captureTime = ros::Time::now();

            cv_image1.encoding = "bgr8";
            cv_image1.image = frame1;
            sensor_msgs::Image ros_image1;
            cv_image1.toImageMsg(ros_image1);
            ros_image1.header.stamp = captureTime;
            ros_image1.header.frame_id = "camera1";

            pub1.publish(ros_image1);
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 释放相机资源
    cap1.release();

    return 0;
}
                                
