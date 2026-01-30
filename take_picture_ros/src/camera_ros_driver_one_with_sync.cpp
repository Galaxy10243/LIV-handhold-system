#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// 共享内存时间戳结构体，与mvs_ros_driver和livox_ros_driver2保持一致
struct time_stamp
{
    int64_t high;
    int64_t low;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_publisher_with_sync");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Read parameters
    std::string topic_name;
    std::string pipeline1;
    int rate_hz;
    bool enable_sync;

    pnh.param<std::string>("topic_name", topic_name, "/image_rgb");
    pnh.param<std::string>("pipeline", pipeline1, "v4l2src device=/dev/video0 ! image/jpeg, width=1920, height=1080, framerate=60/1 ! jpegdec ! videoconvert ! appsink sync=false drop=true max-buffers=1");
    pnh.param<int>("rate_hz", rate_hz, 30);
    pnh.param<bool>("enable_sync", enable_sync, true); // 是否启用时间同步

    // 创建发布者
    ros::Publisher pub1 = nh.advertise<sensor_msgs::Image>(topic_name, 1);

    // 打开共享内存以读取Livox的同步时间戳
    time_stamp *pointt = nullptr;
    int fd = -1;

    if (enable_sync)
    {
        const char *user_name = getlogin();
        std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/timeshare";
        const char *shared_file_name = path_for_time_stamp.c_str();

        fd = open(shared_file_name, O_RDWR);
        if (fd < 0)
        {
            ROS_WARN("Failed to open shared memory file: %s. Will use ros::Time::now() instead.", shared_file_name);
            enable_sync = false;
        }
        else
        {
            pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd, 0);
            if (pointt == MAP_FAILED)
            {
                ROS_WARN("Failed to mmap shared memory. Will use ros::Time::now() instead.");
                close(fd);
                enable_sync = false;
                pointt = nullptr;
            }
            else
            {
                ROS_INFO("Successfully opened shared memory for time synchronization.");
            }
        }
    }

    // 创建OpenCV VideoCapture对象
    cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);

    if (!cap1.isOpened())
    {
        ROS_ERROR("Failed to open the camera.");
        if (pointt && pointt != MAP_FAILED)
        {
            munmap(pointt, sizeof(time_stamp));
        }
        if (fd >= 0)
        {
            close(fd);
        }
        return -1;
    }

    // 获取相机的宽度和高度
    int width = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("Camera resolution: %dx%d", width, height);

    cap1.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // 发布频率设定，低于硬件采集频率
    ros::Rate rate(rate_hz);

    while (ros::ok())
    {
        cv::Mat frame1;
        bool ok = cap1.read(frame1);

        if (!ok || frame1.empty())
        {
            ROS_WARN_THROTTLE(5.0, "No fresh frame available at publish time.");
        }
        else
        {
            cv_bridge::CvImage cv_image1;
            ros::Time captureTime;

            // 根据是否启用同步来决定时间戳来源
            if (enable_sync && pointt != nullptr && pointt != MAP_FAILED && pointt->low != 0)
            {
                // 使用共享内存中的Livox同步时间戳
                int64_t b = pointt->low;
                double time_pc = b / 1000000000.0;
                captureTime = ros::Time(time_pc);
                ROS_DEBUG("Using synced timestamp: %.9f", time_pc);
            }
            else
            {
                // 使用系统当前时间
                captureTime = ros::Time::now();
            }

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

    // 释放共享内存
    if (pointt && pointt != MAP_FAILED)
    {
        munmap(pointt, sizeof(time_stamp));
    }
    if (fd >= 0)
    {
        close(fd);
    }

    return 0;
}
