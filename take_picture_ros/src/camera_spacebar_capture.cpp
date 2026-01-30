#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

bool createDirectory(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Directory doesn't exist, try to create it
        if (mkdir(path.c_str(), 0755) == 0) {
            return true;
        } else {
            return false;
        }
    } else if (info.st_mode & S_IFDIR) {
        // Directory exists
        return true;
    } else {
        // Path exists but is not a directory
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_spacebar_capture");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Read parameters
    std::string topic_name;
    std::string pipeline1;
    int rate_hz;
    int initial_saved_count;
    
    pnh.param<std::string>("topic_name", topic_name, "/image_rgb");
    pnh.param<std::string>("pipeline", pipeline1, "v4l2src device=/dev/video0 ! image/jpeg, width=1920, height=1080, framerate=60/1 ! jpegdec ! videoconvert ! appsink sync=false drop=true max-buffers=1");
    pnh.param<int>("rate_hz", rate_hz, 30);
    pnh.param<int>("saved_count", initial_saved_count, 0);
    
    // Create publisher for camera images
    ros::Publisher pub1 = nh.advertise<sensor_msgs::Image>(topic_name, 1);

    // Create OpenCV VideoCapture object
    cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);

    if (!cap1.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }

    // Get camera resolution
    int width = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("Camera resolution: %dx%d", width, height);

    cap1.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Create save directory
    std::string save_directory = "/home/lyz/handheld_ws/src/take_picture_ros/captured_images";
    if (!createDirectory(save_directory)) {
        ROS_ERROR("Failed to create save directory: %s", save_directory.c_str());
        return -1;
    }

    // Publishing frequency
    ros::Rate rate(rate_hz);
    
    int saved_count = initial_saved_count;
    
    ROS_INFO("Camera spacebar capture node started!");
    ROS_INFO("Press SPACEBAR to save an image, ESC to quit");
    ROS_INFO("Images will be saved to: %s", save_directory.c_str());

    while (ros::ok()) {
        cv::Mat frame1;
        bool ok = cap1.read(frame1);

        if (!ok || frame1.empty()) {
            ROS_WARN_THROTTLE(5.0, "No fresh frame available at publish time.");
        } else {
            // Display the image
            cv::namedWindow("Camera - Press SPACEBAR to save, ESC to quit", cv::WINDOW_NORMAL);
            cv::resizeWindow("Camera - Press SPACEBAR to save, ESC to quit", 960, 540); // Half size for display
            cv::imshow("Camera - Press SPACEBAR to save, ESC to quit", frame1);
            
            // Check for keyboard input
            int key = cv::waitKey(1) & 0xFF;
            
            if (key == 32) { // Spacebar pressed (ASCII code 32)
                std::string timestamp = getCurrentTimeString();
                std::string filename = save_directory + "/" + std::to_string(saved_count) + ".jpg";
                
                if (cv::imwrite(filename, frame1)) {
                    saved_count++;
                    ROS_INFO("Image saved: %s (Total saved: %d)", filename.c_str(), saved_count);
                } else {
                    ROS_ERROR("Failed to save image: %s", filename.c_str());
                }
            } else if (key == 27) { // ESC key pressed
                ROS_INFO("ESC pressed, shutting down...");
                break;
            }

            // Prepare and publish ROS message
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

    // Release camera resources
    cap1.release();
    cv::destroyAllWindows();
    
    ROS_INFO("Camera spacebar capture node shutdown. Total images saved: %d", saved_count);
    return 0;
}
