// multi_cam_node.cpp
// C++17 required
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

// ------------ Configuration ------------
const int NUM_CAM = 1;
const int PUBLISH_FPS = 20;              // 主循环发布/显示帧率
const int DISPLAY_MAX_WIDTH = 640;       // 预览缩放宽度 (保持长宽比)
const bool USE_GSTREAMER_PIPELINE = true; // 如果使用 GStreamer 改为 true，否则会用 device id
// 若使用 GStreamer，请按你系统修改下面 pipeline 字符串（示例基于 v4l2）
std::vector<std::string> gst_pipelines = {
    "v4l2src device=/dev/video0 ! image/jpeg, width=1920, height=1080, framerate=10/1 ! jpegdec ! videoconvert ! appsink drop=true max-buffers=1 sync=false",
    // "v4l2src device=/dev/video1 ! video/x-raw, width=1920, height=1300, framerate=30/1 ! videoconvert ! appsink drop=true max-buffers=1 sync=false",
    // "v4l2src device=/dev/video2 ! video/x-raw, width=1920, height=1300, framerate=30/1 ! videoconvert ! appsink drop=true max-buffers=1 sync=false",
    // "v4l2src device=/dev/video3 ! video/x-raw, width=1920, height=1300, framerate=30/1 ! videoconvert ! appsink drop=true max-buffers=1 sync=false"
};
// 如果不使用 GStreamer，可以用 device indices like "0","1","2","3" by setting USE_GSTREAMER_PIPELINE=false
std::vector<int> dev_ids = {0,1,2,3};

// ------------ Utilities ------------
std::string getTimestampString() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d%H%M%S");
    // append milliseconds
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    oss << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

// ------------ Thread-safe latest-frame container ------------
struct FrameSlot {
    std::mutex m;
    cv::Mat frame;     // latest full-resolution frame
    std::atomic<bool> has_frame{false};
};

void captureThreadFunc(int cam_index, FrameSlot &slot, std::atomic<bool> &stop_flag, const std::string &open_str, bool use_gst) {
    cv::VideoCapture cap;
    if(use_gst) {
        cap.open(open_str, cv::CAP_GSTREAMER);
    } else {
        int id = std::stoi(open_str);
        cap.open(id, cv::CAP_ANY);
    }
    if (!cap.isOpened()) {
        ROS_ERROR("Camera %d: failed to open (%s)", cam_index+1, open_str.c_str());
        stop_flag = true;
        return;
    } else {
        ROS_INFO("Camera %d opened.", cam_index+1);
    }

    // Optionally you can set some properties (not always supported)
    // cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    while (ros::ok() && !stop_flag.load()) {
        cv::Mat frame;
        // read latest frame
        if(!cap.read(frame)) {
            // read failed, small sleep and retry
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // store latest frame (overwrite)
        {
            std::lock_guard<std::mutex> lk(slot.m);
            slot.frame = frame; // deep copy
            slot.has_frame.store(true);
        }
        // no sleep here — try to read as fast as camera provides
    }

    cap.release();
    ROS_INFO("Capture thread %d exiting.", cam_index+1);
}

// ------------ Main ------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_camera_node");
    ros::NodeHandle nh;

    // Prepare save directories
    for (int i = 0; i < NUM_CAM; ++i) {
        std::string dir = "cam" + std::to_string(i+1);
        try {
            fs::create_directories(dir);
        } catch (const std::exception &e) {
            ROS_WARN("Failed to create directory %s : %s", dir.c_str(), e.what());
        }
    }

    // Publishers
    std::vector<ros::Publisher> pubs(NUM_CAM);
    for (int i = 0; i < NUM_CAM; ++i) {
        std::string topic = "/camera" + std::to_string(i+1) + "/image";
        pubs[i] = nh.advertise<sensor_msgs::Image>(topic, 1);
    }

    // Frame slots & threads
    std::vector<FrameSlot> slots(NUM_CAM);
    std::vector<std::thread> threads;
    std::atomic<bool> stop_flag(false);

    for (int i = 0; i < NUM_CAM; ++i) {
        std::string open_str;
        if (USE_GSTREAMER_PIPELINE) open_str = gst_pipelines[i];
        else open_str = std::to_string(dev_ids[i]);

        threads.emplace_back(captureThreadFunc, i, std::ref(slots[i]), std::ref(stop_flag), open_str, USE_GSTREAMER_PIPELINE);
    }

    // Give threads a moment to warm up
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Main loop: publish at PUBLISH_FPS and display
    ros::Rate rate(PUBLISH_FPS);
    ROS_INFO("Multi-camera node started. Publishing at %d Hz. Press 1/2/3/4 to save corresponding camera frames. Press q or ESC to quit.", PUBLISH_FPS);

    cv::namedWindow("Camera1", cv::WINDOW_NORMAL);
    cv::namedWindow("Camera2", cv::WINDOW_NORMAL);
    cv::namedWindow("Camera3", cv::WINDOW_NORMAL);
    cv::namedWindow("Camera4", cv::WINDOW_NORMAL);

    while (ros::ok()) {
        // read latest frames (if any)
        std::vector<cv::Mat> cur_frames(NUM_CAM);
        for (int i = 0; i < NUM_CAM; ++i) {
            if (slots[i].has_frame.load()) {
                std::lock_guard<std::mutex> lk(slots[i].m);
                if (!slots[i].frame.empty()) {
                    cur_frames[i] = slots[i].frame.clone(); // safe copy for processing/publishing
                }
            }
        }

        // publish if available
        ros::Time stamp = ros::Time::now();
        for (int i = 0; i < NUM_CAM; ++i) {
            if (!cur_frames[i].empty()) {
                // publish with cv_bridge
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = stamp;
                cv_img.header.frame_id = "camera" + std::to_string(i+1);
                cv_img.encoding = "bgr8";
                cv_img.image = cur_frames[i];
                sensor_msgs::Image ros_img;
                cv_img.toImageMsg(ros_img);
                ros_img.header.stamp = stamp;
                pubs[i].publish(ros_img);
            }
        }

        // display (resized preview to reduce GUI cost)
        for (int i = 0; i < NUM_CAM; ++i) {
            if (!cur_frames[i].empty()) {
                cv::Mat disp;
                int w = cur_frames[i].cols;
                int h = cur_frames[i].rows;
                if (w > DISPLAY_MAX_WIDTH) {
                    double scale = (double)DISPLAY_MAX_WIDTH / (double)w;
                    cv::resize(cur_frames[i], disp, cv::Size(), scale, scale);
                } else {
                    disp = cur_frames[i];
                }
                std::string win = "Camera" + std::to_string(i+1);
                cv::imshow(win, disp);
            } else {
                // show a black placeholder if no frame yet
                cv::Mat placeholder(240, 320, CV_8UC3, cv::Scalar(0,0,0));
                cv::imshow("Camera" + std::to_string(i+1), placeholder);
            }
        }

        // key handling
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // 'q' or ESC
            ROS_INFO("Quit requested.");
            break;
        } else if (key == '1' || key == '2' || key == '3' || key == '4') {
            int cam_idx = (key - '1'); // 0-based
            if (cam_idx >= 0 && cam_idx < NUM_CAM) {
                if (!cur_frames[cam_idx].empty()) {
                    std::string fname = "cam" + std::to_string(cam_idx+1) + "/" + getTimestampString() + ".jpg";
                    try {
                        cv::imwrite(fname, cur_frames[cam_idx]);
                        ROS_INFO("Saved %s", fname.c_str());
                    } catch (const std::exception &e) {
                        ROS_ERROR("Failed to save %s : %s", fname.c_str(), e.what());
                    }
                } else {
                    ROS_WARN("No frame available for camera %d to save.", cam_idx+1);
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    // stop capture threads
    stop_flag.store(true);
    for (auto &t : threads) {
        if (t.joinable()) t.join();
    }

    cv::destroyAllWindows();
    ROS_INFO("Node exited cleanly.");
    return 0;
}
