#ifndef FAKE_CAMERA_H
#define FAKE_CAMERA_H

#include <chrono>
#include <memory>
#include <cmath>
#include <random>
#include <utility>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraConfig
{
public:
    const char *PARAM_VIDEO_FILENAME = "video_filename";

    const std::string DEFAULT_VIDEO_FILENAME = "0"; // 0 to "default camera" dla opencv

    const char *DESCRIPTION_VIDEO_FILENAME =
        "Video filename - if not provided, camera will be used";

public:
    std::string video_filename;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);
};

class FakeCamera : public rclcpp::Node
{
public:
    FakeCamera();

private:
    CameraConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _scan_publisher;
    rclcpp::TimerBase::SharedPtr _scan_timer;
    void _publish_frame();
    cv::VideoCapture _cap;
    double _fps;
    std::string _pix_fmt;
    long long _count;
};

#endif /* FAKE_CAMERA_H */
