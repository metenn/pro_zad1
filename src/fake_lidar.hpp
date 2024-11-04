#ifndef FAKE_LIDAR_H
#define FAKE_LIDAR_H

#include <chrono>
#include <memory>
#include <cmath>
#include <random>
#include <utility>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

class CameraConfig
{
public:
    const char *PARAM_MIN_RANGE = "min_range";
    const char *PARAM_MAX_RANGE = "max_range";
    const char *PARAM_MIN_ANGLE = "min_angle";
    const char *PARAM_MAX_ANGLE = "max_angle";
    const char *PARAM_SAMPLING_FREQUENCY = "sampling_frequency";
    const char *PARAM_CSV_PATH = "csv_path";
    const char *PARAM_CSV_DELIMITER = "csv_delimiter";
    const char *PARAM_NOISE_STDDEV = "noise_stddev";

    const double DEFAULT_MIN_RANGE = 0.2;
    const double DEFAULT_MAX_RANGE = 2.0;
    const double DEFAULT_MIN_ANGLE = -M_PI;
    const double DEFAULT_MAX_ANGLE = M_PI;
    const double DEFAULT_SAMPLING_FREQUENCY = 10.0;
    const double DEFAULT_NOISE_STDDEV = 0.1;
    const std::string DEFAULT_CSV_DELIMITER = ";";
    const std::string DEFAULT_CSV_PATH = "./lidar.csv";

    const char *DESCRIPTION_MIN_RANGE =
        "minimum range value [m]";
    const char *DESCRIPTION_MAX_RANGE =
        "maximum range value [m]";
    const char *DESCRIPTION_MIN_ANGLE =
        "start angle of the scan [rad]";
    const char *DESCRIPTION_MAX_ANGLE =
        "end angle of the scan [rad]";
    const char *DESCRIPTION_SAMPLING_FREQUENCY =
        "Number of full Scans per second";
    const char *DESCRIPTION_CSV_DELIMITER =
        "Character that separates columns in CSV";
    const char *DESCRIPTION_CSV_PATH =
        "Path to CSV file";
    const char *DESCRIPTION_NOISE_STDDEV =
        "Gaussian noise standard deviation";

public:
    std::pair<double, double> range;
    std::pair<double, double> angle;
    double sampling_frequency;
    char csv_delimiter;
    std::string csv_path;
    double noise_stddev;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);

    int get_scan_period_ms() const;
};

class FakeCamera : public rclcpp::Node
{
public:
    FakeCamera();

private:
    CameraConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
    rclcpp::TimerBase::SharedPtr _scan_timer;
    std::vector<std::vector<float>> _fake_lidar_data;
    int _tick;

    void _read_csv(const std::string &path);
    void _publish_frame();
    double _get_scan_step() const;
};

#endif /* FAKE_LIDAR_H */
