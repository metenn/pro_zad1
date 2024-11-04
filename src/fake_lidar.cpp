#include "fake_lidar.hpp"
#include <fstream>

//=================================================================================================
//                                        LIDAR CONFIG
//=================================================================================================

int CameraConfig::get_scan_period_ms() const
{
    return std::lround(1000 / sampling_frequency);
}

void CameraConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);
    descriptor.description = DESCRIPTION_CSV_DELIMITER;
    node->declare_parameter(PARAM_CSV_DELIMITER, DEFAULT_CSV_DELIMITER, descriptor);
    descriptor.description = DESCRIPTION_CSV_PATH;
    node->declare_parameter(PARAM_CSV_PATH, DEFAULT_CSV_PATH, descriptor);
    descriptor.description = DESCRIPTION_NOISE_STDDEV;
    node->declare_parameter(PARAM_NOISE_STDDEV, DEFAULT_NOISE_STDDEV, descriptor);
}

void CameraConfig::update_parameters(rclcpp::Node *node)
{
    this->range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();
    auto delim_str = node->get_parameter(PARAM_CSV_DELIMITER).as_string();
    if (delim_str.length() > 1)
    {
        RCLCPP_WARN(node->get_logger(), "Parameter '%s' has to be of length = 1, truncating to first character", PARAM_CSV_DELIMITER);
    }
    this->csv_delimiter = delim_str.at(0);
    this->csv_path = node->get_parameter(PARAM_CSV_PATH).as_string();
    this->noise_stddev = node->get_parameter(PARAM_NOISE_STDDEV).as_double();
}

void CameraConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, this->range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, this->range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, this->angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, this->angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY,
                this->sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_NOISE_STDDEV,
                this->noise_stddev);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %c", PARAM_CSV_DELIMITER,
                this->csv_delimiter);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_CSV_PATH,
                this->csv_path.c_str());
}

//=================================================================================================
//                                         FAKE LIDAR NODE
//=================================================================================================

double FakeCamera::_get_scan_step() const
{
    return (_config.angle.second - _config.angle.first) / this->_fake_lidar_data[0].size();
}

FakeCamera::FakeCamera() : rclcpp::Node("fake_lidar")
{
    _tick = 0;
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _read_csv(_config.csv_path);
    _config.print_config(this);
    _scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    _scan_timer = this->create_wall_timer(
        std::chrono::milliseconds(_config.get_scan_period_ms()),
        std::bind(&FakeCamera::_publish_frame, this));
}

void FakeCamera::_publish_frame()
{
    auto msg = sensor_msgs::msg::LaserScan();

    msg.header.stamp = this->now();
    // nie mamy zdefiniowanej transformacji z tf2 ale nie potrzebna jest dla
    // tego przykładu
    msg.header.frame_id = "laser_frame";

    msg.angle_min = _config.angle.first;
    msg.angle_max = _config.angle.second;
    msg.range_min = _config.range.first;
    msg.range_max = _config.range.second;
    msg.angle_increment = this->_get_scan_step();
    msg.time_increment = 0;
    msg.scan_time = _config.get_scan_period_ms() / 1000.0;

    auto ranges_copy = std::vector(this->_fake_lidar_data[_tick]);
    std::random_device rd;
    unsigned seed = rd() ^ static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator(seed);
    constexpr double mean = 0.0;
    std::normal_distribution<float> distribution(mean, _config.noise_stddev);
    for (auto &a : ranges_copy)
    {
        if (_config.noise_stddev != 0)
        {
            a += distribution(generator);
        }
        if (a < _config.range.first)
        {
            RCLCPP_ERROR(this->get_logger(), "Lidar reading out of range: %f < %f (minimum range)", a, _config.range.first);
            goto end;
        }
        else if (a > _config.range.second)
        {
            RCLCPP_ERROR(this->get_logger(), "Lidar reading out of range: %f > %f (maximum range)", a, _config.range.second);
            goto end;
        }
    }
    msg.ranges = ranges_copy;
    _scan_publisher->publish(msg);
end:
    _tick = (_tick + 1) % this->_fake_lidar_data.size();
}

void FakeCamera::_read_csv(const std::string &path)
{
    std::ifstream file(path);
    std::string str;
    std::vector<std::vector<float>> parsed;
    while (std::getline(file, str))
    {
        std::vector<float> vec;
        int last = 0;
        for (size_t i = 0; i < str.length(); i++)
        {
            if (str[i] == _config.csv_delimiter)
            {
                vec.push_back(std::stod(str.substr(last, i - last)));
                last = i + 1;
            }
        }
        vec.push_back(std::stod(str.substr(last)));
        parsed.push_back(vec);
    }
    this->_fake_lidar_data = parsed;
    RCLCPP_INFO(this->get_logger(), "Scan step = %f", this->_get_scan_step());
}

//=================================================================================================
//                                          MAIN
//=================================================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
