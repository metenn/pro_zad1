#include "fake_camera.hpp"

//=================================================================================================
//                                        CAMERA CONFIG
//=================================================================================================

void CameraConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_VIDEO_FILENAME;
    node->declare_parameter(PARAM_VIDEO_FILENAME, DEFAULT_VIDEO_FILENAME, descriptor);
}

void CameraConfig::update_parameters(rclcpp::Node *node)
{
    this->video_filename = node->get_parameter(PARAM_VIDEO_FILENAME).as_string();
}

void CameraConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_VIDEO_FILENAME, this->video_filename.c_str());
}

//=================================================================================================
//                                         FAKE CAMERA NODE
//=================================================================================================

std::string unmarshall_fourcc(double fourcc)
{
    int codec = (int)fourcc;
    std::string str;
    str.push_back(codec & 0xFF);
    str.push_back((codec >> 8) & 0xFF);
    str.push_back((codec >> 16) & 0xFF);
    str.push_back((codec >> 24) & 0xFF);
    return str;
}

FakeCamera::FakeCamera() : rclcpp::Node("fake_camera")
{
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);
    _cap = cv::VideoCapture(_config.video_filename);
    if (!_cap.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Video file couldn't be opened");
        throw std::exception();
    }
    _fps = _cap.get(cv::CAP_PROP_FPS);
    std::string codec = unmarshall_fourcc(_cap.get(cv::CAP_PROP_FOURCC));

    RCLCPP_INFO(this->get_logger(), "Framerate: %f", _fps);
    RCLCPP_INFO(this->get_logger(), "Codec: %s", codec.c_str());
    _scan_publisher = this->create_publisher<sensor_msgs::msg::Image>("/cam", 3);
    _scan_timer = this->create_wall_timer(
        // mikrosekundy dla większej precyzji - dla wideo to ma znaczenie!
        std::chrono::microseconds((int)std::round(1'000'000 / _fps)),
        std::bind(&FakeCamera::_publish_frame, this));
}

void FakeCamera::_publish_frame()
{
    cv::Mat frame;
    _cap.read(frame);
    if (frame.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Stream ended");
        _cap.release();
        _scan_timer.get()->cancel();
        return;
    }
    // czy to zawsze bgr8? według testów nawet dla filmów z NV12 tak jest...
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "video_frame";
    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    _scan_publisher->publish(*msg.get());
    RCLCPP_DEBUG(this->get_logger(), "Frame %lld published", ++_count);
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
