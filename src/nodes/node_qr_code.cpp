
#include <nodes/node_qr_code.hpp>

namespace nodes
{
     node_qr_code::node_qr_code(const std::string& camera_topic, 
        const std::string& output_topic)
        : Node("node_qr_code")
    {
        // Initialize the publisher
        output_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(output_topic, 10);

        // Initialize the subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, std::bind(&node_qr_code::subscriber_callback, this, std::placeholders::_1));
    }

    void node_qr_code::subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        algorithms::ArucoDetector detector;
        auto arucos = detector.detect(cv_bridge::toCvCopy(msg, "bgr8")->image);

        if (arucos.empty()) 
        {
            return;
        }

        int marker_id = 12;
        for (const auto& aruco : arucos) 
        {
            if (aruco.id < marker_id) 
            {
                marker_id = aruco.id;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Detected marker with ID: %d", marker_id);
        auto message = std_msgs::msg::UInt8();
        message.data = marker_id;
        output_publisher_->publish(message);
    };
}