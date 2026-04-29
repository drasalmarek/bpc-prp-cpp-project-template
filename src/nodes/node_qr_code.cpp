
#include <nodes/node_qr_code.hpp>

namespace nodes
{
     node_qr_code::node_qr_code(const std::string& camera_topic, 
        const std::string& output_topic)
        : Node("node_qr_code")
    {
        // Initialize the publisher
        output_publisher_ = this->create_publisher<std_msgs::msg::Int16>(output_topic, 10);

        // Initialize the subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, std::bind(&node_qr_code::subscriber_callback, this, std::placeholders::_1));
    }

    void node_qr_code::subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "camera frame received");   

        algorithms::ArucoDetector detector;
        auto arucos = detector.detect(cv_bridge::toCvCopy(msg, "bgr8")->image);

        if (arucos.empty()) 
        {
            return;
        }

        // For simplicity, we will just take the first detected marker and publish its ID as a float
        int marker_id = arucos[0].id;
        RCLCPP_INFO(this->get_logger(), "Detected marker with ID: %d", marker_id);
        auto message = std_msgs::msg::Int16();
        message.data = marker_id;
        output_publisher_->publish(message);
    };
}