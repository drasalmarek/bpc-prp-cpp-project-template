
#include "nodes/node_gain.hpp"

namespace nodes
{
    node_gain::node_gain(const std::string& input_topic,
        const std::string& output_topic,
        const uint8_t gain_factor) : Node("node_gain")
    {
        // Store the gain factor as a member variable
        gain_factor_ = gain_factor;

        // Create a publisher for UInt8 messages on the "counter" topic
        publisher_ = this->create_publisher<std_msgs::msg::UInt8>("processed_data", 10);

        // Create a subscriber for UInt8 messages on the "counter" topic
        subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            input_topic,
            10, 
            std::bind(&node_gain::subscriber_callback, this, std::placeholders::_1)
        );
    }

    void node_gain::subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        // Log the received message
        RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);

        // Create a new message with the data multiplied by a gain factor (e.g., 2)
        auto message = std_msgs::msg::UInt8();
        message.data = msg->data * gain_factor_;

        // Publish the new message
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
        publisher_->publish(message);
    }
}