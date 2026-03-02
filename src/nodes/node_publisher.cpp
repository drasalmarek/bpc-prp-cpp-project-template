
#include "nodes/node_publisher.hpp"

namespace nodes
{
    node_publisher::node_publisher(const std::string& output_topic) : Node("node_publisher")
    {
        // Create a publisher for UInt8 messages on the output topic
        publisher_ = this->create_publisher<std_msgs::msg::UInt8>(output_topic, 10);

        // Create a timer that calls the timer_callback function every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&node_publisher::timer_callback, this)
        );
    }

    void node_publisher::timer_callback()
    {
        // Create a message and set its data to a counter value
        static uint8_t counter = 0;
        auto message = std_msgs::msg::UInt8();
        message.data = counter++;

        // Publish the message
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
        publisher_->publish(message);
    }
}