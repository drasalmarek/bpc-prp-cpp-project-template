
#include "nodes/node_subscriber.hpp"

namespace nodes
{
    node_subscriber::node_subscriber(const std::string& input_topic) : Node("node_subscriber")
    {
        // Create a subscriber for UInt8 messages on the input topic
        subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            input_topic, 
            10, 
            std::bind(&node_subscriber::subscriber_callback, this, std::placeholders::_1)
        );
    }

    void node_subscriber::subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        // Log the received message
        RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
    }
}