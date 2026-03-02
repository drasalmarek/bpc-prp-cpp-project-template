
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes
{
     class node_subscriber : public rclcpp::Node {
     public:
         // Constructor
         node_subscriber(const std::string& input_topic);
         // Destructor (default)
         ~node_subscriber() override = default;
 
     private:
        // Subscriber member variable
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg);
     };
 }
