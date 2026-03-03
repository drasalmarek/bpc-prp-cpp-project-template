
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes
{
     class node_gain : public rclcpp::Node {
     public:
         // Constructor
         node_gain(const std::string& input_topic,
            const std::string& output_topic,
            const uint8_t gain_factor);
         // Destructor (default)
         ~node_gain() override = default;
 
     private:
        // Gain factor member variable
        uint8_t gain_factor_;
        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
        // Subscriber member variable
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg);
     };
 }