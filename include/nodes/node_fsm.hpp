
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace nodes
{
     class node_fsm : public rclcpp::Node {
     public:
         // Constructor
         node_fsm(const std::string& input_topic, const std::string& motor_topic);
         // Destructor (default)
         ~node_fsm() override = default;
 
     private:
        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
        // Subscriber member variable
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
     };
 }