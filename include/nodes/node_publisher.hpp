#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes
{
     class node_publisher : public rclcpp::Node {
     public:
         // Constructor
         node_publisher(const std::string& output_topic);
         // Destructor (default)
         ~node_publisher() override = default;
 
     private:
        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
        // Timer member variable
        rclcpp::TimerBase::SharedPtr timer_;
        // Callback function for the timer
        void timer_callback();
     };
 }
