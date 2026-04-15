
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nodes/node_motor.hpp>

namespace nodes
{
     class node_fsm : public rclcpp::Node {
     public:
         // Constructor
         node_fsm(const std::string& lidar_control_topic, 
            const std::string& wanted_speed_topic, 
            const std::string& wanted_angle_topic);
         // Destructor (default)
         ~node_fsm() override = default;
 
     private:
        float wanted_angle_ = 0.0;

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_speed_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_angle_publisher_;

        // Subscriber member variable
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
     };
 }