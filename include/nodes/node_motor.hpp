
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <kinematics.hpp>

#define MOTOR_MAX_SPEED (127 + 7)
#define MOTOR_IDLE_SPEED (127)

#define LINE_THRESHOLD 70

namespace nodes
{
     class node_motor : public rclcpp::Node {
     public:
         // Constructor
         node_motor(const std::string& output_topic, const std::string& line_sensor_topic);
         // Destructor (default)
         ~node_motor() override = default;
 
     private:
        int8_t line_pos = 0; // -1 for left, 0 for center, 1 for right
        uint8_t motor_speed[2] = {MOTOR_IDLE_SPEED, MOTOR_IDLE_SPEED}; // Motor speeds (0-255)

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

        // Subscriber member variable for line sensor data
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscriber_line_sensor_;
        // Callback function for the line sensor subscriber
        void line_sensor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

        // Timer for periodic sending of motor data
        rclcpp::TimerBase::SharedPtr timer_;
     };
 }