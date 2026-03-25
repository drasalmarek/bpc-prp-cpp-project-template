
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <kinematics.hpp>
#include <algorithms/pid.hpp>


// fungovalo to s MOTOR_MAX_OFFSET = 5 a PID_P = 1.0f

#define MOTOR_MAX_OFFSET 10
#define MOTOR_DEFAULT_SPEED (128 + MOTOR_MAX_OFFSET)

#define PID_P 1.0f
#define PID_I 0.05f
#define PID_D 0.012f

namespace nodes
{
     class node_motor : public rclcpp::Node {
     public:
         // Constructor
         node_motor(const std::string& line_estimation_topic, const std::string& output_topic);
         // Destructor (default)
         ~node_motor() override = default;
 
     private:
        int8_t line_pos = 0; // -1 for left, 0 for center, 1 for right
        uint8_t motor_speed[2] = {MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED}; // Motor speeds (0-255)

        algorithms::pid pid_controller_{PID_P, PID_I, PID_D};

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

        // Subscriber member variable for line sensor data
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_line_estimation_;
        // Callback function for the line sensor subscriber
        void line_estimation_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // Timer for periodic sending of motor data
        rclcpp::TimerBase::SharedPtr timer_;
     };
 }