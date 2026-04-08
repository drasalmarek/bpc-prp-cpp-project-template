
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <kinematics.hpp>
#include <algorithms/pid.hpp>


// fungovalo to s MOTOR_MAX_OFFSET = 5 a PID_P = 1.0f

#define MOTOR_MAX_OFFSET 10
#define MOTOR_DEFAULT_SPEED (128 + MOTOR_MAX_OFFSET)

#define PID_P 0.3f
#define PID_I 0.0f
#define PID_D 0.0f

namespace nodes
{
     class node_motor : public rclcpp::Node {
     public:
         // Constructor
         node_motor(const std::string& wanted_angle_topic, 
            const std::string& wanted_speed_topic,
            const std::string& imu_angle_topic, 
            const std::string& output_topic);
         // Destructor (default)
         ~node_motor() override = default;
 
     private:
        uint8_t motor_speed[2] = {MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED}; // Motor speeds (0-255)
        float wanted_speed_ = 0.0f; 
        float wanted_angle_ = 0.0f;

        algorithms::pid pid_controller_{PID_P, PID_I, PID_D};

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

        // Subscriber member variable for speed topic
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_wanted_speed_;
        // Callback function for the wanted speed subscriber
        void wanted_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // Subscriber member variable for wanted angle
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_wanted_angle_;
        // Callback function for the wanted angle subscriber
        void wanted_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // Subscriber member variable for IMU angle
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_imu_angle_;
        // Callback function for the IMU angle subscriber
        void imu_angle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        // Timer for periodic sending of motor data
        rclcpp::TimerBase::SharedPtr timer_;
     };
 }