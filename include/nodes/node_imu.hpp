
#pragma once 

#include <rclcpp/rclcpp.hpp>    
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes
{
     class node_imu : public rclcpp::Node {
     public:
         // Constructor
         node_imu(const std::string& imu_topic, const std::string& output_topic, const std::string& motor_topic);
         // Destructor (default)
         ~node_imu() override = default;

         void calibrate_imu()
         {
            calibrating_ = true;
            calibration_count_ = 0;
         }

     private:
        float calibration_offset_ = 0.0f;
        bool calibrating_ = true;
        float calibration_time_ = 3.0f; // seconds
        uint32_t calibration_count_ = 0;
        float calibration_values_[500] = {0.0f};
        float calibration_start_time_ = 0.0f;

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        // Subscriber member variable
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
     };
 }