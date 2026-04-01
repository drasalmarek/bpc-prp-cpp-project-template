
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace nodes
{
     class node_lidar_control : public rclcpp::Node {
     public:
         // Constructor
         node_lidar_control(const std::string& lidar_topic, const std::string& output_topic);
         // Destructor (default)
         ~node_lidar_control() override = default;
 
     private:
        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        // Subscriber member variable
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
     };
 }