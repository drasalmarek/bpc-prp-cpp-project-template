
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nodes/node_motor.hpp>

namespace nodes
{
     class node_pathfinder : public rclcpp::Node {
     public:
         // Constructor
         node_pathfinder(const std::string& lidar_topic, 
            const std::string& wanted_speed_topic, 
            const std::string& wanted_angle_topic);
         // Destructor (default)
         ~node_pathfinder() override = default;
 
     private:
        float angle_ = 0.0;

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_speed_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_angle_publisher_;

        // Subscriber member variable
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
     };
 }