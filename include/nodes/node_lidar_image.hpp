#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace nodes
{
     class node_lidar_image : public rclcpp::Node {
     public:
         // Constructor
         node_lidar_image(const std::string& lidar_topic,
            const std::string& image_topic);
         // Destructor (default)
         ~node_lidar_image() override = default;
 
     private:
        // Publisher member variable
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        // Subscriber member variable
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
     };
 }