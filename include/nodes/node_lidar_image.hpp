#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace nodes
{
     class node_lidar_image : public rclcpp::Node {
     public:
         // Constructor
         node_lidar_image(const std::string& lidar_topic,
            const std::string& image_topic,
            const std::string& aruco_topic,
            const std::string& wanted_speed_topic,
            const std::string& wanted_angle_topic);
         // Destructor (default)
         ~node_lidar_image() override = default;
 
     private:
      int aruco_last_id_ = 2;

      rclcpp::Time last_crossroad_time_;
      rclcpp::Duration crossroad_cooldown_ = rclcpp::Duration::from_seconds(4.0);

      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
      rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aruco_subscriber_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_speed_publisher_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_angle_publisher_;
      // Callback function for the subscriber
      void aruco_callback(const std_msgs::msg::UInt8::SharedPtr msg);
      void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
     };
 }