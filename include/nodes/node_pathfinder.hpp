
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nodes/node_motor.hpp>

namespace nodes
{
     class node_pathfinder : public rclcpp::Node {
     public:
         // Constructor
         node_pathfinder(const std::string& lidar_topic, 
            const std::string& aruco_topic,
            const std::string& wanted_speed_topic, 
            const std::string& wanted_angle_topic);
         // Destructor (default)
         ~node_pathfinder() override = default;
 
     private:
      float angle_ = 0.0;
      rclcpp::Time last_crossroad_time_;
      rclcpp::Duration crossroad_cooldown_ = rclcpp::Duration::from_seconds(2.0);

      uint8_t aruco_last_id_ = 2;

        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_speed_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wanted_angle_publisher_;

        // Lidar subscriber member variable
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
        void lidar_subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        // Aruco subscriber member variable
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aruco_subscriber_;
        void aruco_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg);
     };
 }