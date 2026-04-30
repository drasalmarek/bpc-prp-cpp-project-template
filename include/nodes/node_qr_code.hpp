
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <algorithms/qr_code.hpp>
#include <cv_bridge/cv_bridge.h>

namespace nodes
{
     class node_qr_code : public rclcpp::Node {
     public:
         // Constructor
         node_qr_code(const std::string& camera_topic, 
            const std::string& output_topic);
         // Destructor (default)
         ~node_qr_code() override = default;
 
     private:
        // Publisher member variable
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr output_publisher_;

        // Subscriber member variable
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
        // Callback function for the subscriber
        void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg);
     };
 }