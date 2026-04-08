
#include "nodes/node_motor.hpp"
#include <cmath>

namespace
{
    float normalize_angle_degrees(float angle)
    {
        float normalized = std::remainder(angle, 360.0f);
        if (normalized == -180.0f) {
            normalized = 180.0f;
        }
        return normalized;
    }
}

namespace nodes
{
    node_motor::node_motor(const std::string& wanted_angle_topic, 
        const std::string& wanted_speed_topic,
        const std::string& imu_angle_topic, 
        const std::string& output_topic)
        : Node("node_motor")
    {
        // Initialize publisher
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(output_topic, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() {
                auto message = std_msgs::msg::UInt8MultiArray();
                // Fill the message with dummy motor speed data (for example)
                message.data = {motor_speed[0], motor_speed[1]}; // Example motor speeds
                publisher_->publish(message);
                //RCLCPP_INFO(this->get_logger(), "Published motor speeds: [%d, %d]", motor_speed[0], motor_speed[1]);
            }
        );

        // Initialize subscriber for wanted speed
        subscriber_wanted_speed_ = this->create_subscription<std_msgs::msg::Float32>(
            wanted_speed_topic, rclcpp::SensorDataQoS(),
            std::bind(&node_motor::wanted_speed_callback, this, std::placeholders::_1)
        );

        // Initialize subscriber for wanted angle
        subscriber_wanted_angle_ = this->create_subscription<std_msgs::msg::Float32>(
            wanted_angle_topic, rclcpp::SensorDataQoS(),
            std::bind(&node_motor::wanted_angle_callback, this, std::placeholders::_1)
        );

        // Initialize subscriber for IMU angle
        subscriber_imu_angle_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            imu_angle_topic, rclcpp::SensorDataQoS(),
            std::bind(&node_motor::imu_angle_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "node_motor subscribed to '%s' with SensorDataQoS",
            wanted_angle_topic.c_str());

    }

    void node_motor::wanted_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        wanted_speed_ = msg->data;
    }

    void node_motor::wanted_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        wanted_angle_ += msg->data;
        wanted_angle_ = normalize_angle_degrees(wanted_angle_);
    }


    void node_motor::imu_angle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        float imu_angle = normalize_angle_degrees(msg->data[0]);
        float angle_error = normalize_angle_degrees(wanted_angle_ - imu_angle);

        // Update PID controller and compute motor speed adjustments
        float pid_output = pid_controller_.step(angle_error, msg->data[1]); 

        pid_output = std::min(pid_output, 70.0f);
        pid_output = std::max(pid_output, -70.0f);

        motor_speed[0] = static_cast<uint8_t>(std::min(std::max(static_cast<int>(128 + wanted_speed_ - pid_output), 0), 255));
        motor_speed[1] = static_cast<uint8_t>(std::min(std::max(static_cast<int>(128 + wanted_speed_ + pid_output), 0), 255));
        
    }
}