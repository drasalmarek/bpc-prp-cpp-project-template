
#include "nodes/node_motor.hpp"

namespace nodes
{
    node_motor::node_motor(const std::string& line_estimation_topic, const std::string& output_topic)
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
                RCLCPP_INFO(this->get_logger(), "Published motor speeds: [%d, %d]", motor_speed[0], motor_speed[1]);
            }
        );

        // Initialize subscriber for line sensor data
        subscriber_line_estimation_ = this->create_subscription<std_msgs::msg::Float32>(
            line_estimation_topic, rclcpp::SensorDataQoS(),
            std::bind(&node_motor::line_estimation_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "node_motor subscribed to '%s' with SensorDataQoS",
            line_estimation_topic.c_str());

    }

    void node_motor::line_estimation_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        static rclcpp::Time last_time = this->now();
        rclcpp::Time current_time = this->now();
        float dt = (current_time - last_time).seconds();
        last_time = current_time;

        float pid_out = pid_controller_.step(msg->data, dt);

        uint8_t speed_offset = static_cast<uint8_t>(MOTOR_MAX_OFFSET * pid_out);
        motor_speed[0] = MOTOR_DEFAULT_SPEED + speed_offset; // Adjust left motor speed
        motor_speed[1] = MOTOR_DEFAULT_SPEED - speed_offset; // Adjust right motor speed
        RCLCPP_INFO(this->get_logger(), "Adjusted motor speeds: [%d, %d]", motor_speed[0], motor_speed[1]);
    }
}