
#include "nodes/node_motor.hpp"

namespace nodes
{
    node_motor::node_motor(const std::string& output_topic, const std::string& line_sensor_topic)
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
            }
        );

        // Initialize subscriber for line sensor data
        subscriber_line_sensor_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            line_sensor_topic, rclcpp::SensorDataQoS(),
            std::bind(&node_motor::line_sensor_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "node_motor subscribed to '%s' with SensorDataQoS",
            line_sensor_topic.c_str());

    }

    void node_motor::line_sensor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        // Log received line sensor data
        if((msg->data[0] > LINE_THRESHOLD && msg->data[1] > LINE_THRESHOLD) || (msg->data[0] < LINE_THRESHOLD && msg->data[1] < LINE_THRESHOLD))
        {
            line_pos = 0; // center or unknown
        } 
        else if (msg->data[0] > LINE_THRESHOLD)
        {
            line_pos = -1; // left
        } 
        else if (msg->data[1] > LINE_THRESHOLD) 
        {
            line_pos = 1; // right
        }

        switch(line_pos)
        {
            case -1:
                motor_speed[0] = MOTOR_IDLE_SPEED;
                motor_speed[1] = MOTOR_MAX_SPEED;
                break;
            case 0:
                motor_speed[0] = MOTOR_MAX_SPEED;
                motor_speed[1] = MOTOR_MAX_SPEED;
                break;
            case 1:
                motor_speed[0] = MOTOR_MAX_SPEED;
                motor_speed[1] = MOTOR_IDLE_SPEED;
                break;
        }

        RCLCPP_INFO(this->get_logger(), "Line Sensor: %u, %u, Line Position: %d", msg->data[0], msg->data[1], line_pos);
    }
}