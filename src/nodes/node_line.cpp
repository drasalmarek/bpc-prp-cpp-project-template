
#include "nodes/node_line.hpp"

namespace nodes
{
    node_line::node_line(const std::string& line_sensor_topic, const std::string& output_topic) : Node("line_node")
    {
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            line_sensor_topic, 10, std::bind(&node_line::on_line_sensors_msg, this, std::placeholders::_1));
        output_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            output_topic, 10);
    }

    node_line::~node_line() = default;

    void node_line::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received line sensor data with insufficient size");
            return;
        }

        float left_value = static_cast<float>(msg->data[0]);
        float right_value = static_cast<float>(msg->data[1]);
        left_value = (left_value - LINE_SENSOR_LEFT_MIN) / (LINE_SENSOR_LEFT_MAX - LINE_SENSOR_LEFT_MIN); // Normalize to [0, 1]
        right_value = (right_value - LINE_SENSOR_RIGHT_MIN) / (LINE_SENSOR_RIGHT_MAX - LINE_SENSOR_RIGHT_MIN); // Normalize to [0, 1]

        float continuous_pose = estimate_continuous_line_pose(left_value, right_value);
        DiscreteLinePose discrete_pose = estimate_discrete_line_pose(left_value, right_value);

        RCLCPP_INFO(this->get_logger(), "Continuous Line Pose: %.2f, Discrete Line Pose: %d",
                    continuous_pose, static_cast<int>(discrete_pose));

        auto message = std_msgs::msg::Float32();
        // Fill the message with dummy motor speed data (for example)
        message.data = continuous_pose; // Example motor speed
        output_publisher_->publish(message);
    }

    float node_line::estimate_continuous_line_pose(float left_value, float right_value)
    {
        // Simple linear estimation of line pose based on sensor values
        float total = left_value + right_value;
        if (total == 0) {
            return 0.0f; // No line detected
        }
        return (right_value - left_value) / total; // Range: [-1, 1]
    }

    DiscreteLinePose node_line::estimate_discrete_line_pose(float l_norm, float r_norm)
    {
        if (l_norm > LINE_THRESHOLD && r_norm > LINE_THRESHOLD) {
            return DiscreteLinePose::LineBoth;
        } else if (l_norm > LINE_THRESHOLD) {
            return DiscreteLinePose::LineOnLeft;
        } else if (r_norm > LINE_THRESHOLD) {
            return DiscreteLinePose::LineOnRight;
        } else {
            return DiscreteLinePose::LineNone;
        }
    }
}