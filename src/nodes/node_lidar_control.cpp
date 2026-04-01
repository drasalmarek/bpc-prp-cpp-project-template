
#include <nodes/node_lidar_control.hpp>

namespace nodes
{
    node_lidar_control::node_lidar_control(const std::string& lidar_topic, const std::string& output_topic)
        : Node("node_lidar_control")
    {
        // Initialize the subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&node_lidar_control::subscriber_callback, this, std::placeholders::_1));

        // Initialize the publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic, 10);
    }
    void node_lidar_control::subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float front_avg = 0.0f;
        float left_avg = 0.0f;
        float right_avg = 0.0f;
        float back_avg = 0.0f;

        const float front_left_bound = -150 * M_PI / 180;
        const float front_right_bound = 150 * M_PI / 180;
        const float back_left_bound = -70 * M_PI / 180;
        const float back_right_bound = 70 * M_PI / 180;

        for(size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            if (range > msg->range_min && range < msg->range_max) {
                if (angle < back_right_bound && angle > back_left_bound) {
                    back_avg += range;
                } else if (angle >= back_right_bound && angle < front_right_bound) {
                    right_avg += range;
                } else if (angle <= back_left_bound && angle > front_left_bound) {
                    left_avg += range;
                } else {
                    front_avg += range;
                }
            }
        }

        front_avg /= msg->ranges.size() / 4;
        left_avg /= msg->ranges.size() / 4;
        right_avg /= msg->ranges.size() / 4;
        back_avg /= msg->ranges.size() / 4;

        /*
        RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f, Back: %.2f",
            front_avg, left_avg, right_avg, back_avg);
        */

        auto output_msg = std_msgs::msg::Float32MultiArray();
        output_msg.data.push_back(front_avg);
        output_msg.data.push_back(left_avg);
        output_msg.data.push_back(right_avg);
        output_msg.data.push_back(back_avg);
        publisher_->publish(output_msg);
    }
}