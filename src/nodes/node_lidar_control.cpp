
#include <nodes/node_lidar_control.hpp>
#include <nodes/node_motor.hpp>

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

        float front_corner_values[2] = {0.0f, 0.0f};
        float front_corner_angles[2] = {180.0f, 180.0f};
        float front_angle = 0.0f;

        const float front_left_bound = -160 * M_PI / 180; // 160 fungovalo
        const float front_right_bound = 160 * M_PI / 180;

        const float left_left_bound = -120 * M_PI / 180;
        const float left_right_bound = -90 * M_PI / 180;

        const float right_left_bound = 90 * M_PI / 180;
        const float right_right_bound = 120 * M_PI / 180;

        uint32_t valid_front_count = 0;
        uint32_t valid_left_count = 0;
        uint32_t valid_right_count = 0;
        uint32_t valid_back_count = 0;

        for(size_t i = 0; i < msg->ranges.size(); ++i) 
        {
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            if (range > msg->range_min && range < msg->range_max) 
            {
                if (angle <= front_left_bound || angle >= front_right_bound)
                {
                    if((abs(angle) < abs(front_corner_angles[0])) && (angle < 0))
                    {
                        front_corner_angles[0] = angle;
                        front_corner_values[0] = range;
                    }
                    if((abs(angle) < abs(front_corner_angles[1])) && (angle > 0))
                    {
                        front_corner_angles[1] = angle;
                        front_corner_values[1] = range;
                    }

                    front_avg += range;
                    valid_front_count++;
                } 
                else if (angle >= left_left_bound && angle <= left_right_bound)
                {
                    left_avg += range;
                    valid_left_count++;
                } 
                else if (angle >= right_left_bound && angle <= right_right_bound)
                {
                    right_avg += range;
                    valid_right_count++;
                }
            }
        }

        front_avg /= (valid_front_count > 0) ? valid_front_count : 1;
        left_avg /= (valid_left_count > 0) ? valid_left_count : 1;
        right_avg /= (valid_right_count > 0) ? valid_right_count : 1;
        back_avg /= (valid_back_count > 0) ? valid_back_count : 1;

        // calculate front angle
        if (front_corner_values[0] > 0 && front_corner_values[1] > 0)
        {
            float left_x = front_corner_values[0] * std::cos(front_corner_angles[0]);
            float left_y = front_corner_values[0] * std::sin(front_corner_angles[0]);

            float right_x = front_corner_values[1] * std::cos(front_corner_angles[1]);
            float right_y = front_corner_values[1] * std::sin(front_corner_angles[1]);

            front_angle = std::atan2(right_y - left_y, right_x - left_x) * 180 / M_PI;
            front_angle = normalize_angle_degrees(front_angle);
        }
        else
        {
            front_angle = 90.0f;
        }

        RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f, Back: %.2f, vf: %u, vl: %u, vr: %u, vb: %u",
            front_avg, left_avg, right_avg, back_avg, valid_front_count, valid_left_count, valid_right_count, valid_back_count);

        auto output_msg = std_msgs::msg::Float32MultiArray();
        output_msg.data.push_back(front_avg);
        output_msg.data.push_back(left_avg);
        output_msg.data.push_back(right_avg);
        output_msg.data.push_back(back_avg);
        output_msg.data.push_back(front_angle);
        publisher_->publish(output_msg);
    }
}