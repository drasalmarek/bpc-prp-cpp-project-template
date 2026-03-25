
#include "nodes/node_lidar_image.hpp"

namespace nodes
{
    node_lidar_image::node_lidar_image(const std::string& lidar_topic,
        const std::string& image_topic)
        : Node("node_lidar_image")
    {
        // Initialize the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
        // Initialize the subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&node_lidar_image::subscriber_callback, this, std::placeholders::_1));
    }
    void node_lidar_image::subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a new Image message
        auto image_msg = sensor_msgs::msg::Image();
        image_msg.header.stamp = this->get_clock()->now();
        image_msg.header.frame_id = "lidar_frame";
        image_msg.height = 1024;
        image_msg.width = 1024;
        image_msg.encoding = "mono8";
        image_msg.is_bigendian = false;
        image_msg.step = image_msg.width;
        image_msg.data.resize(image_msg.height * image_msg.step, 0);

        // Convert LaserScan to top-down corridor view
        float max_range = msg->range_max;
        float image_scale = 1024.0f / (0.3f * max_range);
        int center = 512;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            if (range > msg->range_min && range < msg->range_max) {
                int x = static_cast<int>(center + range * std::cos(angle) * image_scale);
                int y = static_cast<int>(center + range * std::sin(angle) * image_scale);

                if (x >= 0 && x < static_cast<int>(image_msg.width) &&
                    y >= 0 && y < static_cast<int>(image_msg.height)) {
                    image_msg.data[y * image_msg.step + x] = 255;
                }
            }
        }

        // Publish the Image message
        publisher_->publish(image_msg);
    }
}