
#include <rclcpp/rclcpp.hpp>
#include "nodes/node_publisher.hpp"
#include "nodes/node_subscriber.hpp"
#include "nodes/node_gain.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    const std::string sensor_topic = "sensor_data";
    const std::string processed_topic = "processed_data";
    const uint8_t gain_factor = 2;

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto node_p = std::make_shared<nodes::node_publisher>(sensor_topic);
    auto node_g = std::make_shared<nodes::node_gain>(sensor_topic, processed_topic, gain_factor);
    auto node_s = std::make_shared<nodes::node_subscriber>(processed_topic);

    // Add nodes to the executor
    executor->add_node(node_p);
    executor->add_node(node_g);
    executor->add_node(node_s);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}