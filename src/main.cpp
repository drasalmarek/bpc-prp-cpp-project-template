
#include <rclcpp/rclcpp.hpp>
#include "nodes/node_publisher.hpp"
#include "nodes/node_subscriber.hpp"
#include "nodes/node_gain.hpp"
#include "nodes/node_motor.hpp"
#include "nodes/node_line.hpp"

// hello nevim pouzivat git tak testuju

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto line_node = std::make_shared<nodes::node_line>(
        "/bpc_prp_robot/line_sensors", 
        "/bpc_prp_robot/line_pose");

    auto motor_node = std::make_shared<nodes::node_motor>(
        "/bpc_prp_robot/line_pose", 
        "/bpc_prp_robot/set_motor_speeds");

    // Add nodes to the executor
    executor->add_node(line_node);
    executor->add_node(motor_node);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}