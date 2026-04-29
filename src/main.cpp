
#include <rclcpp/rclcpp.hpp>
#include "nodes/node_publisher.hpp"
#include "nodes/node_subscriber.hpp"
#include "nodes/node_gain.hpp"
#include "nodes/node_motor.hpp"
#include "nodes/node_line.hpp"
#include "nodes/node_lidar_image.hpp"
#include "nodes/node_lidar_control.hpp"
#include "nodes/node_fsm.hpp"
#include "nodes/node_imu.hpp"
#include "nodes/node_pathfinder.hpp"
#include "nodes/node_motor_pid.hpp"
#include "nodes/node_qr_code.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto pathfinder_node = std::make_shared<nodes::node_pathfinder>(
        "/bpc_prp_robot/lidar",
        "/bpc_prp_robot/wanted_speed",
        "/bpc_prp_robot/wanted_angle");

    auto motor_pid_node = std::make_shared<nodes::node_motor_pid>(
        "/bpc_prp_robot/wanted_angle",
        "/bpc_prp_robot/wanted_speed",
        "/bpc_prp_robot/set_motor_speeds");

    auto qr_code_node = std::make_shared<nodes::node_qr_code>(
        "/bpc_prp_robot/camera",
        "/bpc_prp_robot/qr_code_id");

    // Add nodes to the executor
    //executor->add_node(motor_pid_node);
    //executor->add_node(pathfinder_node);
    executor->add_node(qr_code_node);


    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}