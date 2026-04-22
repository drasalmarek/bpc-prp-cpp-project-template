
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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    auto lidar_control_node = std::make_shared<nodes::node_lidar_control>(
        "/bpc_prp_robot/lidar",
        "/bpc_prp_robot/lidar_control");

    auto fsm_node = std::make_shared<nodes::node_fsm>(
        "/bpc_prp_robot/lidar_control",
        "/bpc_prp_robot/wanted_speed",
        "/bpc_prp_robot/wanted_angle");

    auto imu_node = std::make_shared<nodes::node_imu>(
        "/bpc_prp_robot/imu",
        "/bpc_prp_robot/imu_angle");

    auto motor_node = std::make_shared<nodes::node_motor>(
        "/bpc_prp_robot/wanted_angle",
        "/bpc_prp_robot/wanted_speed",
        "/bpc_prp_robot/imu_angle",
        "/bpc_prp_robot/set_motor_speeds");

    auto pathfinder_node = std::make_shared<nodes::node_pathfinder>(
        "/bpc_prp_robot/lidar",
        "/bpc_prp_robot/wanted_speed",
        "/bpc_prp_robot/wanted_angle");

    // Add nodes to the executor
    //executor->add_node(lidar_control_node);
    executor->add_node(imu_node);
    executor->add_node(motor_node);
    executor->add_node(pathfinder_node);
    //executor->add_node(fsm_node);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}