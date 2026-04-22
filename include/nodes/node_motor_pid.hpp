
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <algorithms/pid.hpp>

#define MOTOR_PID_NEUTRAL 128
#define MOTOR_PID_MAX_STEER 90.0f

#define MOTOR_PID_P 0.2f
#define MOTOR_PID_I 0.0f
#define MOTOR_PID_D 0.0f

namespace nodes
{
	class node_motor_pid : public rclcpp::Node {
	public:
		node_motor_pid(const std::string& wanted_angle_topic,
			const std::string& wanted_speed_topic,
			const std::string& output_topic);
		~node_motor_pid() override = default;

	private:
		float wanted_speed_ = 0.0f;
		float wanted_angle_relative_deg_ = 0.0f;

		uint8_t left_motor_ = MOTOR_PID_NEUTRAL;
		uint8_t right_motor_ = MOTOR_PID_NEUTRAL;

		rclcpp::Time last_control_stamp_;

		algorithms::pid pid_controller_{MOTOR_PID_P, MOTOR_PID_I, MOTOR_PID_D};

		rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wanted_angle_subscriber_;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wanted_speed_subscriber_;
		rclcpp::TimerBase::SharedPtr control_timer_;

		void wanted_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
		void wanted_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
		void control_step();

		static float normalize_angle_degrees(float angle_deg);
		static uint8_t clamp_motor(float command);
	};
}



