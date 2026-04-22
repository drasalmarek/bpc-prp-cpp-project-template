#include <nodes/node_motor_pid.hpp>

#include <algorithm>
#include <cmath>

namespace nodes
{
	node_motor_pid::node_motor_pid(const std::string& wanted_angle_topic,
		const std::string& wanted_speed_topic,
		const std::string& output_topic)
		: Node("node_motor_pid")
	{
		motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(output_topic, 10);

		wanted_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
			wanted_angle_topic,
			rclcpp::SensorDataQoS(),
			std::bind(&node_motor_pid::wanted_angle_callback, this, std::placeholders::_1));

		wanted_speed_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
			wanted_speed_topic,
			rclcpp::SensorDataQoS(),
			std::bind(&node_motor_pid::wanted_speed_callback, this, std::placeholders::_1));

		last_control_stamp_ = this->get_clock()->now();

		control_timer_ = this->create_wall_timer(
			std::chrono::milliseconds(20),
			std::bind(&node_motor_pid::control_step, this));
	}

	float node_motor_pid::normalize_angle_degrees(float angle_deg)
	{
		float normalized = std::remainder(angle_deg, 360.0f);
		if (normalized == -180.0f) {
			normalized = 180.0f;
		}
		return normalized;
	}

	uint8_t node_motor_pid::clamp_motor(float command)
	{
		return static_cast<uint8_t>(std::clamp(static_cast<int>(std::round(command)), 0, 255));
	}

	void node_motor_pid::wanted_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
	{
		wanted_angle_relative_deg_ = normalize_angle_degrees(msg->data);
	}

	void node_motor_pid::wanted_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
	{
		wanted_speed_ = msg->data;
	}

	void node_motor_pid::control_step()
	{
		const rclcpp::Time now = this->get_clock()->now();
		float dt = (now - last_control_stamp_).seconds();
		last_control_stamp_ = now;

		if (dt <= 0.0f) {
			dt = 0.02f;
		}

		const float angle_error = normalize_angle_degrees(wanted_angle_relative_deg_);
		float steer = pid_controller_.step(angle_error, dt);
		steer = std::clamp(steer, -MOTOR_PID_MAX_STEER, MOTOR_PID_MAX_STEER);

		const float base = static_cast<float>(MOTOR_PID_NEUTRAL) + wanted_speed_;
		left_motor_ = clamp_motor(base - steer);
		right_motor_ = clamp_motor(base + steer);

		auto message = std_msgs::msg::UInt8MultiArray();
		message.data = {left_motor_, right_motor_};
		motor_publisher_->publish(message);
	}
}
