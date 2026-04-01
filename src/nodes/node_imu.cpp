
#include <nodes/node_imu.hpp>

namespace nodes
{
    node_imu::node_imu(const std::string& imu_topic, const std::string& output_topic, const std::string& motor_topic)
        : Node("node_imu")
    {
        // Initialize the subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&node_imu::subscriber_callback, this, std::placeholders::_1));

        // Initialize the publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float32>(output_topic, 10);

        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(motor_topic, 10);
    }

    void node_imu::subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(calibrating_)
        {
            if(calibration_count_ == 0)
            {
                calibration_start_time_ = static_cast<float>(msg->header.stamp.sec) + static_cast<float>(msg->header.stamp.nanosec) * 1e-9f;
                RCLCPP_INFO(this->get_logger(), "Starting IMU calibration...");
            }

            if(calibration_count_ >= 500 || (static_cast<float>(msg->header.stamp.sec) + static_cast<float>(msg->header.stamp.nanosec) * 1e-9f) - calibration_start_time_ >= calibration_time_)
            {
                calibrating_ = false;
                calibration_offset_ = 0.0f;
                for(uint32_t i = 0; i < calibration_count_; ++i) {
                    calibration_offset_ += calibration_values_[i];
                }
                calibration_offset_ /= static_cast<float>(calibration_count_);

                RCLCPP_INFO(this->get_logger(), "IMU Calibration complete. Offset: %.4f", calibration_offset_);
            }
            else
            {
                calibration_values_[calibration_count_++] = msg->angular_velocity.z;
            }
        }

        // time estimation
        //RCLCPP_INFO(this->get_logger(), "current time sec: %lf, current time nanosec: %f", static_cast<double>(msg->header.stamp.sec), static_cast<double>(msg->header.stamp.nanosec) / 1e9f);
        static double previous_time = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        double current_time = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        double dt = current_time - previous_time;
        previous_time = current_time;

        // angle estimation
        float angular_velocity_z = msg->angular_velocity.z - calibration_offset_;
        static float previous_angle_z = 0.0f;
        float angle = previous_angle_z + angular_velocity_z * dt;
        previous_angle_z = angle;

        auto output_msg = std_msgs::msg::Float32();
        output_msg.data = angle;
        publisher_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "IMU Angle: %.4f, angular velocity: %.4f, dt: %.4f", angle, msg->angular_velocity.z, dt*1000.0f);


        float error = angle;
        float Kp = 20.0f; // Proportional gain (tune as needed)
        int base_speed = 145; // Base motor speed
        int speed_offset = static_cast<int>(Kp * error);
        speed_offset = std::max(-7, std::min(7, speed_offset));

        auto motor_message = std_msgs::msg::UInt8MultiArray();
        motor_message.data = {static_cast<uint8_t>(base_speed + speed_offset), static_cast<uint8_t>(base_speed - speed_offset)}; // Stop motors
        motor_publisher_->publish(motor_message);
    }

}