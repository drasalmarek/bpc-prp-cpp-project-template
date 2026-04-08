
#include <nodes/node_fsm.hpp>

enum class State : uint8_t {
    CALIBRATING = 0,
    IDLE = 1,
    MOVING_FORWARD = 2,
    TURNING_LEFT = 3,
    TURNING_RIGHT = 4,
    DEAD_END = 5
};

namespace nodes
{
    node_fsm::node_fsm(const std::string& lidar_control_topic, 
        const std::string& wanted_speed_topic,
        const std::string& wanted_angle_topic)
        : Node("node_fsm")
    {
        // Initialize the subscriber
        subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            lidar_control_topic, 10, std::bind(&node_fsm::subscriber_callback, this, std::placeholders::_1));

        // Initialize the publisher
        wanted_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_speed_topic, 10);
        wanted_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_angle_topic, 10);
    }
    void node_fsm::subscriber_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        static State state = State::CALIBRATING;

        float distance_front = msg->data[0];
        float distance_left = msg->data[1];
        float distance_right = msg->data[2];
        float distance_back = msg->data[3];

        auto motor_message = std_msgs::msg::UInt8MultiArray();

        const float FRONT_THRESHOLD = 0.25f; // Example threshold for obstacle detection
        const float LEFT_THRESHOLD = 0.44f; // 0.44
        const float RIGHT_THRESHOLD = 0.44f; // 0.44

        switch(state)
        {
            case State::CALIBRATING:
            {
                std::this_thread::sleep_for(std::chrono::seconds(8));

                state = State::MOVING_FORWARD;

                break;
            }
            case State::IDLE:
            {
                // Handle IDLE state
                RCLCPP_INFO(this->get_logger(), "Idle");

                auto message = std_msgs::msg::Float32();
                message.data = 0.0f; // Stop motors
                wanted_speed_publisher_->publish(message);
                break;
            }
            case State::MOVING_FORWARD:
            {
                // Handle MOVING_FORWARD state

                RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f, Back: %.2f",
                    distance_front, distance_left, distance_right, distance_back);

                // State changes
                if (distance_front < FRONT_THRESHOLD) 
                {
                    if (distance_left > LEFT_THRESHOLD && distance_right < RIGHT_THRESHOLD) 
                    {
                        state = State::TURNING_LEFT;
                        break;
                    } 
                    else if (distance_left < LEFT_THRESHOLD && distance_right > RIGHT_THRESHOLD) 
                    {
                        state = State::TURNING_RIGHT;
                        break;
                    } 
                    else 
                    {
                        state = State::DEAD_END;
                        break;
                    }
                }

                if (distance_left < LEFT_THRESHOLD && distance_right < RIGHT_THRESHOLD) // Go straight according to lidar
                {
                    float distance_diff = distance_left - distance_right;
                    float angle_correction = distance_diff * 5.0f; // Proportional regulator
                    auto message = std_msgs::msg::Float32();
                    message.data = angle_correction;
                    wanted_angle_publisher_->publish(message);
                }

                auto message = std_msgs::msg::Float32();
                message.data = 15.0f; // Example speed value
                wanted_speed_publisher_->publish(message);

                break;
            }
            case State::TURNING_LEFT:
            {
                // Handle TURNING_LEFT state
                RCLCPP_INFO(this->get_logger(), "Turning Left");

                auto message = std_msgs::msg::Float32();
                message.data = 90.0f;
                wanted_angle_publisher_->publish(message);

                message = std_msgs::msg::Float32();
                message.data = 0.0f; // Stop motors
                wanted_speed_publisher_->publish(message);

                std::this_thread::sleep_for(std::chrono::seconds(2));

                state = State::MOVING_FORWARD; // After turning, go back to moving forward

                break;
            }
            case State::TURNING_RIGHT:
            {
                // Handle TURNING_RIGHT state
                RCLCPP_INFO(this->get_logger(), "Turning Right");

                auto message = std_msgs::msg::Float32();
                message.data = -90.0f;
                wanted_angle_publisher_->publish(message);

                message = std_msgs::msg::Float32();
                message.data = 0.0f; // Stop motors
                wanted_speed_publisher_->publish(message);

                std::this_thread::sleep_for(std::chrono::seconds(2));

                state = State::MOVING_FORWARD; // After turning, go back to moving forward

                break;
            }
            case State::DEAD_END:
            {
                // Handle DEAD_END state
                RCLCPP_INFO(this->get_logger(), "Dead End");

                state = State::IDLE;

                break;
            }
            default:
            {
                state = State::IDLE; 

                break;
            }
        }
    }
}