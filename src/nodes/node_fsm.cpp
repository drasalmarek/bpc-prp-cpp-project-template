
#include <nodes/node_fsm.hpp>

enum class State : uint8_t {
    IDLE = 0,
    MOVING_FORWARD = 1,
    TURNING_LEFT = 2,
    TURNING_RIGHT = 3,
    DEAD_END = 4
};

namespace nodes
{
    node_fsm::node_fsm(const std::string& input_topic, const std::string& motor_topic)
        : Node("node_fsm")
    {
        // Initialize the subscriber
        subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            input_topic, 10, std::bind(&node_fsm::subscriber_callback, this, std::placeholders::_1));

        // Initialize the publisher
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(motor_topic, 10);
    }
    void node_fsm::subscriber_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        static State state = State::MOVING_FORWARD;

        float distance_front = msg->data[0];
        float distance_left = msg->data[1];
        float distance_right = msg->data[2];
        float distance_back = msg->data[3];

        auto motor_message = std_msgs::msg::UInt8MultiArray();

        const float FRONT_THRESHOLD = 0.2f; // Example threshold for obstacle detection
        const float LEFT_THRESHOLD = 1000.0f; // 0.44
        const float RIGHT_THRESHOLD = 1000.0f; // 0.44

        switch(state)
        {
            case State::IDLE:
            {
                // Handle IDLE state
                RCLCPP_INFO(this->get_logger(), "Idle");

                // Motor control
                motor_message.data = {128, 128}; // Stop motors
                publisher_->publish(motor_message);
                break;
            }
            case State::MOVING_FORWARD:
            {
                // Handle MOVING_FORWARD state

                RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f, Back: %.2f",
                    distance_front, distance_left, distance_right, distance_back);

                // State changes
                if(distance_front < FRONT_THRESHOLD) 
                {
                    state = State::DEAD_END;
                    break;
                }
                else if (distance_right > RIGHT_THRESHOLD && distance_left > LEFT_THRESHOLD) 
                {
                    state = State::MOVING_FORWARD;
                }
                else if (distance_right > RIGHT_THRESHOLD) 
                {
                    state = State::TURNING_RIGHT;
                    break;
                } 
                else if (distance_left > LEFT_THRESHOLD) 
                {
                    state = State::TURNING_LEFT;
                    break;
                }

                distance_left = std::min(distance_left, 0.44f);
                distance_right = std::min(distance_right, 0.44f);
                float error = distance_left - distance_right;
                float Kp = 50.0f; // Proportional gain (tune as needed)
                int base_speed = 145; // Base motor speed
                int speed_offset = static_cast<int>(Kp * error);
                speed_offset = std::max(-7, std::min(7, speed_offset));

                int speed_left = base_speed - speed_offset;
                int speed_right = base_speed + speed_offset;

                // Motor control
                motor_message.data = {static_cast<uint8_t>(speed_left), static_cast<uint8_t>(speed_right)};
                //motor_message.data = {135, 135}; // Move forward at base speed
                publisher_->publish(motor_message);

                break;
            }
            case State::TURNING_LEFT:
            {
                // Handle TURNING_LEFT state
                RCLCPP_INFO(this->get_logger(), "Turning Left");

                // Motor control
                motor_message.data = {128, 128}; // Stop motors
                publisher_->publish(motor_message);

                break;
            }
            case State::TURNING_RIGHT:
            {
                // Handle TURNING_RIGHT state
                RCLCPP_INFO(this->get_logger(), "Turning Right");

                // Motor control
                motor_message.data = {128, 128}; // Stop motors
                publisher_->publish(motor_message);

                break;
            }
            case State::DEAD_END:
            {
                // Handle DEAD_END state
                RCLCPP_INFO(this->get_logger(), "Dead End");

                // Motor control
                motor_message.data = {128, 128}; // Stop motors
                publisher_->publish(motor_message);

                break;
            }
            default:
            {
                // Motor control
                motor_message.data = {128, 128}; // Stop motors
                publisher_->publish(motor_message);    

                break;
            }
        }
    }
}