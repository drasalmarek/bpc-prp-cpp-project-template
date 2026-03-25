
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

/* orange
#define LINE_SENSOR_LEFT_MAX 630
#define LINE_SENSOR_RIGHT_MAX 750
#define LINE_SENSOR_LEFT_MIN 10
#define LINE_SENSOR_RIGHT_MIN 30
*/

#define LINE_SENSOR_LEFT_MAX 900
#define LINE_SENSOR_RIGHT_MAX 900
#define LINE_SENSOR_LEFT_MIN 0
#define LINE_SENSOR_RIGHT_MIN 0

#define LINE_THRESHOLD 0.5f

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

namespace nodes
{
    class node_line : public rclcpp::Node {
    public:
        node_line(const std::string& line_sensor_topic, const std::string& output_topic);
        ~node_line();

        // Relative pose to line [m]
        float get_continuous_line_pose() const;

        DiscreteLinePose get_discrete_line_pose() const;

    private:
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_publisher_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
    };
}
