// Public API sketch; adapt to your project
enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineNode : public rclcpp::Node {
public:
    LineNode();
    ~LineNode();

    // Relative pose to line [m]
    float get_continuous_line_pose() const;

    DiscreteLinePose get_discrete_line_pose() const;

private:
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

    void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr& msg);

    float estimate_continuous_line_pose(float left_value, float right_value);

    DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
};
