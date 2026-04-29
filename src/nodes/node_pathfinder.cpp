
#include <nodes/node_pathfinder.hpp>
#include <cmath>

#define DETECTION_RADIUS (0.45f) // 0.5f
#define DETECTION_RADIUS_HYSTERESIS (0.05f) // 0.1f

#define OBSTACLE (1)
#define NO_OBSTACLE (0)

namespace nodes
{
    node_pathfinder::node_pathfinder(const std::string& lidar_topic, 
        const std::string& wanted_speed_topic, 
        const std::string& wanted_angle_topic)
        : Node("node_pathfinder")
    {
        // Initialize the publisher
        wanted_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_speed_topic, 10);
        wanted_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_angle_topic, 10);

        // Initialize the subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&node_pathfinder::subscriber_callback, this, std::placeholders::_1));
        // initialize last_crossroad_time_ with node's clock to ensure same time source
        last_crossroad_time_ = this->now() - crossroad_cooldown_;
    }

    void node_pathfinder::subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // create a boolean array to store whether an obstacle is detected in each direction
        // false = no obstacle, true = obstacle
        bool obstacle_map[msg->ranges.size()];

        for(size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float distance = msg->ranges[i];
            if (distance < DETECTION_RADIUS - DETECTION_RADIUS_HYSTERESIS) 
            {
                obstacle_map[i] = OBSTACLE;
            } 
            else if (distance > DETECTION_RADIUS + DETECTION_RADIUS_HYSTERESIS) 
            {
                obstacle_map[i] = NO_OBSTACLE;
            } 
            else
            {
                if(i > 0)
                {
                    obstacle_map[i] = obstacle_map[i - 1];
                }
                else
                {
                    obstacle_map[i] = NO_OBSTACLE;
                }
            }
        }

        // invert direction of the obstacle map so that front is in the middle of the array and left and right are on the sides
        bool inverted_obstacle_map[msg->ranges.size()];
        for(size_t i = 0; i < msg->ranges.size(); ++i)
        {
            inverted_obstacle_map[i] = obstacle_map[(i + msg->ranges.size() / 2) % msg->ranges.size()];
        }  
        std::copy(inverted_obstacle_map, inverted_obstacle_map + msg->ranges.size(), obstacle_map);

        // delete singularities in the obstacle map
        for(size_t i = 1; i < msg->ranges.size() - 1; ++i)
        {
            if(obstacle_map[i] && !obstacle_map[i - 1] && !obstacle_map[i + 1])
            {
                obstacle_map[i] = NO_OBSTACLE;
            }
            else if(!obstacle_map[i] && obstacle_map[i - 1] && obstacle_map[i + 1])
            {
                obstacle_map[i] = OBSTACLE;
            }
        }

        // find centres of gaps in the obstacle map
        std::vector<size_t> gap_centres;
        bool last_value = obstacle_map[0];
        size_t gap_start = 0; 

        for(size_t i = 1; i < msg->ranges.size() - 1; ++i)
        {
            if(obstacle_map[i] != last_value)
            {
                if(last_value == NO_OBSTACLE)
                {
                    if(i >= gap_start + 30) // only consider gaps that are at least 15 indices wide
                    {
                        gap_centres.push_back((gap_start + i) / 2);
                    }
                }
                else
                {
                    gap_start = i;
                }
                last_value = obstacle_map[i];
            }
        }

        // delete gap centres that are in the back
        gap_centres.erase(std::remove_if(gap_centres.begin(), gap_centres.end(), [&](size_t index) {
            return std::abs((int)index - (int)(msg->ranges.size() / 2)) > 95 / (msg->angle_increment * 180.0f / static_cast<float>(M_PI)); // only consider gaps that are within +-95 degrees of the front
        }), gap_centres.end());

        // find the gap centre closest to the front
        size_t front_index = msg->ranges.size() / 2;
        if(gap_centres.empty())
        {
            auto message_speed = std_msgs::msg::Float32();
            message_speed.data = 0.0f;
            wanted_speed_publisher_->publish(message_speed);

            auto message_angle = std_msgs::msg::Float32();
            message_angle.data = 0.0f;
            wanted_angle_publisher_->publish(message_angle);

            RCLCPP_WARN(this->get_logger(), "No valid gap found, stopping robot.");
            return;
        }

        size_t closest_gap_centre = gap_centres[0];
        for(size_t i = 1; i < gap_centres.size(); ++i)
        {
            if(std::abs((int)gap_centres[i] - (int)front_index) < std::abs((int)closest_gap_centre - (int)front_index))
            {
                closest_gap_centre = gap_centres[i];
            }
        }

        std::string obstacle_map_str;
        for(size_t i = 0; i < msg->ranges.size(); ++i)
        {
            obstacle_map_str += (obstacle_map[i] == OBSTACLE) ? "1" : "0";
            if(gap_centres.size() > 0 && i == closest_gap_centre)
            {
                obstacle_map_str += "X"; // mark the closest gap centre with an X
            }
            else if(std::find(gap_centres.begin(), gap_centres.end(), i) != gap_centres.end())
            {
                obstacle_map_str += "G"; // mark other gap centres with a G
            }
        }
        //RCLCPP_INFO(this->get_logger(), "Obstacle map: %s", obstacle_map_str.c_str());
        std::string gap_centres_str;
        std::vector<float> gap_centres_angles; 

        for(size_t i = 0; i < gap_centres.size(); ++i)
        {
            gap_centres_angles.push_back(gap_centres[i] * msg->angle_increment * 180.0f / static_cast<float>(M_PI) - 180);
            gap_centres_str += std::to_string(gap_centres_angles[i]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Gap centres: %s", gap_centres_str.c_str()); 



        // === find crossroads ===
        static int num_crossroad_detects = 0;

        // If we're within the cooldown window since the last detected crossroad,
        // skip detection to avoid duplicate detections.
        if ((this->now() - last_crossroad_time_) < crossroad_cooldown_)
        {
            // still cooling down; reset consecutive detection counter
            num_crossroad_detects = 0;
        }
        else
        {
            if(gap_centres.size() >= 2)
            {
                ++num_crossroad_detects;

                if(num_crossroad_detects >= 5) // only consider it a crossroad if it is detected for 5 consecutive scans
                {
                    num_crossroad_detects = 0;
                    RCLCPP_INFO(this->get_logger(), "Crossroad detected!");

                    // find the gap centre on the right (lowest angle)
                    float right_gap_angle = gap_centres_angles[0];
                    for(size_t i = 1; i < gap_centres.size(); ++i)
                    {
                        if(gap_centres_angles[i] < right_gap_angle)
                        {
                            right_gap_angle = gap_centres_angles[i];
                        }
                    }

                    auto message_speed = std_msgs::msg::Float32();
                    message_speed.data = 0.0f;
                    wanted_speed_publisher_->publish(message_speed);

                    auto message_angle = std_msgs::msg::Float32();
                    message_angle.data = right_gap_angle;
                    wanted_angle_publisher_->publish(message_angle);

                    std::this_thread::sleep_for(std::chrono::milliseconds(600)); // wait for the robot to turn

                    // record the time of this detection so we can ignore subsequent
                    // detections for a short cooldown period instead of blocking the thread
                    last_crossroad_time_ = this->now();
                }
            }
            else
            {
                num_crossroad_detects = 0;
            }
        }
        // === end of crossroad detection ===


        // calculate the angle to the closest gap centre
        float angle_increment = msg->angle_increment;
        float angle_to_gap_centre = ((int)closest_gap_centre - (int)front_index) * angle_increment;
        
        auto message_speed = std_msgs::msg::Float32();
        message_speed.data = 15.0f;
        wanted_speed_publisher_->publish(message_speed);

        auto message_angle = std_msgs::msg::Float32();
        message_angle.data = angle_to_gap_centre * 180.0f / static_cast<float>(M_PI);
        wanted_angle_publisher_->publish(message_angle);
    }
}