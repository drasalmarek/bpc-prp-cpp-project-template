
#include <nodes/node_pathfinder.hpp>
#include <cmath>

#define DETECTION_RADIUS (0.4f)
#define DETECTION_RADIUS_HYSTERESIS (0.1f) // 0.1f

#define DETECTION_RADIUS_2 (0.75f)
#define DETECTION_RADIUS_HYSTERESIS_2 (0.1f) // 0.1f

#define MIN_GAP_WIDTH_INDEXES (50)

#define OBSTACLE (1)
#define NO_OBSTACLE (0)

std::vector<float> gap_centres_angles(const sensor_msgs::msg::LaserScan::SharedPtr msg, const float& detection_radius, const float& detection_radius_hysteresis)
{
    // create a boolean array to store whether an obstacle is detected in each direction
    // false = no obstacle, true = obstacle
    bool obstacle_map[msg->ranges.size()];

    for(size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float distance = msg->ranges[i];
        if (distance < detection_radius - detection_radius_hysteresis) 
        {
            obstacle_map[i] = OBSTACLE;
        } 
        else if (distance > detection_radius + detection_radius_hysteresis) 
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
                if(i >= gap_start + MIN_GAP_WIDTH_INDEXES) // only consider gaps that are at least MIN_GAP_WIDTH_INDEXES indices wide
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

    // convert gap centres to angles
    std::vector<float> gap_centres_angles;
    for(const auto& gap_centre : gap_centres)
    {
        gap_centres_angles.push_back(((float)gap_centre) * msg->angle_increment * 180.0f / static_cast<float>(M_PI) - 180.0f);
    }

    // delete gap centres that are in the back
    gap_centres_angles.erase(std::remove_if(gap_centres_angles.begin(), gap_centres_angles.end(), [](float angle) {
        return std::abs(angle) > 95.0f; // only consider gaps that are within +-95 degrees of the front
    }), gap_centres_angles.end());

    return gap_centres_angles;
}

namespace nodes
{
    node_pathfinder::node_pathfinder(const std::string& lidar_topic, 
        const std::string& aruco_topic,
        const std::string& wanted_speed_topic, 
        const std::string& wanted_angle_topic)
        : Node("node_pathfinder")
    {
        // Initialize the publisher
        wanted_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_speed_topic, 10);
        wanted_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_angle_topic, 10);

        // Initialize the subscriber
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&node_pathfinder::lidar_subscriber_callback, this, std::placeholders::_1));

        aruco_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            aruco_topic, 10, std::bind(&node_pathfinder::aruco_subscriber_callback, this, std::placeholders::_1));
        
        // initialize last_crossroad_time_ with node's clock to ensure same time source
        last_crossroad_time_ = this->now() - crossroad_cooldown_;
    }

    void node_pathfinder::aruco_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        uint8_t aruco_id = static_cast<uint8_t>(msg->data);

        //if((aruco_id > 2) && (aruco_id < 10 || aruco_id > 12))
        if(aruco_id > 2)
        {
            return;
        }

        if((aruco_id / 10) >= (aruco_last_id_ / 10))
        {
            aruco_last_id_ = aruco_id;
        }
    }

    void node_pathfinder::lidar_subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<float> gap_centres_close = gap_centres_angles(msg, DETECTION_RADIUS, DETECTION_RADIUS_HYSTERESIS);
        std::vector<float> gap_centres_far = gap_centres_angles(msg, DETECTION_RADIUS_2, DETECTION_RADIUS_HYSTERESIS_2);

        // find the gap centre from both sets closest to the front (angles are in degrees)
        if(gap_centres_close.empty() && gap_centres_far.empty())
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

        const float front_angle = 0.0f; // degrees

        float closest_gap_centre = gap_centres_close.empty() ? gap_centres_far[0] : gap_centres_close[0];
        for(size_t i = 0; i < gap_centres_close.size(); ++i)
        {
            if(std::abs(gap_centres_close[i] - front_angle) < std::abs(closest_gap_centre - front_angle))
            {
                closest_gap_centre = gap_centres_close[i];
            }
        }

        if(gap_centres_close.empty())
        {
            for(size_t i = 0; i < gap_centres_far.size(); ++i)
            {
                if(std::abs(gap_centres_far[i] - front_angle) < std::abs(closest_gap_centre - front_angle))
                {
                    closest_gap_centre = gap_centres_far[i];
                }
            }
        }

        std::string gap_centres_close_str;
        std::string gap_centres_far_str;

        for(const auto& gap_centre : gap_centres_close)
        {
            gap_centres_close_str += std::to_string(gap_centre) + " ";
        }
        for(const auto& gap_centre : gap_centres_far)
        {
            gap_centres_far_str += std::to_string(gap_centre) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Gap centres close: %s, far: %s", gap_centres_close_str.c_str(), gap_centres_far_str.c_str()); 



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
            if(gap_centres_far.size() >= 2)
            {
                ++num_crossroad_detects;

                if(num_crossroad_detects >= 3) // only consider it a crossroad if it is detected for 3 consecutive scans
                {
                    num_crossroad_detects = 0;
                    RCLCPP_INFO(this->get_logger(), "Crossroad detected!");

                    // process aruco
                    // 0|10 = straight, 1|11 = left turn, 2|12 = right turn

                    /*
                    // find the gap centre based on aruco ID
                    float right_gap_angle = gap_centres_angles[0];
                    for(size_t i = 1; i < gap_centres.size(); ++i)
                    {
                        if(gap_centres_angles[i] < right_gap_angle)
                        {
                            right_gap_angle = gap_centres_angles[i];
                        }
                    }
                    */

                    RCLCPP_INFO(this->get_logger(), "Last Aruco ID: %d", aruco_last_id_);

                    if(aruco_last_id_ == 0 || aruco_last_id_ == 10)
                    {
                        ;;
                    }
                    else
                    {
                        auto message_speed = std_msgs::msg::Float32();
                        message_speed.data = 0.0f;
                        wanted_speed_publisher_->publish(message_speed);

                        auto message_angle = std_msgs::msg::Float32();
                        message_angle.data = (aruco_last_id_ == 1 || aruco_last_id_ == 11) ? 90.0f : -90.0f;
                        wanted_angle_publisher_->publish(message_angle);

                        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // wait for the robot to turn

                        message_speed = std_msgs::msg::Float32();
                        message_speed.data = 0.0f;
                        wanted_speed_publisher_->publish(message_speed);

                        message_angle = std_msgs::msg::Float32();
                        message_angle.data = 0.0f;
                        wanted_angle_publisher_->publish(message_angle);

                        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait for the robot to turn
                    }

                    aruco_last_id_ = 2; // reset aruco ID to default right turn

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


        // calculate the angle to the closest gap centre (already in degrees)
        float angle_to_gap_centre_deg = closest_gap_centre;

        auto message_speed = std_msgs::msg::Float32();
        message_speed.data = 10.0f;
        wanted_speed_publisher_->publish(message_speed);

        auto message_angle = std_msgs::msg::Float32();
        message_angle.data = angle_to_gap_centre_deg;
        wanted_angle_publisher_->publish(message_angle);
    }
}