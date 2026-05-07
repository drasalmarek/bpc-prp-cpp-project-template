
#include "nodes/node_lidar_image.hpp"

void lidar_data(const sensor_msgs::msg::LaserScan::SharedPtr msg, float& front_avg, float& left_avg, float& right_avg, float& back_avg)
{
    front_avg = 0.0f;
    left_avg = 0.0f;
    right_avg = 0.0f;
    back_avg = 0.0f;

    float front_angle = 0.0f;

    const float front_left_bound = -175 * M_PI / 180; // 160 fungovalo
    const float front_right_bound = 175 * M_PI / 180;

    const float left_left_bound = -120 * M_PI / 180;
    const float left_right_bound = -90 * M_PI / 180;

    const float right_left_bound = 90 * M_PI / 180;
    const float right_right_bound = 120 * M_PI / 180;

    const float back_left_bound = -20 * M_PI / 180;
    const float back_right_bound = 20 * M_PI / 180;

    uint32_t valid_front_count = 0;
    uint32_t valid_left_count = 0;
    uint32_t valid_right_count = 0;
    uint32_t valid_back_count = 0;
    for(size_t i = 0; i < msg->ranges.size(); ++i) 
    {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];

        if (range > msg->range_min && range < msg->range_max) 
        {
            if (angle <= front_left_bound || angle >= front_right_bound)
            {
                front_avg += range;
                valid_front_count++;
            } 
            else if (angle >= left_left_bound && angle <= left_right_bound)
            {
                left_avg += range;
                valid_left_count++;
            } 
            else if (angle >= right_left_bound && angle <= right_right_bound)
            {
                right_avg += range;
                valid_right_count++;
            }
            else if (angle >= back_left_bound && angle <= back_right_bound)
            {
                back_avg += range;
                valid_back_count++;
            }
        }
    }

    front_avg /= (valid_front_count > 0) ? valid_front_count : 1;
    left_avg /= (valid_left_count > 0) ? valid_left_count : 1;
    right_avg /= (valid_right_count > 0) ? valid_right_count : 1;
    back_avg /= (valid_back_count > 0) ? valid_back_count : 1;
}

float find_max_length(cv::Mat& input_image, float angle_min_deg, float angle_max_deg)
{
    size_t rows = input_image.rows;
    size_t cols = input_image.cols;
    size_t center_x = cols / 2;
    size_t center_y = rows / 2;

    float max_distance = -1.0f;
    float best_angle = angle_min_deg;

    // Convention:
    // 0° = up (front), positive clockwise
    // image coords: +x right, +y down
    for (float angle_deg = angle_min_deg; angle_deg <= angle_max_deg; angle_deg += 3.0f)
    {
        float angle_rad = angle_deg * CV_PI / 180.0f;
        float dx = std::sin(angle_rad);
        float dy = -std::cos(angle_rad);  // IMPORTANT: invert Y for image coordinates

        float traveled = 0.0f;

        for (float t = 0.0f; t < 100.0f; t += 0.1f)
        {
            float next_x = static_cast<float>(center_x) + dx * t;
            float next_y = static_cast<float>(center_y) + dy * t;

            int ix = static_cast<int>(next_x);
            int iy = static_cast<int>(next_y);

            if (ix < 0 || ix >= static_cast<int>(cols) || iy < 0 || iy >= static_cast<int>(rows))
            {
                traveled = t;
                break;
            }

            // obstacle
            if (input_image.at<uint8_t>(iy, ix) > 128)
            {
                traveled = t;
                break;
            }

            traveled = t;
        }

        // Apply continuous weighting: front angles (0°) get higher weight
        float weight = std::cos((angle_deg * CV_PI / 180.0f) / 2.0f) + 1.0f;  // ranges from ~0.3 to 2.0
        float weighted_distance = traveled * weight;

        if (weighted_distance > max_distance)
        {
            max_distance = weighted_distance;
            best_angle = angle_deg;
        }
    }

    return best_angle;
}

double findDominantAngle(const cv::Mat& input)
{
    cv::Mat gray;
    if (input.channels() == 3)
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    else
        gray = input.clone();

    cv::Mat binary;
    cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY);

    cv::Mat edges;
    cv::Canny(binary, edges, 50, 150);

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edges, lines, 1, CV_PI / 180.0, 20);

    if (lines.empty())
        return 0.0;

    // Histogram for [-90 ... +90]
    std::vector<int> hist(181, 0);

    for (const auto& line : lines)
    {
        double theta = line[1] * 180.0 / CV_PI;

        // Convert Hough normal angle to line angle
        // 0 = vertical(up), positive = clockwise
        double angle = theta;

        // fold 180-periodicity
        if (angle > 180.0)
            angle -= 180.0;

        // make 0 = up
        angle -= 180.0;

        // normalize to [-90,+90]
        while (angle > 90.0) angle -= 180.0;
        while (angle < -90.0) angle += 180.0;

        int idx = (int)std::round(angle) + 90;
        hist[idx]++;
    }

    int bestVotes = -1;
    double bestAngle = 0.0;

    for (int i = 0; i < 181; i++)
    {
        double angle = i - 90.0;
        int votes = hist[i];

        if (votes > bestVotes ||
            (votes == bestVotes &&
             std::abs(angle) < std::abs(bestAngle)))
        {
            bestVotes = votes;
            bestAngle = angle;
        }
    }

    if(bestAngle > 70.0)
    {
        bestAngle -= 90.0;
    }
    else if(bestAngle < -70.0)
    {
        bestAngle += 90.0;
    }

    return bestAngle;
}

namespace nodes
{
// Compute candidate headings (degrees) from an occupancy image where obstacles are 255 and free is 0.
// Returns a vector of angles in degrees (0 = +x image direction, increasing counter-clockwise).
    node_lidar_image::node_lidar_image(const std::string& lidar_topic,
        const std::string& image_topic,
        const std::string& aruco_topic,
        const std::string& wanted_speed_topic,
        const std::string& wanted_angle_topic)
        : Node("node_lidar_image")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&node_lidar_image::subscriber_callback, this, std::placeholders::_1));

        aruco_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            aruco_topic, 10, std::bind(&node_lidar_image::aruco_callback, this, std::placeholders::_1));

        wanted_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_speed_topic, 10);
        wanted_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(wanted_angle_topic, 10);

        last_crossroad_time_ = this->now() - crossroad_cooldown_;
    }
    void node_lidar_image::aruco_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        uint8_t aruco_id = static_cast<uint8_t>(msg->data);

        //if((aruco_id > 2) && (aruco_id < 10 || aruco_id > 12))
        if(aruco_id > 2)
        {
            return;
        }

        aruco_last_id_ = aruco_id;
        RCLCPP_INFO(this->get_logger(), "Updated Aruco ID: %d", aruco_last_id_);
    }

    void node_lidar_image::subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a new opencv Image
        cv::Mat image_msg(128, 128, CV_8UC1, cv::Scalar(0));

        // Convert LaserScan to top-down corridor view
        float max_range = msg->range_max;
        float image_scale = 128.0f / (0.15f * max_range);
        int center = 64;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            if (range > msg->range_min && range < msg->range_max && range > 0.1f) {
                int x = static_cast<int>(center + range * std::cos(angle) * image_scale);
                int y = static_cast<int>(center + range * std::sin(angle) * image_scale);

                if (x >= 0 && x < static_cast<int>(image_msg.cols) &&
                    y >= 0 && y < static_cast<int>(image_msg.rows)) {
                    image_msg.at<uint8_t>(y, x) = 255;
                }
            }
        }

        cv::rotate(image_msg, image_msg, cv::ROTATE_90_CLOCKWISE);
        cv::flip(image_msg, image_msg, 1);

        cv::Mat dilated;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20));
        cv::dilate(image_msg, dilated, kernel, cv::Point(-1, -1), 1);

        cv::Mat output_image = dilated;

        float best_heading = findDominantAngle(output_image);

        // filter heading angle
        static float filtered_heading = best_heading;
        const float alpha = 0.5f;
        filtered_heading = alpha * best_heading + (1.0f - alpha) * filtered_heading;
        best_heading = filtered_heading;
        std::vector<double> headings = {best_heading};

        //RCLCPP_INFO(this->get_logger(), "Best heading: %.2f degrees", best_heading);

        float lidar_front_avg = 0.0f;
        float lidar_left_avg = 0.0f;
        float lidar_right_avg = 0.0f;
        float lidar_back_avg = 0.0f;
        lidar_data(msg, lidar_front_avg, lidar_left_avg, lidar_right_avg, lidar_back_avg);

        RCLCPP_INFO(this->get_logger(), "Lidar front avg: %.2f m, left avg: %.2f m, right avg: %.2f m, back avg: %.2f m", lidar_front_avg, lidar_left_avg, lidar_right_avg, lidar_back_avg);

        // detect crossroad
        const float crossroad_left_threshold = 0.55f; // adjust as needed
        const float crossroad_right_threshold = 0.55f; // adjust as needed
        const float crossroad_front_threshold = 0.55f; // adjust as needed
        const float crossroad_back_threshold = 0.5f; // adjust as needed
        if (lidar_back_avg > crossroad_back_threshold && this->now() - last_crossroad_time_ > crossroad_cooldown_)
        {
            if((lidar_left_avg > crossroad_left_threshold && lidar_right_avg > crossroad_right_threshold && lidar_front_avg > crossroad_front_threshold) || 
               (lidar_right_avg > crossroad_right_threshold && lidar_front_avg > crossroad_front_threshold) ||
               (lidar_left_avg > crossroad_left_threshold && lidar_front_avg > crossroad_front_threshold))
            {
                RCLCPP_INFO(this->get_logger(), "Crossroad detected!");

                if(lidar_back_avg > 0.6f)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                auto message_speed = std_msgs::msg::Float32();
                message_speed.data = 0.0f;
                wanted_speed_publisher_->publish(message_speed);

                auto message_angle = std_msgs::msg::Float32();

                switch(aruco_last_id_)
                {
                    case 0:
                    case 10:
                        // go straight
                        break;
                    case 1:
                    case 11:
                        // turn left
                        message_angle.data = 30.0f; // turn left
                        wanted_angle_publisher_->publish(message_angle);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                        message_angle.data = 0.0f;
                        wanted_angle_publisher_->publish(message_angle);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        break;
                    case 2:
                    case 12:
                        // turn right
                        message_angle.data = -30.0f; // turn right
                        wanted_angle_publisher_->publish(message_angle);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                        message_angle.data = 0.0f;
                        wanted_angle_publisher_->publish(message_angle);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        break;
                    default:
                        break;
                }

                last_crossroad_time_ = this->now();
            }
        }

        static uint8_t num_crossroad_detects = 0;
        if (lidar_front_avg < 0.3f) 
        {
            num_crossroad_detects++;

            if (num_crossroad_detects > 3) // require multiple consecutive detects to avoid false positives
            {
                num_crossroad_detects = 0;

                RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f meters ahead, stopping robot.", lidar_front_avg);

                auto message_speed = std_msgs::msg::Float32();
                message_speed.data = 0.0f;
                wanted_speed_publisher_->publish(message_speed);

                if(lidar_left_avg > crossroad_left_threshold && lidar_right_avg > crossroad_right_threshold)
                {
                    auto message_angle = std_msgs::msg::Float32();
                    switch(aruco_last_id_)
                    {
                        case 0:
                        case 10:
                            // go straight (but we are in a dead end, so turn around)
                            message_angle.data = 30.0f; // turn around
                            wanted_angle_publisher_->publish(message_angle);
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                            message_angle.data = 0.0f;
                            wanted_angle_publisher_->publish(message_angle);
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                            break;
                        case 1:
                        case 11:
                            message_angle.data = 30.0f; // turn left
                            wanted_angle_publisher_->publish(message_angle);
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                            message_angle.data = 0.0f;
                            wanted_angle_publisher_->publish(message_angle);
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                            break;
                        case 2:
                        case 12:
                            message_angle.data = -30.0f; // turn right
                            wanted_angle_publisher_->publish(message_angle);
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                            message_angle.data = 0.0f;
                            wanted_angle_publisher_->publish(message_angle);
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                            break;
                        default:
                            break;
                    }

                    last_crossroad_time_ = this->now();
                }

                else if(lidar_left_avg > crossroad_left_threshold)
                {
                    auto message_angle = std_msgs::msg::Float32();
                    message_angle.data = 30.0f; // turn left
                    wanted_angle_publisher_->publish(message_angle);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                    message_angle.data = 0.0f;
                    wanted_angle_publisher_->publish(message_angle);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                else if(lidar_right_avg > crossroad_right_threshold)
                {
                    auto message_angle = std_msgs::msg::Float32();
                    message_angle.data = -30.0f; // turn right
                    wanted_angle_publisher_->publish(message_angle);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                    message_angle.data = 0.0f;
                    wanted_angle_publisher_->publish(message_angle);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                else
                {
                    // we are in a dead end, turn around
                    auto message_angle = std_msgs::msg::Float32();
                    message_angle.data = 30.0f; // turn around
                    wanted_angle_publisher_->publish(message_angle);
                    std::this_thread::sleep_for(std::chrono::milliseconds(4000));

                    message_angle.data = 0.0f;
                    wanted_angle_publisher_->publish(message_angle);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }
        }

        if(lidar_left_avg < 0.2f)
        {
            best_heading += 3.0f;
        }
        else if(lidar_right_avg < 0.2f)
        {
            best_heading -= 3.0f;
        }

        auto message_speed = std_msgs::msg::Float32();
        if(lidar_front_avg < 0.6f)
        {
            message_speed.data = 10.0f;
        }
        else
        {
            message_speed.data = 20.0f;
        }
        wanted_speed_publisher_->publish(message_speed);

        auto message_angle = std_msgs::msg::Float32();
        message_angle.data = -best_heading;
        wanted_angle_publisher_->publish(message_angle); 
            

        // Create an RGB debug image (BGR for OpenCV) and draw red lines for each candidate heading
        cv::Mat debug_bgr;
        cv::cvtColor(output_image, debug_bgr, cv::COLOR_GRAY2BGR);
        cv::Point centerPt(debug_bgr.cols/2, debug_bgr.rows/2);
        int line_len = std::min(debug_bgr.cols, debug_bgr.rows) / 2 - 2;
        for (double hdeg : headings) {
            // Same convention as find_max_length:
            // 0° up, positive clockwise
            double th = hdeg * CV_PI / 180.0;
            double dx = std::sin(th) * line_len;
            double dy = -std::cos(th) * line_len;

            cv::Point endPt;
            endPt.x = static_cast<int>(std::round(centerPt.x + dx));
            endPt.y = static_cast<int>(std::round(centerPt.y + dy));

            cv::line(debug_bgr, centerPt, endPt, cv::Scalar(0,0,255), 2, cv::LINE_AA);
            cv::circle(debug_bgr, centerPt, 2, cv::Scalar(0,255,0), -1);
        }

        // Convert the OpenCV BGR debug image to a ROS Image message (bgr8)
        sensor_msgs::msg::Image ros_image_msg;
        ros_image_msg.header = msg->header;
        ros_image_msg.height = debug_bgr.rows;
        ros_image_msg.width = debug_bgr.cols;
        ros_image_msg.encoding = "bgr8";
        ros_image_msg.is_bigendian = false;
        ros_image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(debug_bgr.cols * debug_bgr.elemSize());
        ros_image_msg.data.assign(debug_bgr.data, debug_bgr.data + debug_bgr.total() * debug_bgr.elemSize());

        // Publish the debug RGB Image message
        publisher_->publish(ros_image_msg);
    }
}