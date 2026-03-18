
#include "kinematics.hpp"

namespace algorithms
{   
    Kinematics::Kinematics(float wheel_radius, float wheel_base, int32_t ticks_revolution)
                : wheel_radius_(wheel_radius), wheel_base_(wheel_base), ticks_revolution_(ticks_revolution),
                    previous_left_ticks_(0), previous_right_ticks_(0),
                    previous_x_(0.0f), previous_y_(0.0f), previous_theta_(0.0f) {}

    RobotSpeed Kinematics::forward(const struct WheelSpeed& x) const 
    {
        struct RobotSpeed result;
        result.v = wheel_radius_ * (x.r + x.l) / 2.0;
        result.w = wheel_radius_ * (x.r - x.l) / wheel_base_;
        return result;
    }

    WheelSpeed Kinematics::inverse(const struct RobotSpeed& x) const 
    {
        struct WheelSpeed result;
        result.r = (2.0 * x.v + x.w * wheel_base_) / (2.0 * wheel_radius_);
        result.l = (2.0 * x.v - x.w * wheel_base_) / (2.0 * wheel_radius_);
        return result;
    }

    Coordinates Kinematics::forward(const struct Encoders& x) 
    {
        struct Coordinates result;
        float left_distance = static_cast<float>(x.l - previous_left_ticks_) * 2.0 * M_PI * wheel_radius_ / static_cast<float>(ticks_revolution_);
        float right_distance = static_cast<float>(x.r - previous_right_ticks_) * 2.0 * M_PI * wheel_radius_ / static_cast<float>(ticks_revolution_);
        previous_left_ticks_ = x.l;
        previous_right_ticks_ = x.r;

        float ds = (left_distance + right_distance) / 2.0;
        float dtheta = (right_distance - left_distance) / wheel_base_;

        result.x = previous_x_ + ds * cos(previous_theta_ + dtheta / 2.0);
        result.y = previous_y_ + ds * sin(previous_theta_ + dtheta / 2.0);
        result.theta = previous_theta_ + dtheta;

        previous_x_ = result.x;
        previous_y_ = result.y;
        previous_theta_ = result.theta;

        return result;
    }

    Encoders Kinematics::inverse(const struct Coordinates& x) const 
    {
        struct Encoders result;

        float ds = sqrt(pow(x.x, 2) + pow(x.y, 2));
        float dtheta = x.theta;
        
        float left_distance = ds - (dtheta * wheel_base_) / 2.0;
        float right_distance = ds + (dtheta * wheel_base_) / 2.0;
        
        result.l = static_cast<uint32_t>((left_distance / (2.0 * M_PI * wheel_radius_)) * static_cast<float>(ticks_revolution_));
        result.r = static_cast<uint32_t>((right_distance / (2.0 * M_PI * wheel_radius_)) * static_cast<float>(ticks_revolution_));
        return result;
    }
}