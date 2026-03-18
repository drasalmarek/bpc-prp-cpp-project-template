
#include <math.h>
#include <stdint.h>
#include "iostream"

namespace algorithms
{
    struct RobotSpeed{
    float v; //linear
    float w; //angluar
    };

    struct WheelSpeed{ //depends on you in what units
    float l; //left
    float r; //right
    };

    struct Encoders{
    uint32_t l; //left
    uint32_t r; //right
    };

    struct Coordinates{ //Cartesian coordinates
    float x; 
    float y;
    float theta; //orientation
    };

    class Kinematics{

        float wheel_radius_;
        float wheel_base_;
        int32_t ticks_revolution_;

        uint32_t previous_left_ticks_;
        uint32_t previous_right_ticks_;
        float previous_x_;
        float previous_y_;
        float previous_theta_;

    public:
        Kinematics(float wheel_radius, float wheel_base, int32_t ticks_revolution);

        RobotSpeed forward(const struct WheelSpeed& x) const;
        WheelSpeed inverse(const struct RobotSpeed& x) const;
        Coordinates forward(const struct Encoders& x);
        Encoders inverse(const struct Coordinates& x) const;
    };
}