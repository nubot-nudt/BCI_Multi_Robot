#ifndef _NUBOT_BALL_H_
#define _NUBOT_BALL_H_

#include "robot.hpp"

namespace nubot{
class Ball
{
public:
    Ball(bool is_valid=false,DPoint loc=DPoint(0,0),
         PPoint real_loc=PPoint(Angle(0),0.0),DPoint velocity=DPoint(0,0), char state=FREE_BALL)
    {
        ball_global_loc_ = loc;
        ball_real_loc_   = real_loc;
        is_ball_valid_   = is_valid;
        ball_velocity_   = velocity;
        ball_state_      = state;
    }

    ~Ball(){}
public:
    bool      is_ball_valid_;
    PPoint    ball_real_loc_;
    DPoint    ball_global_loc_;
    DPoint    ball_velocity_;
    char      ball_state_;
};
}

#endif
