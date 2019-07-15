#ifndef NUBOT_ROBOT_H_
#define NUBOT_ROBOT_H_

#include "common.hpp"

namespace nubot{
class Robot
{
public:
    Robot(int id=-1, DPoint loc=DPoint(0,0), Angle head=Angle(0), DPoint vec=DPoint(0,0),
          double w=0.0, bool isdribble=false, bool isvalid=false, bool islocation=true, float _battery_s=100,
          char _current_role=0, char _current_action_=0, DPoint _target = DPoint(0,0))
    {
        robot_id_  = id;
        robot_loc_ = loc;
        robot_head_= head;
        robot_vec_ = vec;
        robot_w_   = w;
        is_robot_dribble_ = isdribble;
        is_robot_valid_   = isvalid;
        is_robot_location_= islocation;
        battery_state_    = _battery_s;
        current_role_     = _current_role;
        current_action_   = _current_action_;
        target_           = _target;
    }
    ~Robot(){}
public:
    int      robot_id_;
    DPoint   robot_loc_;
    Angle    robot_head_;
    DPoint   robot_vec_;
    double   robot_w_;
    bool     is_robot_dribble_;
    bool     is_robot_valid_;
    bool     is_robot_location_;
    float    battery_state_;
    char     current_role_;
    char     current_action_;
    DPoint   target_;
};
}

#endif // ROBOTINFO_H
