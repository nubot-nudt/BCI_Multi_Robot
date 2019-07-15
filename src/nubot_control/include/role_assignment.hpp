#ifndef ROLE_ASSIGNMENT_H
#define ROLE_ASSIGNMENT_H

#include "behaviour.hpp"
#include "Fieldinformation.hpp"

namespace nubot{
class Role_assignment
{
public:
    Role_assignment(Gazebo_Info &_gazebo_info);
    int getNearestRobot_(DPoint _point);
    DPoint getNearestOpp_(DPoint _point, int order=0, int step=0, bool except_nearest_ball=true);
    void assignment_process_(char _current_strategy);
    void calculate_position_(char _current_strategy);
//    void predict_opponents_(char _our_strategy, char _opp_strategy);

    DPoint active_position_(char _current_strategy);
    DPoint passive_position_(char _current_strategy);
    DPoint assistant_position_(char _current_strategy);
    DPoint midfield_position_(char _current_strategy);
    DPoint substitute_position_(char _robotID);

    DPoint passive_attack_(char _current_strategy);
    DPoint assistant_attack_();
    DPoint midfield_attack_(char _current_strategy);

public:
    FieldInformation field_info_;
    Gazebo_Info      *gazebo_info_;
    vector<DPoint>   robot_positions_;
    vector<DPoint>   predict_opp_positions_;
    vector<int>      assignment_result_;
    bool             is_allocated_;
};
}
#endif //ROLE_ASSIGNMENT_H

