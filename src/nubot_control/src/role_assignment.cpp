#include "role_assignment.hpp"

using namespace nubot;
Role_assignment::Role_assignment(Gazebo_Info & _gazebo_info)
{
    gazebo_info_=& _gazebo_info;
    assignment_result_.resize(TEAM_NUM,-1);
    is_allocated_=false;
}

int Role_assignment::getNearestRobot_(DPoint _point)
{
    vector<Robot> _robots_info=gazebo_info_->Teammates_;
    _robots_info.push_back(gazebo_info_->RobotInfo_);
    float _less_distance=10000;
    int   _nearest_ID=0;
    for(int i=0; i<_robots_info.size(); i++)
    {
        if(assignment_result_[_robots_info[i].robot_id_-1]==-1&&_robots_info[i].is_robot_valid_)
        {
            float _distance=_robots_info[i].robot_loc_.distance(_point);
            if(_distance<_less_distance)
            {
                _less_distance=_distance;
                _nearest_ID=_robots_info[i].robot_id_;
            }
        }
    }
    return _nearest_ID;
}

DPoint Role_assignment::getNearestOpp_(DPoint _point, int order, int step, bool except_nearest_ball)
{
    vector<DPoint> _opponents_info;
    for(int i=0; i<gazebo_info_->Opponents_.size(); i++)
    {
        if(except_nearest_ball&&gazebo_info_->Opponents_[i].robot_loc_.distance(gazebo_info_->BallInfo_.ball_global_loc_)<100)
            continue;
        _opponents_info.push_back(gazebo_info_->Opponents_[i].robot_loc_+step*gazebo_info_->Opponents_[i].robot_vec_);
    }

    DPoint _tmp_info;
    for(int i=0; i<_opponents_info.size()-1; i++)
        for(int j=i+1; j<_opponents_info.size(); j++)
        {
            float _distance_0=_opponents_info[i].distance(_point);
            float _distance_1=_opponents_info[j].distance(_point);
            if(_distance_1<_distance_0)
            {
                _tmp_info=_opponents_info[i];
                _opponents_info[i]=_opponents_info[j];
                _opponents_info[j]=_tmp_info;
            }
        }
    return _opponents_info[order];
}

void Role_assignment::assignment_process_(char _current_strategy)
{
    assignment_result_.clear();
    assignment_result_.resize(TEAM_NUM,-1);

    calculate_position_(_current_strategy);

    if(!gazebo_info_->RobotInfo_.is_robot_valid_)
    {
        is_allocated_=true;
        gazebo_info_->RobotInfo_.current_role_=SUBSTITUTE;
        return;
    }
    if(_current_strategy>STRATEGY_ROBOT && _current_strategy<REGIONAL)
    {
        is_allocated_=true;
        if(gazebo_info_->RobotInfo_.is_robot_dribble_)
        {
            assignment_result_[gazebo_info_->myID]=0;
            gazebo_info_->RobotInfo_.current_role_=ACTIVE;
        }
        else
            for(int i=1; i<robot_positions_.size(); i++)
            {
                int _tmp_id=getNearestRobot_(robot_positions_[i]);
                assignment_result_[_tmp_id-1]=i;
                if(_tmp_id==gazebo_info_->myID)
                {
                    switch (i)
                    {
                    case 1:
                        gazebo_info_->RobotInfo_.current_role_=PASSIVE;
                        break;
                    case 2:
                        gazebo_info_->RobotInfo_.current_role_=ASSISTANT;
                        break;
                    case 3:
                        gazebo_info_->RobotInfo_.current_role_=MIDFIELD;
                        break;
                    }
                    break;
                }
            }
    }
    else if(_current_strategy>RADICAL && _current_strategy<NUBOT_ONE)
    {
        is_allocated_=true;
        for(int i=0; i<robot_positions_.size(); i++)
        {
            int _tmp_id=getNearestRobot_(robot_positions_[i]);
            assignment_result_[_tmp_id-1]=i;
            if(_tmp_id==gazebo_info_->myID)
            {
                switch (i)
                {
                case 0:
                    gazebo_info_->RobotInfo_.current_role_=ACTIVE;
                    break;
                case 1:
                    gazebo_info_->RobotInfo_.current_role_=PASSIVE;
                    break;
                case 2:
                    gazebo_info_->RobotInfo_.current_role_=ASSISTANT;
                    break;
                case 3:
                    gazebo_info_->RobotInfo_.current_role_=MIDFIELD;
                    break;
                }
                break;
            }
        }
    }
    else
    {
        is_allocated_=true;
        gazebo_info_->RobotInfo_.current_role_=gazebo_info_->myID;
    }
}

void Role_assignment::calculate_position_(char _current_strategy)
{
    robot_positions_.clear();
    // robot_position 0
    robot_positions_.push_back(active_position_(_current_strategy));
    // robot_position 1
    robot_positions_.push_back(passive_position_(_current_strategy));
    // robot_position 2
    robot_positions_.push_back(assistant_position_(_current_strategy));
    // robot_position 3
    robot_positions_.push_back(midfield_position_(_current_strategy));
//    std::cout<<gazebo_info_->RobotInfo_.robot_id_<<" -----------------"<<std::endl;
//    for(int i=0; i<robot_positions_.size(); i++)
//        std::cout<<robot_positions_[i].x_<<", "<<robot_positions_[i].y_<<std::endl;
}

//void Role_assignment::predict_opponents_(char _our_strategy, char _opp_strategy)
//{
//    calculate_position_(_our_strategy);
//    switch (_opp_strategy)
//    {
//    case REGIONAL:

//        break;
//    }
//}

DPoint Role_assignment::active_position_(char _current_strategy)
{
    DPoint _position=DPoint(0,0);
    if(_current_strategy>STRATEGY_ROBOT && _current_strategy<REGIONAL)
        _position=gazebo_info_->RobotInfo_.robot_loc_;
    else if(_current_strategy>RADICAL && _current_strategy<NUBOT_ONE)
        _position=gazebo_info_->BallInfo_.ball_global_loc_;

    return _position;
}

DPoint Role_assignment::passive_position_(char _current_strategy)
{
    DPoint _position=DPoint(0,0);
    if(_current_strategy<=RADICAL)
        _position=passive_attack_(_current_strategy);
    else
        switch (_current_strategy)
        {
        case REGIONAL:
            _position=0.3*gazebo_info_->BallInfo_.ball_global_loc_+0.7*field_info_.ourGoal_[GOAL_MIDDLE];
            break;
        case MAN2MAN:
        {
            DPoint _opp_pos=getNearestOpp_(field_info_.ourGoal_[GOAL_MIDDLE]);
            _position=100/gazebo_info_->BallInfo_.ball_global_loc_.distance(_opp_pos)
                    *(gazebo_info_->BallInfo_.ball_global_loc_-_opp_pos)+_opp_pos;
            break;
        }
        case FOCUS:
            _position=200/field_info_.ourGoal_[GOAL_MIDDLE].distance(gazebo_info_->BallInfo_.ball_global_loc_)
                    *(field_info_.ourGoal_[GOAL_MIDDLE]-gazebo_info_->BallInfo_.ball_global_loc_)+gazebo_info_->BallInfo_.ball_global_loc_;
            break;
        }
    return _position;
}

DPoint Role_assignment::assistant_position_(char _current_strategy)
{
    DPoint _position=DPoint(0,0);
    if(_current_strategy<=RADICAL)
        _position=assistant_attack_();
    else
        switch (_current_strategy)
        {
        case REGIONAL:
        {
            DPoint _opp_pos=getNearestOpp_(field_info_.ourGoal_[GOAL_MIDDLE]);
            _position=0.5*_opp_pos+0.5*field_info_.ourGoal_[GOAL_MIDDLE];
            break;
        }
        case MAN2MAN:
        {
            DPoint _opp_pos=getNearestOpp_(field_info_.ourGoal_[GOAL_MIDDLE],1);
            _position=100/gazebo_info_->BallInfo_.ball_global_loc_.distance(_opp_pos)
                    *(gazebo_info_->BallInfo_.ball_global_loc_-_opp_pos)+_opp_pos;
            break;
        }
        case FOCUS:
        {
            DPoint _opp_pos=getNearestOpp_(gazebo_info_->BallInfo_.ball_global_loc_,1);
            _position=100/_opp_pos.distance(gazebo_info_->BallInfo_.ball_global_loc_)
                    *(_opp_pos-gazebo_info_->BallInfo_.ball_global_loc_)+gazebo_info_->BallInfo_.ball_global_loc_;
            break;
        }
        }
    return _position;
}

DPoint Role_assignment::midfield_position_(char _current_strategy)
{
    DPoint _position=DPoint(0,0);
    if(_current_strategy<=RADICAL)
        _position=midfield_attack_(_current_strategy);
    else
        switch (_current_strategy)
        {
        case REGIONAL:
        {
            DPoint _opp_pos=getNearestOpp_(field_info_.ourGoal_[GOAL_MIDDLE],1);
            _position=0.5*_opp_pos+0.5*field_info_.ourGoal_[GOAL_MIDDLE];
            break;
        }
        case MAN2MAN:
        {
            DPoint _opp_pos=getNearestOpp_(field_info_.ourGoal_[GOAL_MIDDLE],2);
            _position=100/gazebo_info_->BallInfo_.ball_global_loc_.distance(_opp_pos)
                    *(gazebo_info_->BallInfo_.ball_global_loc_-_opp_pos)+_opp_pos;
            break;
        }
        case FOCUS:
        {
            DPoint _opp_pos=getNearestOpp_(gazebo_info_->BallInfo_.ball_global_loc_,2);
            _position=300/_opp_pos.distance(gazebo_info_->BallInfo_.ball_global_loc_)
                    *(_opp_pos-gazebo_info_->BallInfo_.ball_global_loc_)+gazebo_info_->BallInfo_.ball_global_loc_;
            break;
        }
        }
    return _position;
}

DPoint Role_assignment::substitute_position_(char _robotID)
{
    return DPoint(_robotID*110-982,-692);
}

DPoint Role_assignment::passive_attack_(char _current_strategy)
{
    DPoint _position=DPoint(0,0);
    DPoint _opp_pos=getNearestOpp_(field_info_.ourGoal_[GOAL_MIDDLE]);
    if(_opp_pos.x_<0)
    {
        if(_current_strategy==RADICAL)
            _position=0.5*_opp_pos+0.5*field_info_.ourGoal_[GOAL_MIDDLE];
        else if(_current_strategy==CONSERVATIVE)
            _position=0.5*_opp_pos+0.5*gazebo_info_->BallInfo_.ball_global_loc_;
        else
            _position=DPoint(-400,0);
    }
    else
        _position=DPoint(-400,0);
    return _position;
}

DPoint Role_assignment::assistant_attack_()
{
    DPoint _robot_pos = gazebo_info_->RobotInfo_.robot_loc_;
    DPoint _ball_pos  = gazebo_info_->BallInfo_.ball_global_loc_;

    float radii_lower = (FIELD_WIDTH/2 > 500) ? 500 : (FIELD_WIDTH/2);
    float radii_upper = FIELD_WIDTH*4/5;
    int pass_line_expand = 100;
    int tooFarObs  = 400;
    int step_radii = 50;
    int num_angle  = 80;
    int obs_expand_radii = 80;
    std::map<double,DPoint> pointToChoose;

    for(int i=1;i<num_angle;i++)
    {
        float angle_temp = i*DOUBLEPI_CONSTANT/num_angle;
        double profit=0;

        for(float j=radii_lower;j<radii_upper;j+=step_radii)
        {
            float radii_temp = j;
            DPoint pos_temp = DPoint(radii_temp*cos(angle_temp),radii_temp*sin(angle_temp))+_ball_pos;
            if ((fabs(pos_temp.x_) > (FIELD_LENGTH/2 - 300)) || (fabs(pos_temp.y_ ) > (FIELD_WIDTH/2 - 150)))
                break;

            float dis2obs_sum=0;
            if(gazebo_info_->Opponents_.size()==0)
            {
                profit = -1*fabs(pos_temp.x_-FIELD_LENGTH/4)-0.3*_robot_pos.distance(pos_temp);
                pointToChoose.insert(make_pair(profit,pos_temp));
            }
            else
                for(int i=0;i<gazebo_info_->Opponents_.size();i++)
                {
                    if(pos_temp.distance(gazebo_info_->Opponents_[i].robot_loc_) < obs_expand_radii)
                        break;
                    if(_ball_pos.distance(gazebo_info_->Opponents_[i].robot_loc_) < tooFarObs)
                    {
                        double dis2ball = pos_temp.distance(_ball_pos);
                        DPoint ball_pos_closer = _ball_pos+100.0/dis2ball*(pos_temp-_ball_pos);

                        LineSegment ball2temp_pos = LineSegment(ball_pos_closer,pos_temp);
                        if((ball2temp_pos.decideWhere(gazebo_info_->Opponents_[i].robot_loc_) == ball2temp_pos.BetweenStartAndEnd) &&
                                (ball2temp_pos.distance(gazebo_info_->Opponents_[i].robot_loc_) < pass_line_expand))
                            break;

                        dis2obs_sum += pos_temp.distance(gazebo_info_->Opponents_[i].robot_loc_);
                    }

                    if(i==gazebo_info_->Opponents_.size()-1)
                    {
                        profit = -1*fabs(pos_temp.x_ - FIELD_LENGTH/4)+0.5*dis2obs_sum-0.3*_robot_pos.distance(pos_temp);
                        pointToChoose.insert(make_pair(profit,pos_temp));
                    }
                }
        }
    }
    if(pointToChoose.size() != 0)
    {
        std::map<double,DPoint>::iterator mapiter_end = pointToChoose.end();
        mapiter_end--;
        return mapiter_end->second;
    }
    else
        return _robot_pos;
}

DPoint Role_assignment::midfield_attack_(char _current_strategy)
{
    int pass_line_expand = 100;
    int obs_expand_radii = 80;
    int tooFarObs        = 400;
    int step_radii       = 50;
    int num_angle        = 80;
    float radii_lower    = 0;
    float radii_upper = FIELD_WIDTH*4/5;
    std::map<double,DPoint> pointToChoose;

    DPoint _ball_pos = gazebo_info_->BallInfo_.ball_global_loc_;
    DPoint _robot_pos  = gazebo_info_->RobotInfo_.robot_loc_;

    for(int i=0;i<num_angle;i++)
    {
        float angle_temp = i*DOUBLEPI_CONSTANT/num_angle;
        double profit=0;

        for(float j=radii_lower;j<radii_upper;j+=step_radii)
        {
            float radii_temp = j;
            DPoint pos_temp = DPoint(radii_temp*cos(angle_temp),radii_temp*sin(angle_temp)) + DPoint(0,0);
            if((fabs(pos_temp.x_) > 300) || (fabs(pos_temp.y_) > (FIELD_WIDTH/2 - 150)))
                break;

            if ((_current_strategy==CONSERVATIVE) && (pos_temp.x_<0))
                break;
            else if ((_current_strategy==RADICAL) && (pos_temp.x_>0))
                break;

            if(pos_temp.distance(_ball_pos) < 300)
                break;

            float dis2obs_sum=0;
            if(gazebo_info_->Opponents_.size()==0)
            {
                profit = -1*fabs(pos_temp.y_ - _ball_pos.y_)-0.3*fabs(pos_temp.x_);
                pointToChoose.insert(make_pair(profit,pos_temp));
            }
            else
                for(int i=0;i<gazebo_info_->Opponents_.size();i++)
                {

                    if(pos_temp.distance(gazebo_info_->Opponents_[i].robot_loc_) < obs_expand_radii)
                        break;
                    if(pos_temp.distance(gazebo_info_->Opponents_[i].robot_loc_) < tooFarObs)
                    {
                        double dis2ball = pos_temp.distance(_ball_pos);
                        DPoint ball_pos_closer = _ball_pos + 80.0/dis2ball*(pos_temp-_ball_pos);

                        LineSegment ball2temp_pos = LineSegment(ball_pos_closer,pos_temp);

                        if((ball2temp_pos.decideWhere(gazebo_info_->Opponents_[i].robot_loc_)==ball2temp_pos.BetweenStartAndEnd) &&
                                (ball2temp_pos.distance(gazebo_info_->Opponents_[i].robot_loc_) < pass_line_expand) )
                            break;

                        dis2obs_sum += pos_temp.distance(gazebo_info_->Opponents_[i].robot_loc_);
                    }
                    if(i==gazebo_info_->Opponents_.size()-1)
                    {
                        profit = -3*fabs(pos_temp.y_ - _ball_pos.y_)-0.5*fabs(pos_temp.x_);
                        pointToChoose.insert(make_pair(profit,pos_temp));
                    }
                }
        }
    }
    if(pointToChoose.size() != 0)
    {
        std::map<double,DPoint>::iterator mapiter_end = pointToChoose.end();
        mapiter_end--;
        return mapiter_end->second;
    }
    else
        return _robot_pos;
}
