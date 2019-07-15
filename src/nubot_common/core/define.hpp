#ifndef DEFINE_H
#define DEFINE_H

#include "ball.hpp"

namespace nubot{

class Gazebo_Info
{
public:
    Ball                    BallInfo_;
    Robot                   RobotInfo_;
    std::vector<Robot>      Teammates_;
    std::vector<Robot>      Opponents_;
    std::vector<DPoint>     Obstacles_;

    int CurActiveRobotNums_;
    int myID;

public:
    Gazebo_Info()
    {
        Teammates_.resize(TEAM_NUM-1);
        Opponents_.resize(TEAM_NUM);
        CurActiveRobotNums_=0;
        myID=0;
        Obstacles_.resize(2*TEAM_NUM-1);
    }

    void caculateActiveRobots()
    {
        CurActiveRobotNums_=0;
        for(unsigned i=0 ; i<TEAM_NUM-1; i++)
            CurActiveRobotNums_ += Teammates_[i].is_robot_valid_;
        if(RobotInfo_.is_robot_valid_)
            CurActiveRobotNums_=+1;
    }

    char getballstate()
    {
        char _ball_state=FREE_BALL;
        for(unsigned i=0; i<TEAM_NUM-1; i++)
            if(Teammates_[i].is_robot_valid_&&Teammates_[i].is_robot_dribble_)
                _ball_state=OUR_DRIBBLE;
        if(RobotInfo_.is_robot_valid_&&RobotInfo_.is_robot_dribble_)
            _ball_state=OUR_DRIBBLE;

        return _ball_state;
    }
    //    void checkDribble(const bool & ball_holding)
    //    {
    //        DPoint robot_pos = RobotInfo_[robotID_-1].robot_loc_;
    //        DPoint ball_pos = BallInfo_[robotID_-1].getGlobalLocation();
    //        DPoint ball2robot= DPoint(BallInfo_[robotID_-1].getRealLocation());

    //        double dis2b = robot_pos.distance(ball_pos);
    //        bool dribblecheck = dis2b <= ConstDribbleDisFirst ? ball_holding : false;
    //        DribbleState_.update(dribblecheck,robot_pos,ball2robot);
    //        RobotInfo_[robotID_-1].is_robot_dribble_=dribblecheck;
    //    }
};

struct interface_Info
{
    Ball                BallInfo_;
    Robot               RobotInfo_[TEAM_NUM];
    std::vector<Robot>  OpponentInfo_;
    bool                isNewstrategy_;
    char                recommend_strategy_;
};

struct BCI_signal
{
    bool      select_signal;
    char      result_signal;
};

struct BCIcontrol_Info
{
    bool      isReset;
    bool      isNeworder;
    bool      stopORstart;
    char      selectStrategy;
    char      attackMode;
    char      defendMode;
    char      selectRobot;
    char      robotMode;
};
}
#endif // DEFINE_H
