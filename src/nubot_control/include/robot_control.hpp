#ifndef _ROBOT_CONTROL_H
#define _ROBOT_CONTROL_H

#include <nubot_common/VelCmd.h>
#include <nubot_common/BallHandle.h>
#include <nubot_common/Shoot.h>
#include <nubot_common/GazeboInfo.h>
#include <nubot_common/BCIInfo.h>
#include <nubot_common/InterfaceInfo.h>
#include <nubot_common/StrategyInfo.h>
#include <nubot_common/AllocationInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <boost/bind.hpp>

#include "role_assignment.hpp"
#include "dribblestate.hpp"

using namespace nubot;
typedef message_filters::sync_policies::ApproximateTime<nubot_common::GazeboInfo, nubot_common::AllocationInfo> syncPolicy;
class Robot_Control
{
public:
//    ros::Subscriber  gazeboinfo_sub_;
    ros::Subscriber  bciinfo_sub_;
//    ros::Subscriber  allocationinfo_sub_;

    ros::Publisher   velcmd_pub_;
    ros::Publisher   strategy_pub_;
    ros::Publisher   interface_pub_;

    ros::Timer         control_timer_;
    ros::Time          start_human_strategy_;
    ros::Time          start_move_;
    ros::NodeHandle    nh_;
    ros::ServiceClient ballhandle_client_;
    ros::ServiceClient shoot_client_;

    message_filters::Subscriber<nubot_common::GazeboInfo> *gazeboinfo_sub_;
    message_filters::Subscriber<nubot_common::AllocationInfo> *allocationinfo_sub_;
    message_filters::Synchronizer<syncPolicy> *sync_;

    nubot_common::BallHandle  ballhandle;
    nubot_common::Shoot       shoot;
public:
    Gazebo_Info gazebo_info_;
    BCIcontrol_Info bcicontrol_info_;
    Behaviour *m_behaviour_;
    Role_assignment *m_role_assignment_;

    char ball_state_;
    char recommend_strategy_;
    char final_strategy_;
    vector<float> payoff_;
    bool wait_human_strategy_;
    bool message_lock_;
    bool isReset_;
    float  max_vel_;

public:
    Robot_Control(int argc,char **argv);
    ~Robot_Control();

    void  updateRobotinfo_(const nubot_common::GazeboInfo::ConstPtr & _gazebo_msg, const nubot_common::AllocationInfo::ConstPtr & _allocation_msg);
    void  updateBCIinfo_(const nubot_common::BCIInfo & _bci_msg);
    void  loopControl_(const ros::TimerEvent& event);
    char  getStrategy_(char _ball_state);
    float calculateDBI_(char _possible_strategy, int step);
    float calculateR_(vector<DPoint> _group_1, vector<DPoint> _group_2);
//    float calculatePayoff_(vector<DPoint> _our_positions, vector<DPoint> _opp_positions);
    float whichMAX_(int size, float *_list);
    void getConvexhull_(vector<DPoint> & _convex_hull, vector<DPoint> _tmp_pos);
    void calculateArea_(vector<float> & _p_strategy, vector<DPoint> _convex_hull);
    void singleControl_();

    void normalization_(vector<float> & _ori_value);
    void setVelCommond_();
    void pubStrategy_();
    void pubInterfaceinfo_();
    void stopRobot_();
    void catchBall_();
    bool position_(DPoint _target);
};

#endif // _ROBOT_CONTROL_H
