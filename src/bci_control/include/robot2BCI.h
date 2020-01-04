#ifndef ROBOT2BCI_H
#define ROBOT2BCI_H

#include "define.hpp"
#include "nubot_common/BCIInfo.h"
#include "nubot_common/InterfaceInfo.h"
#include "nubot_common/StrategyInfo.h"
#include "nubot_common/AllocationInfo.h"
#include "nubot_common/ResetInfo.h"
#include <QDebug>
#include <QtCore>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

using namespace std;
namespace nubot {

class Robot2BCI:public QThread
{
public:
    ros::Subscriber  robot2BCI_sub_[TEAM_NUM];                    //sub the interface_info from robots
    ros::Subscriber  robot_strategy_sub_;
    ros::Subscriber  BCI_info_sub_;
    ros::Publisher   BCI2robot_pub_;                              //pub BCI_info to all robots
    ros::Publisher   BCI2opponent_pub_;
    ros::Publisher   robot_allocation_pub_;
    ros::Timer       BCI_publish_timer_;

    interface_Info   robot2BCI_info;
    BCIcontrol_Info  BCI2robot_info;
    BCI_signal       BCI_signal_info;

public:
    Robot2BCI();
    void run();
    void update_info_(const nubot_common::InterfaceInfo::ConstPtr & _msg, int topic_id);
    void update_bci_info_(const std_msgs::Int8 & _msg);
    void recommend_strategy_(const nubot_common::StrategyInfo & _msg);
    void publish_(const ros::TimerEvent &);
    void reset_opp_ball(nubot_common::ResetInfo &_tmp_resetinfo);
    void operator ()();
    ~Robot2BCI();
};

}

#endif // ROBOT2BCI_H
