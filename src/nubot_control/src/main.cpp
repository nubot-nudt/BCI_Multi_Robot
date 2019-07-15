#include "robot_control.hpp"
using namespace nubot;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"robot_control_node");
    ros::Time::init();
    Robot_Control Robot_Control(argc,argv);
    ros::spin();
    return 0;
}

