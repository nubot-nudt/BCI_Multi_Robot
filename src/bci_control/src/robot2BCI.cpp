#include <robot2BCI.h>

using namespace nubot;

Robot2BCI::Robot2BCI()
{
    ros::Time::init();
    boost::shared_ptr<ros::NodeHandle> nh_;
    nh_= boost::make_shared<ros::NodeHandle>();
    BCI2robot_pub_       = nh_->advertise<nubot_common::BCIInfo>("BCI_control/BCI_info",10);
    BCI2opponent_pub_    = nh_->advertise<nubot_common::ResetInfo>("BCI_control/reset_info",10);
    robot_allocation_pub_= nh_->advertise<nubot_common::AllocationInfo>("BCI_control/allocation_info",10);
    BCI_publish_timer_   = nh_->createTimer(ros::Duration(0.1),&Robot2BCI::publish_,this);
    BCI_info_sub_        = nh_->subscribe("BCI_background/bci_signal",10,&Robot2BCI::update_bci_info_,this);
    robot_strategy_sub_  = nh_->subscribe("nubot1/nubotcontrol/recommend_strategy",10,&Robot2BCI::recommend_strategy_,this);
    for(unsigned i=0;i<TEAM_NUM;i++)
        robot2BCI_sub_[i]= nh_->subscribe<nubot_common::InterfaceInfo>
                ("nubot"+std::to_string(i+1)+"/nubotcontrol/interface_info",10,boost::bind(&Robot2BCI::update_info_,this,_1,i));
}
Robot2BCI::~Robot2BCI()
{
}

//update the interface info from robots
void Robot2BCI::update_info_(const nubot_common::InterfaceInfo::ConstPtr &_msg, int topic_id)
{
    //robot information
    robot2BCI_info.RobotInfo_[topic_id].robot_id_=_msg->robotinfo.robotID;
    robot2BCI_info.RobotInfo_[topic_id].robot_loc_.x_=_msg->robotinfo.pos.x;
    robot2BCI_info.RobotInfo_[topic_id].robot_loc_.y_=_msg->robotinfo.pos.y;
    robot2BCI_info.RobotInfo_[topic_id].robot_head_.radian_=_msg->robotinfo.heading.theta;
    robot2BCI_info.RobotInfo_[topic_id].robot_vec_.x_=_msg->robotinfo.vtrans.x;
    robot2BCI_info.RobotInfo_[topic_id].robot_vec_.y_=_msg->robotinfo.vtrans.y;
    robot2BCI_info.RobotInfo_[topic_id].robot_w_=_msg->robotinfo.vrot;
    robot2BCI_info.RobotInfo_[topic_id].is_robot_valid_=_msg->robotinfo.isvalid;
    robot2BCI_info.RobotInfo_[topic_id].is_robot_dribble_=_msg->robotinfo.isdribble;
    robot2BCI_info.RobotInfo_[topic_id].is_robot_location_=_msg->robotinfo.islocation;
    robot2BCI_info.RobotInfo_[topic_id].current_role_=_msg->robotinfo.currentRole;
    robot2BCI_info.RobotInfo_[topic_id].current_action_=_msg->robotinfo.currentAction;
    robot2BCI_info.RobotInfo_[topic_id].battery_state_=_msg->robotinfo.batteryState;
}

void Robot2BCI::update_bci_info_(const std_msgs::Int8 &_msg)
{
    std::cout<<_msg<<std::endl;
    if(_msg.data<0||_msg.data>6)
        ROS_ERROR("Invalid BCI signal!");
    else if(_msg.data==6)
        BCI_signal_info.select_signal=true;
    else
        BCI_signal_info.result_signal=_msg.data;
}

void Robot2BCI::recommend_strategy_(const nubot_common::StrategyInfo &_msg)
{
    robot2BCI_info.isNewstrategy_=true;
    robot2BCI_info.recommend_strategy_=_msg.recommend_strategy;
}

void Robot2BCI::publish_(const ros::TimerEvent &)
{
    nubot_common::AllocationInfo _tmp_allocation_info;
    for(int i=0;i<TEAM_NUM;i++)
    {
        _tmp_allocation_info.header.stamp  =ros::Time::now();
        _tmp_allocation_info.robotID.push_back(robot2BCI_info.RobotInfo_[i].robot_id_);
        _tmp_allocation_info.isvalid.push_back(robot2BCI_info.RobotInfo_[i].is_robot_valid_);
        _tmp_allocation_info.isdribble.push_back(robot2BCI_info.RobotInfo_[i].is_robot_dribble_);
        _tmp_allocation_info.islocation.push_back(robot2BCI_info.RobotInfo_[i].is_robot_location_);
        _tmp_allocation_info.currentRole.push_back(robot2BCI_info.RobotInfo_[i].current_role_);
        _tmp_allocation_info.currentAction.push_back(robot2BCI_info.RobotInfo_[i].current_action_);
        _tmp_allocation_info.batteryState.push_back(robot2BCI_info.RobotInfo_[i].battery_state_);
    }
    for(int j=0; j<robot2BCI_info.OpponentInfo_.size(); j++)
    {
        nubot_common::Point2d _tmp_vel;
        _tmp_vel.x=robot2BCI_info.OpponentInfo_[j].robot_vec_.x_;
        _tmp_vel.y=robot2BCI_info.OpponentInfo_[j].robot_vec_.y_;
        _tmp_allocation_info.except_vel.push_back(_tmp_vel);
    }
    robot_allocation_pub_.publish(_tmp_allocation_info);

    if(BCI2robot_info.isNeworder)
    {
        nubot_common::BCIInfo _tmp_BCIinfo;
        _tmp_BCIinfo.selectstrategy=BCI2robot_info.selectStrategy;
        _tmp_BCIinfo.attackmode=BCI2robot_info.attackMode;
        _tmp_BCIinfo.defendmode=BCI2robot_info.defendMode;
        _tmp_BCIinfo.selectrobot=BCI2robot_info.selectRobot;
        _tmp_BCIinfo.robotmode=BCI2robot_info.robotMode;
        _tmp_BCIinfo.stop_start=BCI2robot_info.stopORstart;

        BCI2robot_pub_.publish(_tmp_BCIinfo);
        //after publish,reset
        BCI2robot_info.isNeworder=false;
        BCI2robot_info.selectStrategy=NO_STRATEGY;
        BCI2robot_info.attackMode=NO_STRATEGY;
        BCI2robot_info.defendMode=NO_STRATEGY;
        BCI2robot_info.selectRobot=NO_STRATEGY;
        BCI2robot_info.robotMode=NO_STRATEGY;
    }

    if(BCI2robot_info.isReset)
    {
        nubot_common::ResetInfo _tmp_resetinfo;
        reset_opp_ball(_tmp_resetinfo);
        BCI2opponent_pub_.publish(_tmp_resetinfo);

        nubot_common::BCIInfo _tmp_BCIinfo;
        _tmp_BCIinfo.isreset=BCI2robot_info.isReset;
        _tmp_BCIinfo.ball_state=_tmp_resetinfo.ball_state;
        BCI2robot_pub_.publish(_tmp_BCIinfo);

        BCI2robot_info.isReset=false;
    }
}

void Robot2BCI::reset_opp_ball(nubot_common::ResetInfo & _tmp_resetinfo)
{
    //choose the ball's position randomly
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
    if(qrand()%(2)==FREE_BALL)
    {
        int _tmp_ball_x,_tmp_ball_y;
        bool is_accept_ball;
        do
        {
            is_accept_ball=true;
            _tmp_ball_x=qrand()%(FIELD_LENGTH-200)-800;    //-800~800
            _tmp_ball_y=qrand()%(FIELD_WIDTH)-600;         //-600~600
            for(int j=0; j<TEAM_NUM; j++)
                if(DPoint(_tmp_ball_x,_tmp_ball_y).distance(robot2BCI_info.RobotInfo_[j].robot_loc_)<100&&
                   robot2BCI_info.RobotInfo_[j].current_role_!=SUBSTITUTE)
                {
                    is_accept_ball=false;
                    break;
                }
        }while(!is_accept_ball);

        _tmp_resetinfo.ball_x=_tmp_ball_x;
        _tmp_resetinfo.ball_y=_tmp_ball_y;
        _tmp_resetinfo.ball_state=FREE_BALL;
        robot2BCI_info.BallInfo_.ball_global_loc_=DPoint(_tmp_ball_x,_tmp_ball_y);
        robot2BCI_info.BallInfo_.ball_state_=_tmp_resetinfo.ball_state;
    }
    else
    {
        _tmp_resetinfo.ball_state=UNKNOWN;
        robot2BCI_info.BallInfo_.ball_state_=_tmp_resetinfo.ball_state;
    }

    //choose the opponents' position randomly
    robot2BCI_info.OpponentInfo_.clear();
    int _tmp_opp_x,_tmp_opp_y;
    bool is_accept;
    for(int i=0; i<OPP_TEAM_NUM; i++)
    {
        if(_tmp_resetinfo.ball_state==FREE_BALL&&i==0)
        {
            _tmp_opp_x=qrand()%(100);
            _tmp_opp_y=100-_tmp_opp_x;
            _tmp_opp_x=qrand()%(2)==0? -_tmp_opp_x:_tmp_opp_x;
            _tmp_opp_y=qrand()%(2)==0? -_tmp_opp_y:_tmp_opp_y;
            _tmp_resetinfo.opp_x.push_back(_tmp_opp_x+_tmp_resetinfo.ball_x);
            _tmp_resetinfo.opp_y.push_back(_tmp_opp_y+_tmp_resetinfo.ball_y);

            continue;
        }
        do
        {
            is_accept=true;
            _tmp_opp_x=qrand()%(FIELD_LENGTH-200)-800;    //-800~800
            _tmp_opp_y=qrand()%(FIELD_WIDTH)-600;         //-600~600

            if(DPoint(_tmp_opp_x,_tmp_opp_y).distance(robot2BCI_info.BallInfo_.ball_global_loc_)<100)
                is_accept=false;
            if(is_accept)
                for(int j=0; j<TEAM_NUM; j++)
                    if(DPoint(_tmp_opp_x,_tmp_opp_y).distance(robot2BCI_info.RobotInfo_[j].robot_loc_)<200&&
                            robot2BCI_info.RobotInfo_[j].current_role_!=SUBSTITUTE)
                    {
                        is_accept=false;
                        break;
                    }
            if(is_accept)
                for(int j=0; j<_tmp_resetinfo.opp_x.size(); j++)
                    if(DPoint(_tmp_opp_x,_tmp_opp_y).distance(DPoint(_tmp_resetinfo.opp_x[j],_tmp_resetinfo.opp_y[j]))<200)
                    {
                        is_accept=false;
                        break;
                    }
        }while(!is_accept);
        _tmp_resetinfo.opp_x.push_back(_tmp_opp_x);
        _tmp_resetinfo.opp_y.push_back(_tmp_opp_y);
    }

    //choose the opponents' angle and velocity
    _tmp_resetinfo.opp_theta.push_back(M_PI+thetaof2p(DPoint(_tmp_resetinfo.opp_x[0],_tmp_resetinfo.opp_y[0]),
                                                      robot2BCI_info.BallInfo_.ball_global_loc_));
    _tmp_resetinfo.opp_Vx.push_back((robot2BCI_info.BallInfo_.ball_global_loc_.x_-_tmp_resetinfo.opp_x[0])/PREDICT_TIMER);
    _tmp_resetinfo.opp_Vy.push_back((robot2BCI_info.BallInfo_.ball_global_loc_.y_-_tmp_resetinfo.opp_y[0])/PREDICT_TIMER);

    for(int i=1; i<OPP_TEAM_NUM; i++)
    {
        _tmp_opp_x=qrand()%(FIELD_LENGTH-200)-800;    //-800~800
        _tmp_opp_y=qrand()%(FIELD_WIDTH)-600;         //-600~600
        _tmp_resetinfo.opp_theta.push_back(M_PI+thetaof2p(DPoint(_tmp_resetinfo.opp_x[i],_tmp_resetinfo.opp_y[i]),
                                                     DPoint(_tmp_opp_x,_tmp_opp_y)));
        _tmp_resetinfo.opp_Vx.push_back((_tmp_opp_x-_tmp_resetinfo.opp_x[0])/PREDICT_TIMER);
        _tmp_resetinfo.opp_Vy.push_back((_tmp_opp_y-_tmp_resetinfo.opp_y[0])/PREDICT_TIMER);
    }


    for(unsigned i=0;i<OPP_TEAM_NUM;i++)
    {
        Robot _tmp_opponent_info;
        _tmp_opponent_info.robot_id_=i+1;
        _tmp_opponent_info.robot_loc_.x_=_tmp_resetinfo.opp_x[i];
        _tmp_opponent_info.robot_loc_.y_=_tmp_resetinfo.opp_y[i];
        _tmp_opponent_info.robot_head_.radian_=_tmp_resetinfo.opp_theta[i]-M_PI;
        _tmp_opponent_info.robot_vec_.x_=_tmp_resetinfo.opp_Vx[i];
        _tmp_opponent_info.robot_vec_.y_=_tmp_resetinfo.opp_Vy[i];
        _tmp_opponent_info.is_robot_valid_=true;

        robot2BCI_info.OpponentInfo_.push_back(_tmp_opponent_info);
    }
}

void Robot2BCI::run()
{
    ros::spin();
}
