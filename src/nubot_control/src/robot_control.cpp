#include "robot_control.hpp"

using namespace nubot;
Robot_Control::Robot_Control(int argc, char **argv)
{
    const char * environment;
    std::string robot_name;
    std::string str = argv[1];
    std::string str2= str.substr(str.size()-1);
    environment = str2.c_str();
    robot_name = str;
    ROS_INFO("robot_control initialize: robot_name:%s",robot_name.c_str());

    velcmd_pub_         = nh_.advertise<nubot_common::VelCmd>(robot_name+"/nubotcontrol/velcmd",10);
    strategy_pub_       = nh_.advertise<nubot_common::StrategyInfo>(robot_name+"/nubotcontrol/recommend_strategy",10);
    interface_pub_      = nh_.advertise<nubot_common::InterfaceInfo>(robot_name+"/nubotcontrol/interface_info",10);
    ballhandle_client_  = nh_.serviceClient<nubot_common::BallHandle>(robot_name+"/BallHandle");
    shoot_client_       = nh_.serviceClient<nubot_common::Shoot>(robot_name+"/Shoot");
    control_timer_      = nh_.createTimer(ros::Duration(0.1),&Robot_Control::loopControl_,this);
    bciinfo_sub_        = nh_.subscribe("BCI_control/BCI_info",1,&Robot_Control::updateBCIinfo_,this);

    gazeboinfo_sub_=new message_filters::Subscriber<nubot_common::GazeboInfo>(nh_, robot_name+"/nubot_gazebo/gazebo_info",1);
    allocationinfo_sub_=new message_filters::Subscriber<nubot_common::AllocationInfo>(nh_, "BCI_control/allocation_info",1);
    sync_=new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *gazeboinfo_sub_, *allocationinfo_sub_);
    sync_->registerCallback(boost::bind(&Robot_Control::updateRobotinfo_,this,_1,_2));

    m_behaviour_= new Behaviour(gazebo_info_.Obstacles_);
    m_role_assignment_ = new Role_assignment(gazebo_info_);
    gazebo_info_.myID = atoi(environment);
    ball_state_ = OUR_DRIBBLE;
    recommend_strategy_=NO_STRATEGY;
    bcicontrol_info_.stopORstart=STOPROBOT;
    final_strategy_=REGIONAL;
    max_vel_=MAXVEL;
    wait_human_strategy_=false;
    if(gazebo_info_.myID==5)
        gazebo_info_.RobotInfo_.is_robot_valid_=false;
    else
        gazebo_info_.RobotInfo_.is_robot_valid_=true;
    payoff_.resize(9,0);
    start_human_strategy_=ros::Time::now();
    message_lock_=false;
    isReset_=false;
}

Robot_Control::~Robot_Control()
{
    m_behaviour_->app_vx_ = 0;
    m_behaviour_->app_vy_ = 0;
    m_behaviour_->app_w_  = 0;
    setVelCommond_();
}

void Robot_Control::setVelCommond_()
{
    nubot_common::VelCmd  command;
    command.Vx = m_behaviour_->app_vx_;
    command.Vy = m_behaviour_->app_vy_;
    command.w  = m_behaviour_->app_w_;

    m_behaviour_->app_vx_=0;
    m_behaviour_->app_vy_=0;
    m_behaviour_->app_w_=0;

    velcmd_pub_.publish(command);
}

void Robot_Control::updateRobotinfo_(const nubot_common::GazeboInfo::ConstPtr &_gazebo_msg, const nubot_common::AllocationInfo::ConstPtr &_allocation_msg)
{
    message_lock_=true;

    gazebo_info_.Teammates_.clear();
    gazebo_info_.Opponents_.clear();
    gazebo_info_.Obstacles_.clear();

    //robot_info and teammates_info(obstacles) obtained from Gazebo
    for(unsigned i=0;i<_gazebo_msg->robotinfo.size();i++)
    {
        if(gazebo_info_.myID==_gazebo_msg->robotinfo[i].robotID)
        {
            DPoint _current_pos=DPoint(_gazebo_msg->robotinfo[i].pos.x,_gazebo_msg->robotinfo[i].pos.y);
            double _distance=_current_pos.distance(gazebo_info_.RobotInfo_.robot_loc_);
            gazebo_info_.RobotInfo_.battery_state_-=_distance/1000;

            gazebo_info_.RobotInfo_.robot_id_=_gazebo_msg->robotinfo[i].robotID;
            gazebo_info_.RobotInfo_.robot_loc_.x_=_gazebo_msg->robotinfo[i].pos.x;
            gazebo_info_.RobotInfo_.robot_loc_.y_=_gazebo_msg->robotinfo[i].pos.y;
            gazebo_info_.RobotInfo_.robot_head_.radian_=_gazebo_msg->robotinfo[i].heading.theta;
            gazebo_info_.RobotInfo_.robot_vec_.x_=_gazebo_msg->robotinfo[i].vtrans.x;
            gazebo_info_.RobotInfo_.robot_vec_.y_=_gazebo_msg->robotinfo[i].vtrans.y;
            gazebo_info_.RobotInfo_.robot_w_=_gazebo_msg->robotinfo[i].vrot;
        }
        else
        {
            Robot _tmp_teammate;
            _tmp_teammate.robot_id_=_gazebo_msg->robotinfo[i].robotID;
            _tmp_teammate.robot_loc_.x_=_gazebo_msg->robotinfo[i].pos.x;
            _tmp_teammate.robot_loc_.y_=_gazebo_msg->robotinfo[i].pos.y;
            _tmp_teammate.robot_head_.radian_=_gazebo_msg->robotinfo[i].heading.theta;
            _tmp_teammate.robot_vec_.x_=_gazebo_msg->robotinfo[i].vtrans.x;
            _tmp_teammate.robot_vec_.y_=_gazebo_msg->robotinfo[i].vtrans.y;
            _tmp_teammate.robot_w_=_gazebo_msg->robotinfo[i].vrot;

            for(int j=0;j<_allocation_msg->robotID.size();j++)
                if(_allocation_msg->robotID[j]==_tmp_teammate.robot_id_)
                {
                    _tmp_teammate.is_robot_valid_=_allocation_msg->isvalid[j];
                    _tmp_teammate.is_robot_dribble_=_allocation_msg->isdribble[j];
                    _tmp_teammate.is_robot_location_=_allocation_msg->islocation[j];
                    _tmp_teammate.current_role_=_allocation_msg->currentRole[j];
                    _tmp_teammate.current_action_=_allocation_msg->currentAction[j];
                    _tmp_teammate.battery_state_=_allocation_msg->batteryState[j];
                }

            gazebo_info_.Teammates_.push_back(_tmp_teammate);

            //teammates also are obstacles for the robot
            DPoint _tmp_obstacle;
            _tmp_obstacle.x_=_gazebo_msg->robotinfo[i].pos.x;
            _tmp_obstacle.y_=_gazebo_msg->robotinfo[i].pos.y;
            gazebo_info_.Obstacles_.push_back(_tmp_obstacle);
        }
    }

    //ball_info obtained from Gazebo
    gazebo_info_.BallInfo_.ball_global_loc_.x_=_gazebo_msg->ballinfo.pos.x;
    gazebo_info_.BallInfo_.ball_global_loc_.y_=_gazebo_msg->ballinfo.pos.y;
    gazebo_info_.BallInfo_.ball_velocity_.x_=_gazebo_msg->ballinfo.velocity.x;
    gazebo_info_.BallInfo_.ball_velocity_.y_=_gazebo_msg->ballinfo.velocity.y;

    //opponent_info(obstacles) obtainde from Gazebo
    for(unsigned i=0;i<_gazebo_msg->opponentinfo.size();i++)
    {
        Robot _tmp_opponent;
        _tmp_opponent.robot_id_=_gazebo_msg->opponentinfo[i].opponentID;
        _tmp_opponent.robot_loc_.x_=_gazebo_msg->opponentinfo[i].pos.x;
        _tmp_opponent.robot_loc_.y_=_gazebo_msg->opponentinfo[i].pos.y;
        _tmp_opponent.robot_head_.radian_=_gazebo_msg->opponentinfo[i].heading.theta;
        _tmp_opponent.robot_w_=_gazebo_msg->opponentinfo[i].vrot;
        _tmp_opponent.is_robot_valid_=_gazebo_msg->opponentinfo[i].isvalid;

        //the velocity from allocation_msg
        if(_allocation_msg->except_vel.size())
        {
            _tmp_opponent.robot_vec_.x_=_allocation_msg->except_vel[_tmp_opponent.robot_id_-1].x;
            _tmp_opponent.robot_vec_.y_=_allocation_msg->except_vel[_tmp_opponent.robot_id_-1].y;
        }

        gazebo_info_.Opponents_.push_back(_tmp_opponent);

        //opponent are obstacles for the robot
        DPoint _tmp_obstacle;
        _tmp_obstacle.x_=_gazebo_msg->opponentinfo[i].pos.x;
        _tmp_obstacle.y_=_gazebo_msg->opponentinfo[i].pos.y;
        gazebo_info_.Obstacles_.push_back(_tmp_obstacle);
    }

    gazebo_info_.caculateActiveRobots();

    message_lock_=false;
}

void Robot_Control::updateBCIinfo_(const nubot_common::BCIInfo &_bci_msg)
{
    if(_bci_msg.isreset)
    {
        isReset_=true;
        ball_state_=_bci_msg.ball_state==UNKNOWN? gazebo_info_.getballstate():_bci_msg.ball_state;
        return;
    }

    if(wait_human_strategy_)
        wait_human_strategy_=false;

    bcicontrol_info_.stopORstart=_bci_msg.stop_start;
    bcicontrol_info_.attackMode=_bci_msg.attackmode;
    bcicontrol_info_.defendMode=_bci_msg.defendmode;
    bcicontrol_info_.robotMode=_bci_msg.robotmode;
    bcicontrol_info_.selectRobot=_bci_msg.selectrobot;
    bcicontrol_info_.selectStrategy=_bci_msg.selectstrategy;

    if(bcicontrol_info_.stopORstart==STOPROBOT)
    {
        stopRobot_();
        setVelCommond_();
    }
    else if(bcicontrol_info_.selectStrategy==STRATEGY_ROBOT)
        singleControl_();
    else if(ball_state_==OUR_DRIBBLE&&bcicontrol_info_.selectStrategy==STRATEGY_ATTACK)
    {
        if(bcicontrol_info_.attackMode!=NO_STRATEGY)
            final_strategy_=bcicontrol_info_.attackMode;
        else
            final_strategy_=recommend_strategy_;
    }
    else if(ball_state_==FREE_BALL&&bcicontrol_info_.selectStrategy==STRATEGY_DEFEND)
    {
        if(bcicontrol_info_.defendMode!=NO_STRATEGY)
            final_strategy_=bcicontrol_info_.defendMode;
        else
            final_strategy_=recommend_strategy_;
    }
}

void Robot_Control::loopControl_(const ros::TimerEvent &event)
{
    if(message_lock_)
        return;
    if(bcicontrol_info_.stopORstart==STOPROBOT)
    {
        pubInterfaceinfo_();
        return;
    }

    bool _tmp_is_all_location=gazebo_info_.RobotInfo_.is_robot_location_;
    for(int i=0; i<gazebo_info_.Teammates_.size(); i++)
        _tmp_is_all_location=_tmp_is_all_location&&gazebo_info_.Teammates_[i].is_robot_location_;

    if(_tmp_is_all_location)
        ballhandle.request.enable=0;

    if(!ballhandle_client_.call(ballhandle))
        ROS_ERROR("Failed to call service");

    gazebo_info_.RobotInfo_.is_robot_dribble_=ballhandle.response.BallIsHolding;

    if(isReset_)
    {
        gazebo_info_.RobotInfo_.is_robot_location_=false;
        m_role_assignment_->is_allocated_=false;
        recommend_strategy_=getStrategy_(ball_state_);
        pubStrategy_();
        wait_human_strategy_=true;
        isReset_=false;
        start_human_strategy_=ros::Time::now();
    }
    if(wait_human_strategy_)
    {
        ros::Duration _duration=ros::Time::now()-start_human_strategy_;
        float _wait_time=_duration.toSec();
        if(_wait_time>30)
        {
            wait_human_strategy_=false;
            final_strategy_=recommend_strategy_;
        }
    }
    else if(!m_role_assignment_->is_allocated_)
    {
        m_role_assignment_->assignment_process_(final_strategy_);
        start_move_=ros::Time::now();
    }
    else
    {
//        DPoint _my_target;
//        std::cout<<"ID: "<<gazebo_info_.RobotInfo_.robot_id_<<" role: "<<int(gazebo_info_.RobotInfo_.current_role_)<<std::endl;
        switch (gazebo_info_.RobotInfo_.current_role_)
        {
        case ACTIVE:
//            _my_target=m_role_assignment_->active_position_(final_strategy_);
            ballhandle.request.enable=1;
            if(ball_state_==FREE_BALL)
            {
                catchBall_();
                gazebo_info_.RobotInfo_.current_action_=CatchBall;
            }
            else
            {
                gazebo_info_.RobotInfo_.is_robot_location_=position_(m_role_assignment_->robot_positions_[0]);
                if(gazebo_info_.RobotInfo_.is_robot_location_)
                    gazebo_info_.RobotInfo_.current_action_=Positioned;
                else
                    gazebo_info_.RobotInfo_.current_action_=AvoidObs;
            }
            break;
        case PASSIVE:
//            _my_target=m_role_assignment_->passive_position_(final_strategy_);
            ballhandle.request.enable=0;
            gazebo_info_.RobotInfo_.is_robot_location_=position_(m_role_assignment_->robot_positions_[1]);
            if(gazebo_info_.RobotInfo_.is_robot_location_)
                gazebo_info_.RobotInfo_.current_action_=Positioned;
            else
                gazebo_info_.RobotInfo_.current_action_=AvoidObs;
            break;
        case ASSISTANT:
//            _my_target=m_role_assignment_->assistant_position_(final_strategy_);
            ballhandle.request.enable=0;
            gazebo_info_.RobotInfo_.is_robot_location_=position_(m_role_assignment_->robot_positions_[2]);
            if(gazebo_info_.RobotInfo_.is_robot_location_)
                gazebo_info_.RobotInfo_.current_action_=Positioned;
            else
                gazebo_info_.RobotInfo_.current_action_=AvoidObs;
            break;
        case MIDFIELD:
//            _my_target=m_role_assignment_->midfield_position_(final_strategy_);
            ballhandle.request.enable=0;
            gazebo_info_.RobotInfo_.is_robot_location_=position_(m_role_assignment_->robot_positions_[3]);
            if(gazebo_info_.RobotInfo_.is_robot_location_)
                gazebo_info_.RobotInfo_.current_action_=Positioned;
            else
                gazebo_info_.RobotInfo_.current_action_=AvoidObs;
            break;
        case SUBSTITUTE:
//            _my_target=DPoint(40+(gazebo_info_.myID-1)*55,665);
            ballhandle.request.enable=0;
            gazebo_info_.RobotInfo_.is_robot_location_=position_(DPoint((gazebo_info_.myID-5)*55-430,-720));
            gazebo_info_.RobotInfo_.current_action_=No_Action;
            break;
        }
        setVelCommond_();
    }

    pubInterfaceinfo_();
}

char Robot_Control::getStrategy_(char _ball_state)
{
    if(_ball_state==OUR_DRIBBLE)
    {
        float _max_regional=0,_max_man2man=0,_max_focus=0;
        float _DBI_regional=0,_DBI_man2man=0,_DBI_focus=0;
        vector<float> _P_strategy;

        for(int i=0; i<PREDICT_TIMER; i++)
        {
            _DBI_regional=calculateDBI_(REGIONAL,i);
            _DBI_man2man =calculateDBI_(MAN2MAN,i);
            _DBI_focus   =calculateDBI_(FOCUS,i);

            if(_DBI_regional>_max_regional)
                _max_regional=_DBI_regional;
            if(_DBI_man2man>_max_man2man)
                _max_man2man=_DBI_man2man;
            if(_DBI_focus>_max_focus)
                _max_focus=_DBI_focus;
        }

        //normalization
        _P_strategy.push_back(_max_regional-_DBI_regional);
        _P_strategy.push_back(_max_man2man-_DBI_man2man);
        _P_strategy.push_back(_max_focus-_DBI_focus);
        normalization_(_P_strategy);

//        for(int i=CONSERVATIVE; i<=RADICAL; i++)
//            for(int j=REGIONAL; j<=FOCUS; j++)
//            {
//                m_role_assignment_->predict_opponents_(i,j);
//                payoff_.push_back(calculatePayoff_(m_role_assignment_->robot_positions_,m_role_assignment_->predict_opp_positions_));
//            }

        vector<float> _payoff_strategy;
        for(int i=0; i<3; i++)
            _payoff_strategy.push_back(_P_strategy[0]*PAY_OFF[0][i]+_P_strategy[1]*PAY_OFF[1][i]+_P_strategy[2]*PAY_OFF[2][i]);

        if(_payoff_strategy[0]>_payoff_strategy[1] && _payoff_strategy[0]>_payoff_strategy[2])
            return CONSERVATIVE;
        else if(_payoff_strategy[1]>_payoff_strategy[0] && _payoff_strategy[1]>_payoff_strategy[2])
            return BALANCE;
        else if(_payoff_strategy[2]>_payoff_strategy[0] && _payoff_strategy[2]>_payoff_strategy[1])
            return RADICAL;
        else
        return BALANCE;
    }
    else
    {
        vector<DPoint> _convex_hull;
        vector<float>  _P_strategy;
        vector<float>  _payoff_strategy;
        vector<DPoint> _tmp_opponents_pos;
        for(int i=0; i<gazebo_info_.Opponents_.size(); i++)
            _tmp_opponents_pos.push_back(gazebo_info_.Opponents_[i].robot_loc_+PREDICT_TIMER*gazebo_info_.Opponents_[i].robot_vec_);

        getConvexhull_(_convex_hull,_tmp_opponents_pos);
        calculateArea_(_P_strategy,_convex_hull);
        normalization_(_P_strategy);

//        std::cout<<_P_strategy[0]<<" "<<_P_strategy[1]<<" "<<_P_strategy[2]<<std::endl;
        for(int i=0; i<3; i++)
            _payoff_strategy.push_back(_P_strategy[0]*PAY_OFF[2][i]+_P_strategy[1]*PAY_OFF[1][i]+_P_strategy[2]*PAY_OFF[0][i]);

        if(_payoff_strategy[0]>_payoff_strategy[1] && _payoff_strategy[0]>_payoff_strategy[2])
            return REGIONAL;
        else if(_payoff_strategy[1]>_payoff_strategy[0] && _payoff_strategy[1]>_payoff_strategy[2])
            return MAN2MAN;
        else if(_payoff_strategy[2]>_payoff_strategy[0] && _payoff_strategy[2]>_payoff_strategy[1])
            return FOCUS;
        else
            return MAN2MAN;
    }
}

float Robot_Control::calculateDBI_(char _possible_strategy, int step)
{
    float _DBI=0.0;
    vector<DPoint> _tmp_opponents_pos;
    for(int i=0; i<gazebo_info_.Opponents_.size(); i++)
        _tmp_opponents_pos.push_back(gazebo_info_.Opponents_[i].robot_loc_+step*gazebo_info_.Opponents_[i].robot_vec_);

    switch (_possible_strategy)
    {
    case REGIONAL:
    {
        DPoint _delete_opp=m_role_assignment_->getNearestOpp_(gazebo_info_.BallInfo_.ball_global_loc_,0,step,false);
        vector<DPoint> _group_A,_group_B;

        //group A is the set of our robots' positions
        _group_A.push_back(gazebo_info_.RobotInfo_.robot_loc_);
        for(int i=0; i<gazebo_info_.Teammates_.size(); i++)
            _group_A.push_back(gazebo_info_.Teammates_[i].robot_loc_);

        //group B is the set of opponents' positions (except the opponent which want to catch the ball)
        for(int i=0; i<_tmp_opponents_pos.size(); i++)
            if(_tmp_opponents_pos[i]!=_delete_opp)
                _group_B.push_back(_tmp_opponents_pos[i]);

        float _R_AB=calculateR_(_group_A,_group_B);
        _DBI=_R_AB;
        break;
    }
    case MAN2MAN:
    {
        vector<DPoint> _robots_pos;
        _robots_pos.push_back(gazebo_info_.RobotInfo_.robot_loc_);
        for(int i=0; i<gazebo_info_.Teammates_.size(); i++)
            _robots_pos.push_back(gazebo_info_.Teammates_[i].robot_loc_);

        //group A,B,C,D is composed with one our robot and an opponent which defend it
        vector<DPoint> _group_A,_group_B,_group_C,_group_D;
        vector<char> _tmp_opponent;
        _tmp_opponent.resize(_tmp_opponents_pos.size(),-1);
        for(int i=0; i<_robots_pos.size(); i++)
        {
            float _less_distance=2000;
            char  _tmp_label=0;
            for(int j=0; j<_tmp_opponents_pos.size(); j++)
            {
                if(_tmp_opponent[j]==-1)
                {
                    float _distance=_robots_pos[i].distance(_tmp_opponents_pos[j]);
                    if(_distance<_less_distance)
                    {
                        _less_distance=_distance;
                        _tmp_label=j;
                    }
                }
                if(j==_tmp_opponents_pos.size()-1)
                    _tmp_opponent[_tmp_label]=i;
            }
        }

        _group_A.push_back(_tmp_opponents_pos[0]);
        _group_A.push_back(_robots_pos[_tmp_opponent[0]]);
        _group_B.push_back(_tmp_opponents_pos[1]);
        _group_B.push_back(_robots_pos[_tmp_opponent[1]]);
        _group_C.push_back(_tmp_opponents_pos[2]);
        _group_C.push_back(_robots_pos[_tmp_opponent[2]]);
        _group_D.push_back(_tmp_opponents_pos[3]);
        _group_D.push_back(_robots_pos[_tmp_opponent[3]]);

        float _R_AB=calculateR_(_group_A,_group_B);
        float _R_AC=calculateR_(_group_A,_group_C);
        float _R_AD=calculateR_(_group_A,_group_D);
        float _R_BC=calculateR_(_group_B,_group_C);
        float _R_BD=calculateR_(_group_B,_group_D);
        float _R_CD=calculateR_(_group_C,_group_D);

        float _R_A[3]={_R_AB,_R_AC,_R_AD};
        float _R_B[3]={_R_AB,_R_BC,_R_BD};
        float _R_C[3]={_R_AC,_R_BC,_R_CD};
        float _R_D[3]={_R_AD,_R_BD,_R_CD};

        float _D_A=whichMAX_(3,_R_A);
        float _D_B=whichMAX_(3,_R_B);
        float _D_C=whichMAX_(3,_R_C);
        float _D_D=whichMAX_(3,_R_D);

        _DBI=(_D_A+_D_B+_D_C+_D_D)/4;

        break;
    }
    case FOCUS:
    {
        DPoint _dribble_pos=gazebo_info_.BallInfo_.ball_global_loc_;
        char   _dribble_id=0;
        vector<DPoint> _robots_pos;

        if(gazebo_info_.RobotInfo_.is_robot_dribble_)
        {
            _dribble_pos=gazebo_info_.RobotInfo_.robot_loc_;
            _dribble_id=gazebo_info_.RobotInfo_.robot_id_;
        }
        else
            for(int i=0; i<gazebo_info_.Teammates_.size(); i++)
                if(gazebo_info_.Teammates_[i].is_robot_dribble_)
                {
                    _dribble_pos=gazebo_info_.Teammates_[i].robot_loc_;
                    _dribble_id=gazebo_info_.Teammates_[i].robot_id_;
                    break;
                }

        if(!gazebo_info_.RobotInfo_.is_robot_dribble_)
            _robots_pos.push_back(gazebo_info_.RobotInfo_.robot_loc_);
        for(int i=0; i<gazebo_info_.Teammates_.size(); i++)
            if(!gazebo_info_.Teammates_[i].is_robot_dribble_)
                _robots_pos.push_back(gazebo_info_.Teammates_[i].robot_loc_);

        vector<DPoint> _group_A,_group_B,_group_C,_group_D;
        //group A is composed with dribble robot and all opponents
        _group_A.push_back(_dribble_pos);
        for(int i=0; i<_tmp_opponents_pos.size(); i++)
            _group_A.push_back(_tmp_opponents_pos[i]);

        //group B,C,D just has one our robot
        _group_B.push_back(_robots_pos[0]);
        _group_C.push_back(_robots_pos[1]);
        _group_D.push_back(_robots_pos[2]);

        float _R_AB=calculateR_(_group_A,_group_B);
        float _R_AC=calculateR_(_group_A,_group_C);
        float _R_AD=calculateR_(_group_A,_group_D);
        float _R_BC=calculateR_(_group_B,_group_C);
        float _R_BD=calculateR_(_group_B,_group_D);
        float _R_CD=calculateR_(_group_C,_group_D);

        float _R_A[3]={_R_AB,_R_AC,_R_AD};
        float _R_B[3]={_R_AB,_R_BC,_R_BD};
        float _R_C[3]={_R_AC,_R_BC,_R_CD};
        float _R_D[3]={_R_AD,_R_BD,_R_CD};

        float _D_A=whichMAX_(3,_R_A);
        float _D_B=whichMAX_(3,_R_B);
        float _D_C=whichMAX_(3,_R_C);
        float _D_D=whichMAX_(3,_R_D);

        _DBI=(_D_A+_D_B+_D_C+_D_D)/4;

        break;
    }
    }

    return _DBI;
}

float Robot_Control::calculateR_(vector<DPoint> _group_1, vector<DPoint> _group_2)
{
    DPoint _sum_point_1=DPoint(0,0);
    DPoint _sum_point_2=DPoint(0,0);
    for(int i=0; i<_group_1.size(); i++)
        _sum_point_1=_sum_point_1+_group_1[i];
    for(int i=0; i<_group_2.size(); i++)
        _sum_point_2=_sum_point_2+_group_2[i];
    DPoint _center_1=DPoint(_sum_point_1.x_/_group_1.size(),_sum_point_1.y_/_group_1.size());
    DPoint _center_2=DPoint(_sum_point_2.x_/_group_2.size(),_sum_point_2.y_/_group_2.size());
    float _M12=_center_1.distance(_center_2);

    float _sum_dis_1=0;
    float _sum_dis_2=0;
    for(int i=0; i<_group_1.size(); i++)
        _sum_dis_1=_sum_dis_1+_group_1[i].distance(_center_1);
    for(int i=0; i<_group_2.size(); i++)
        _sum_dis_2=_sum_dis_2+_group_2[i].distance(_center_2);
    float _S1=sqrt(_sum_dis_1/_group_1.size());
    float _S2=sqrt(_sum_dis_2/_group_2.size());

    return (_S1+_S2)/_M12;
}

//float Robot_Control::calculatePayoff_(vector<DPoint> _our_positions, vector<DPoint> _opp_positions)
//{

//}

float Robot_Control::whichMAX_(int size,float *_list)
{
    float max_out=_list[0];
    for(int i=1;i<size;i++)
    {
        if(_list[i]>max_out)
            max_out=_list[i];
    }
    return max_out;
}

void Robot_Control::getConvexhull_(vector<DPoint> &_convex_hull, vector<DPoint> _tmp_pos)
{
    //first point is the point with least y
    float _least_y=600;
    int   _least_label=-1;
    for(int i=0; i<_tmp_pos.size(); i++)
        if(_tmp_pos[i].y_<_least_y)
        {
            _least_y=_tmp_pos[i].y_;
            _least_label=i;
        }

    if(_least_label!=-1)
        _convex_hull.push_back(_tmp_pos[_least_label]);
    else
    {
        ROS_ERROR("Invalid opponents' positions");
        return;
    }

    //the other points
    DPoint _init_direction=DPoint(1,0);
    float  _least_angel=M_PI;
    float  _angle;
    do
    {
        _least_angel=M_PI;
        _least_label=-1;
        if(_convex_hull.size()>1)
            _init_direction=_convex_hull.back()-*(_convex_hull.end()-2);
        for(int i=0; i<_tmp_pos.size();i++)
        {
            _angle=ThetaOf2Vector(_tmp_pos[i]-_convex_hull.back(),_init_direction);
            if(_angle<_least_angel)
            {
                _least_angel=_angle;
                _least_label=i;
            }
        }
        if(_least_label!=-1&&_tmp_pos[_least_label]!=_convex_hull.front())
            _convex_hull.push_back(_tmp_pos[_least_label]);
    }while(_tmp_pos[_least_label]!=_convex_hull.front());
}

void Robot_Control::calculateArea_(vector<float> &_p_strategy, vector<DPoint> _convex_hull)
{
    //find the min_x and max_x
    float _min_x=900;
    float _max_x=-900;
    vector<DPoint> _left_points;
    vector<DPoint> _middle_points;
    vector<DPoint> _right_points;

    for(int i=0; i<_convex_hull.size(); i++)
        if(_convex_hull[i].x_<_min_x)
            _min_x=_convex_hull[i].x_;
    for(int i=0; i<_convex_hull.size(); i++)
        if(_convex_hull[i].x_>_max_x)
            _max_x=_convex_hull[i].x_;

    if(_max_x>=200)
    {
        if(_min_x>=200)
        {
            _p_strategy.push_back(0);
            _p_strategy.push_back(0);
            _p_strategy.push_back(1);
        }
        else if(_min_x>=-200)
        {            
            for(int i=0; i<_convex_hull.size(); i++)
            {
                if(_convex_hull[i].x_>=200)
                    _right_points.push_back(_convex_hull[i]);
                else
                    _left_points.push_back(_convex_hull[i]);

                DPoint _tmp_p;
                if((i!=_convex_hull.size()-1)&&
                   ((_convex_hull[i].x_<200&&_convex_hull[i+1].x_>=200)||
                    (_convex_hull[i].x_>=200&&_convex_hull[i+1].x_<200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[i+1],200);
                    _left_points.push_back(_tmp_p);
                    _right_points.push_back(_tmp_p);
                }
                else if((i==_convex_hull.size()-1)&&
                        ((_convex_hull[i].x_<200&&_convex_hull[0].x_>=200)||
                         (_convex_hull[i].x_>=200&&_convex_hull[0].x_<200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[0],200);
                    _left_points.push_back(_tmp_p);
                    _right_points.push_back(_tmp_p);
                }
            }

            vector<DPoint> _left_modify;
            vector<DPoint> _right_modify;
            getConvexhull_(_left_modify,_left_points);
            getConvexhull_(_right_modify,_right_points);

            _p_strategy.push_back(0);
            _p_strategy.push_back(AreaofPolygon(_left_modify));
            _p_strategy.push_back(AreaofPolygon(_right_modify));
        }
        else
        {
            for(int i=0; i<_convex_hull.size(); i++)
            {
                if(_convex_hull[i].x_<-200)
                    _left_points.push_back(_convex_hull[i]);
                else if(_convex_hull[i].x_<200)
                    _middle_points.push_back(_convex_hull[i]);
                else
                    _right_points.push_back(_convex_hull[i]);

                DPoint _tmp_p;
                if((i!=_convex_hull.size()-1)&&
                   ((_convex_hull[i].x_<200&&_convex_hull[i+1].x_>=200)||
                    (_convex_hull[i].x_>=200&&_convex_hull[i+1].x_<200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[i+1],200);
                    _middle_points.push_back(_tmp_p);
                    _right_points.push_back(_tmp_p);
                }
                else if((i==_convex_hull.size()-1)&&
                        ((_convex_hull[i].x_<200&&_convex_hull[0].x_>=200)||
                         (_convex_hull[i].x_>=200&&_convex_hull[0].x_<200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[0],200);
                    _middle_points.push_back(_tmp_p);
                    _right_points.push_back(_tmp_p);
                }

                if((i!=_convex_hull.size()-1)&&
                   ((_convex_hull[i].x_<-200&&_convex_hull[i+1].x_>=-200)||
                    (_convex_hull[i].x_>=-200&&_convex_hull[i+1].x_<-200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[i+1],-200);
                    _left_points.push_back(_tmp_p);
                    _middle_points.push_back(_tmp_p);
                }
                else if((i==_convex_hull.size()-1)&&
                        ((_convex_hull[i].x_<-200&&_convex_hull[0].x_>=-200)||
                         (_convex_hull[i].x_>=-200&&_convex_hull[0].x_<-200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[0],-200);
                    _left_points.push_back(_tmp_p);
                    _middle_points.push_back(_tmp_p);
                }
            }

            vector<DPoint> _left_modify;
            vector<DPoint> _middle_modify;
            vector<DPoint> _right_modify;
            getConvexhull_(_left_modify,_left_points);
            getConvexhull_(_middle_modify,_left_points);
            getConvexhull_(_right_modify,_right_points);

            _p_strategy.push_back(AreaofPolygon(_left_modify));
            _p_strategy.push_back(AreaofPolygon(_middle_modify));
            _p_strategy.push_back(AreaofPolygon(_right_modify));
        }
    }
    else if(_max_x>=-200)
    {
        if(_min_x>=-200)
        {
            _p_strategy.push_back(0);
            _p_strategy.push_back(1);
            _p_strategy.push_back(0);
        }
        else
        {
            for(int i=0; i<_convex_hull.size(); i++)
            {
                if(_convex_hull[i].x_>=-200)
                    _right_points.push_back(_convex_hull[i]);
                else
                    _left_points.push_back(_convex_hull[i]);

                DPoint _tmp_p;
                if((i!=_convex_hull.size()-1)&&
                   ((_convex_hull[i].x_<-200&&_convex_hull[i+1].x_>=-200)||
                    (_convex_hull[i].x_>=-200&&_convex_hull[i+1].x_<-200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[i+1],-200);
                    _left_points.push_back(_tmp_p);
                    _right_points.push_back(_tmp_p);
                }
                else if((i==_convex_hull.size()-1)&&
                        ((_convex_hull[i].x_<-200&&_convex_hull[0].x_>=-200)||
                         (_convex_hull[i].x_>=-200&&_convex_hull[0].x_<-200)))
                {
                    _tmp_p=PointOfLine_XorY(_convex_hull[i],_convex_hull[0],-200);
                    _left_points.push_back(_tmp_p);
                    _right_points.push_back(_tmp_p);
                }
            }

            vector<DPoint> _left_modify;
            vector<DPoint> _right_modify;
            getConvexhull_(_left_modify,_left_points);
            getConvexhull_(_right_modify,_right_points);

            _p_strategy.push_back(AreaofPolygon(_left_modify));
            _p_strategy.push_back(AreaofPolygon(_right_modify));
            _p_strategy.push_back(0);
        }
    }
    else
    {
        _p_strategy.push_back(1);
        _p_strategy.push_back(0);
        _p_strategy.push_back(0);
    }
}

void Robot_Control::singleControl_()
{
    switch (bcicontrol_info_.robotMode)
    {
    case UP_MAXSPEED:
        if(bcicontrol_info_.selectRobot-10==gazebo_info_.myID)
            max_vel_+=50;
        break;
    case DOWN_MAXSPEED:
        if(bcicontrol_info_.selectRobot-10==gazebo_info_.myID)
            max_vel_= max_vel_>50 ? max_vel_-50:0;
        break;
    case CHANGE_ROBOT:
    {
        if(!gazebo_info_.RobotInfo_.is_robot_valid_)
        {
            char _tmp_role=SUBSTITUTE;
            for(int i=0;i<gazebo_info_.Teammates_.size();i++)
                if(gazebo_info_.Teammates_[i].robot_id_==bcicontrol_info_.selectRobot-9)
                {
                    _tmp_role=gazebo_info_.Teammates_[i].current_role_;
                    break;
                }
            gazebo_info_.RobotInfo_.is_robot_valid_=true;
            gazebo_info_.RobotInfo_.current_role_=_tmp_role;
            start_move_=ros::Time::now();
        }
        else if(bcicontrol_info_.selectRobot-9==gazebo_info_.myID)
        {
            gazebo_info_.RobotInfo_.current_role_=SUBSTITUTE;
            gazebo_info_.RobotInfo_.is_robot_valid_=false;
            start_move_=ros::Time::now();
        }
        break;
    }
    }
}

void Robot_Control::normalization_(vector<float> &_ori_value)
{
    float _tmp_sum=0;
    for(int i=0; i<_ori_value.size(); i++)
        _tmp_sum+=_ori_value[i];
    for(int i=0; i<_ori_value.size(); i++)
        _ori_value[i]=_ori_value[i]/_tmp_sum;
}

void Robot_Control::pubStrategy_()
{
    nubot_common::StrategyInfo _tmp_info;
    _tmp_info.recommend_strategy=recommend_strategy_;

    strategy_pub_.publish(_tmp_info);
}

void Robot_Control::pubInterfaceinfo_()
{
    nubot_common::InterfaceInfo _tmp_info;

    _tmp_info.robotinfo.robotID=gazebo_info_.RobotInfo_.robot_id_;
    _tmp_info.robotinfo.pos.x=gazebo_info_.RobotInfo_.robot_loc_.x_;
    _tmp_info.robotinfo.pos.y=gazebo_info_.RobotInfo_.robot_loc_.y_;
    _tmp_info.robotinfo.heading.theta=gazebo_info_.RobotInfo_.robot_head_.radian_;
    _tmp_info.robotinfo.vtrans.x=gazebo_info_.RobotInfo_.robot_vec_.x_;
    _tmp_info.robotinfo.vtrans.y=gazebo_info_.RobotInfo_.robot_vec_.y_;
    _tmp_info.robotinfo.vrot=gazebo_info_.RobotInfo_.robot_w_;
    _tmp_info.robotinfo.isvalid=gazebo_info_.RobotInfo_.is_robot_valid_;
    _tmp_info.robotinfo.isdribble=gazebo_info_.RobotInfo_.is_robot_dribble_;
    _tmp_info.robotinfo.islocation=gazebo_info_.RobotInfo_.is_robot_location_;
    _tmp_info.robotinfo.currentRole=gazebo_info_.RobotInfo_.current_role_;
    _tmp_info.robotinfo.currentAction=gazebo_info_.RobotInfo_.current_action_;
    _tmp_info.robotinfo.batteryState=gazebo_info_.RobotInfo_.battery_state_;

    interface_pub_.publish(_tmp_info);
}

void Robot_Control::stopRobot_()
{
    m_behaviour_->app_vx_ = m_behaviour_->app_vy_ =  0.0;
    m_behaviour_->app_w_  =  0;
}

void Robot_Control::catchBall_()
{
    if(gazebo_info_.RobotInfo_.is_robot_dribble_)
    {
        gazebo_info_.RobotInfo_.is_robot_location_=true;
        return;
    }

    double thetaofr2b = thetaof2p(gazebo_info_.RobotInfo_.robot_loc_,gazebo_info_.BallInfo_.ball_global_loc_);
    if(gazebo_info_.BallInfo_.ball_global_loc_.distance(gazebo_info_.RobotInfo_.robot_loc_)>100)
        m_behaviour_->move2Positionwithobs(1,2,gazebo_info_.BallInfo_.ball_global_loc_,max_vel_,gazebo_info_.RobotInfo_.robot_loc_,gazebo_info_.RobotInfo_.robot_head_);
    else
        m_behaviour_->move2target(1,2,gazebo_info_.BallInfo_.ball_global_loc_,DPoint(0,0),max_vel_,gazebo_info_.RobotInfo_.robot_loc_,gazebo_info_.RobotInfo_.robot_head_);
    m_behaviour_->rotate2AbsOrienation(1,2,thetaofr2b,10,gazebo_info_.RobotInfo_.robot_head_);
}

bool Robot_Control::position_(DPoint _target)
{
    ros::Duration _duration=ros::Time::now()-start_move_;
    if(_duration.sec>10)
        return true;
    double thetaofr2b = thetaof2p(gazebo_info_.RobotInfo_.robot_loc_,gazebo_info_.BallInfo_.ball_global_loc_);
    if(gazebo_info_.RobotInfo_.current_role_==SUBSTITUTE)
        thetaofr2b=0;

    double theta_d = gazebo_info_.RobotInfo_.robot_head_.radian_-thetaofr2b;
    if(_target.distance(gazebo_info_.RobotInfo_.robot_loc_)<=50&&fabs(theta_d)<=0.2)
        return true;
    if(_target.distance(gazebo_info_.RobotInfo_.robot_loc_)>50)
        m_behaviour_->move2Positionwithobs(1,2,_target,max_vel_,gazebo_info_.RobotInfo_.robot_loc_,gazebo_info_.RobotInfo_.robot_head_);
    if(fabs(theta_d)>0.1)
        m_behaviour_->rotate2AbsOrienation(1,2,thetaofr2b,10,gazebo_info_.RobotInfo_.robot_head_);
    return false;
}
