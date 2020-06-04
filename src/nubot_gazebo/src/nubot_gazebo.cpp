// NOTICE:
// GAZEBO uses ISO units, i.e. length uses meters.
// but other code uses cm as the length unit, so for publishing
// and subscribing messages, length unit should be changed to 'cm'

#include <algorithm>
#include <assert.h>
#include <cmath>
#include "nubot_gazebo.h"
#include "vector_angle.h"

#define RUN 1
#define FLY -1
#define ZERO_VECTOR ignition::math::Vector3d::Zero
#define PI 3.14159265

enum {NOTSEEBALL = 0, SEEBALLBYOWN = 1,SEEBALLBYOTHERS = 2};
const ignition::math::Vector3d kick_vector_robot(1,0,0);    // assume the normalized vector from origin to kicking mechanism in robot refercence frame
                                                 // is in x-axis direction
const double goal_x = 9.0;
const double goal_height = 1.0;

using namespace gazebo;
using namespace std;
GZ_REGISTER_MODEL_PLUGIN(NubotGazebo)

NubotGazebo::NubotGazebo()
{
    // Variables initialization
    desired_rot_vector_ = ZERO_VECTOR;
    desired_trans_vector_ = ZERO_VECTOR;
    nubot_ball_vec_ =ignition::math::Vector3d(1,0,0);
    kick_vector_world_ = kick_vector_robot;
    nubot_ball_vec_len_ = 1;
    ball_index_=robot_index_=0;
    Vx_cmd_=Vy_cmd_=w_cmd_=0;
    force_ = 0.0; mode_=1;
    can_move_ = true;

    model_count_ = 0;
    dribble_req_ = false;
    is_dribble_ = false;
    shot_req_ = false;
    ModelStatesCB_flag_ = false;
    judge_nubot_stuck_ = false;
    is_kick_ = false;
    flip_cord_ = false;
    AgentID_ = 0;
    noise_scale_ = 0.0;
    noise_rate_ = 0.0;
    state_ = CHASE_BALL;
    sub_state_ = MOVE_BALL;

    obs_ = new Obstacles();
    if(!obs_)
        ROS_FATAL("Cannot allocate memory to type Obstacles!");

    // Resize message fields
    model_states_.name.reserve(20);
    model_states_.pose.reserve(20);
    model_states_.twist.reserve(20);

    dribble_P_ = 0.5;
    dribble_I_ = 0;
    dribble_D_ = 0;
    I_term_max_= 10;
    I_term_min_= 0;
}

NubotGazebo::~NubotGazebo()
{
//    event::Events::DisconnectWorldUpdateBegin(update_connection_);
    // Removes all callbacks from the queue. Does not wait for calls currently in progress to finish.
    message_queue_.clear();
    service_queue_.clear();
    // Disable the queue, meaning any calls to addCallback() will have no effect.
    message_queue_.disable();
    service_queue_.disable();
    rosnode_->shutdown();                     // This MUST BE CALLED before thread join()!!
    message_callback_queue_thread_.join();
    service_callback_queue_thread_.join();

    delete rosnode_;
    delete obs_;
}

void NubotGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Get the world name.
    world_ = _model->GetWorld();
    robot_model_ = _model;
    model_name_ = robot_model_->GetName();
    robot_namespace_ = robot_model_->GetName();

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libnubot_gazebo.so' in the gazebo_ros package)");
        return;
    }

    rosnode_ = new ros::NodeHandle(robot_namespace_);
    rosnode_->param<std::string>("/football/name",                   ball_name_,             std::string("football") );
    rosnode_->param<std::string>("/football/chassis_link",           ball_chassis_,          std::string("football::ball") );
    rosnode_->param<std::string>("/cyan/prefix",                     cyan_pre_,              std::string("nubot"));
    rosnode_->param<std::string>("/magenta/prefix",                  mag_pre_,               std::string("rival"));
    rosnode_->param<double>("/general/dribble_distance_thres",       dribble_distance_thres_,    0.50);
    rosnode_->param<double>("/general/dribble_angle_thres",          dribble_angle_thres_,       30.0);
    rosnode_->param<double>("/field/length",                         field_length_,              18.0);
    rosnode_->param<double>("/field/width",                          field_width_,               12.0);
    rosnode_->param<double>("/general/noise_scale",                  noise_scale_,               0.10);
    rosnode_->param<double>("/general/noise_rate",                   noise_rate_,                0.01);

    if(!_sdf->HasElement("flip_cord"))
    {
        ROS_INFO("NubotGazebo plugin missing <flip_cord>, defaults to false");
        flip_cord_ = false;
    }
    else
        flip_cord_ = _sdf->GetElement("flip_cord")->Get<bool>();

    if(!flip_cord_)
        AgentID_ = atoi( model_name_.substr(cyan_pre_.size(),1).c_str() );    // get the robot id
    else
        AgentID_ = atoi( model_name_.substr(mag_pre_.size(),1).c_str() );    // get the robot id

    // Load the football model
    ball_model_ = world_->ModelByName(ball_name_);
    if (!ball_model_)
        ROS_ERROR("model [%s] does not exist", ball_name_.c_str());
    else
    {
        ball_link_ = ball_model_->GetLink(ball_chassis_);
        if(!ball_link_)
            ROS_ERROR("link [%s] does not exist!", ball_chassis_.c_str());
    }

    // Publishers
    gazebo_info_pub_ = rosnode_->advertise<nubot_common::GazeboInfo>("nubot_gazebo/gazebo_info",10);

    // Subscribers.
    ros::SubscribeOptions so1 = ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(
                "/gazebo/model_states", 100, boost::bind( &NubotGazebo::model_states_CB,this,_1),
                ros::VoidPtr(), &message_queue_);
    ModelStates_sub_ = rosnode_->subscribe(so1);

    ros::SubscribeOptions so2 = ros::SubscribeOptions::create<nubot_common::VelCmd>(
                "nubotcontrol/velcmd", 100, boost::bind( &NubotGazebo::vel_cmd_CB,this,_1),
                ros::VoidPtr(), &message_queue_);
    Velcmd_sub_ = rosnode_->subscribe(so2);

    // Service Servers & clients
    ros::AdvertiseServiceOptions aso1 = ros::AdvertiseServiceOptions::create<nubot_common::BallHandle>(
                "BallHandle", boost::bind(&NubotGazebo::ball_handle_control_service, this, _1, _2),
                ros::VoidPtr(), &service_queue_);
    ballhandle_server_ =   rosnode_->advertiseService(aso1);

    ros::AdvertiseServiceOptions aso2 = ros::AdvertiseServiceOptions::create<nubot_common::Shoot>(
                "Shoot", boost::bind(&NubotGazebo::shoot_control_servive, this, _1, _2),
                ros::VoidPtr(), &service_queue_);
    shoot_server_ =   rosnode_->advertiseService(aso2);
    dribbleId_client_ = rosnode_->serviceClient<nubot_common::Dribble>("/DribbleId");

    // Custom Callback Queue Thread. Use threads to process message and service callback queue
    message_callback_queue_thread_ = boost::thread( boost::bind( &NubotGazebo::message_queue_thread,this ) );
    service_callback_queue_thread_ = boost::thread( boost::bind( &NubotGazebo::service_queue_thread,this ) );

    // This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&NubotGazebo::update_child, this));

    // Output info
    ROS_INFO(" %s  id: %d  flip_cord:%d  gaussian scale: %f  rate: %f\n",
              model_name_.c_str(),  AgentID_, flip_cord_, noise_scale_, noise_rate_);
}

void NubotGazebo::Reset()
{
    ROS_DEBUG("%s Reset() running now!", model_name_.c_str());

    // Variables initialization
    desired_rot_vector_ = ZERO_VECTOR;
    desired_trans_vector_ = ZERO_VECTOR;
    nubot_ball_vec_ = ignition::math::Vector3d(1,0,0);
    kick_vector_world_ = kick_vector_robot;
    nubot_ball_vec_len_ = 1;
    Vx_cmd_=Vy_cmd_=w_cmd_=0;
    force_ = 0.0; mode_=1;
    can_move_ = true;

    dribble_req_ = false;
    is_dribble_ = false;
    shot_req_ = false;
    ModelStatesCB_flag_ = false;
    judge_nubot_stuck_ = false;
    is_kick_ = false;
    state_ = CHASE_BALL;
    sub_state_ = MOVE_BALL;

}

void NubotGazebo::message_queue_thread()
{
    static const double timeout = 0.01;
    while (rosnode_->ok())
    {
        // Invoke all callbacks currently in the queue. If a callback was not ready to be called,
        // pushes it back onto the queue. This version includes a timeout which lets you specify
        // the amount of time to wait for a callback to be available before returning.
        message_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void NubotGazebo::service_queue_thread()
{
    static const double timeout = 0.01;
    while (rosnode_->ok())
        service_queue_.callAvailable(ros::WallDuration(timeout));
}

/*
void NubotGazebo::config(nubot_gazebo::NubotGazeboConfig &config, uint32_t level)
{
    dribble_P_      = config.P;
    dribble_I_      = config.I;
    dribble_D_      = config.D;
    I_term_max_     = config.I_max;
    I_term_min_     = config.I_min;
    ROS_FATAL("Reconfig request: P:%f I:%f D:%f I_term_max:%f I_term_min:%f",
              dribble_P_, dribble_I_, dribble_D_, I_term_max_, I_term_min_);
}*/

void NubotGazebo::model_states_CB(const gazebo_msgs::ModelStates::ConstPtr& _msg)
{
    msgCB_lock_.lock();
    ModelStatesCB_flag_ = true;
    model_count_ = 0;
    model_states_.name.clear();
    model_states_.pose.clear();
    model_states_.twist.clear();

    for(int i=0; i<world_->ModelCount() ;i++)
    {
        // get info of robots and the ball; reference frame: world
        if( (_msg->name[i].find(cyan_pre_) != std::string::npos) ||
            (_msg->name[i].find(mag_pre_) != std::string::npos) ||
            (_msg->name[i] == ball_name_) )
        {
            geometry_msgs::Pose     ps = _msg->pose[i];
            geometry_msgs::Twist    tw = _msg->twist[i];
            ps.position.x += noise(noise_scale_, noise_rate_);  // add gaussian noise
            ps.position.y += noise(noise_scale_, noise_rate_);
            // ps.position.z += noise(noise_scale_, noise_rate_);
            tw.linear.x   += noise(noise_scale_, noise_rate_);
            tw.linear.y   += noise(noise_scale_, noise_rate_);
            // tw.linear.z   += noise(noise_scale_, noise_rate_);

            if(flip_cord_)      // rival robot model
            {
                // We only have to change the sign of x and y positions, and x and y velocities;
                // Since the coordinate frame of the rival model has been flipped, we don't have
                // to change the orientation here.
                ps.position.x *= -1.0;
                ps.position.y *= -1.0;
                tw.linear.x *= -1.0;
                tw.linear.y *= -1.0;
            }

            model_states_.name.push_back(_msg->name[i]);
            model_states_.pose.push_back(ps);
            model_states_.twist.push_back(tw);

            if(model_states_.name.back() == ball_name_ )
                ball_index_ =  model_count_;
            else if(model_states_.name.back() == model_name_)
                robot_index_ = model_count_;

            model_count_ ++;
        }
    }
    msgCB_lock_.unlock();
}

bool NubotGazebo::update_model_info(void)
{
    if(ModelStatesCB_flag_)
    {
        // Get football and nubot's pose and twist
        ball_state_.model_name = ball_name_ ;
        ball_state_.pose.position.X(model_states_.pose[ball_index_].position.x);
        ball_state_.pose.position.Y(model_states_.pose[ball_index_].position.y);
        ball_state_.pose.position.Z(model_states_.pose[ball_index_].position.z);
        ball_state_.pose.orient.W(model_states_.pose[ball_index_].orientation.w);
        ball_state_.pose.orient.X(model_states_.pose[ball_index_].orientation.x);
        ball_state_.pose.orient.Y(model_states_.pose[ball_index_].orientation.y);
        ball_state_.pose.orient.Z(model_states_.pose[ball_index_].orientation.z);
        ball_state_.twist.linear.X(model_states_.twist[ball_index_].linear.x);
        ball_state_.twist.linear.Y(model_states_.twist[ball_index_].linear.y);
        ball_state_.twist.linear.Z(model_states_.twist[ball_index_].linear.z);
        ball_state_.twist.angular.X(model_states_.twist[ball_index_].angular.x);
        ball_state_.twist.angular.Y(model_states_.twist[ball_index_].angular.y);
        ball_state_.twist.angular.Z(model_states_.twist[ball_index_].angular.z);

        robot_state_.model_name = model_name_ ;
        robot_state_.pose.position.X(model_states_.pose[robot_index_].position.x);
        robot_state_.pose.position.Y(model_states_.pose[robot_index_].position.y);
        robot_state_.pose.position.Z(model_states_.pose[robot_index_].position.z);
        robot_state_.pose.orient.W(model_states_.pose[robot_index_].orientation.w);
        robot_state_.pose.orient.X(model_states_.pose[robot_index_].orientation.x);
        robot_state_.pose.orient.Y(model_states_.pose[robot_index_].orientation.y);
        robot_state_.pose.orient.Z(model_states_.pose[robot_index_].orientation.z);
        robot_state_.twist.linear.X(model_states_.twist[robot_index_].linear.x);
        robot_state_.twist.linear.Y(model_states_.twist[robot_index_].linear.y);
        robot_state_.twist.linear.Z(model_states_.twist[robot_index_].linear.z);
        robot_state_.twist.angular.X(model_states_.twist[robot_index_].angular.x);
        robot_state_.twist.angular.Y(model_states_.twist[robot_index_].angular.y);
        robot_state_.twist.angular.Z(model_states_.twist[robot_index_].angular.z);

        // calculate vector from nubot to football
        nubot_ball_vec_ = ball_state_.pose.position - robot_state_.pose.position;
        nubot_ball_vec_len_ = nubot_ball_vec_.Length();

        // transform kick_vector_nubot in world frame
        ignition::math::Quaterniond    rotation_quaternion = robot_state_.pose.orient;
        ignition::math::Matrix3d       RotationMatrix3(rotation_quaternion);
        kick_vector_world_ = RotationMatrix3 * kick_vector_robot; // vector from nubot origin to kicking mechanism in world frame
        // ROS_INFO("kick_vector_world_: %f %f %f",kick_vector_world_.x, kick_vector_world_.y, kick_vector_world_.z);

        return 1;
    }
    else
    {
        ROS_INFO("%s update_model_info(): Waiting for model_states messages!", model_name_.c_str());
        return 0;
    }
}

double NubotGazebo::noise(double scale, double probability)
{
    if(ignition::math::equal<double>(scale, 0.0))
        return 0.0;
    else
    {
        if(rand_.IntUniform(0,10) <= int(10.0*probability))
            return scale*rand_.DblNormal(0,1);
        else
            return 0.0;
    }
}

void NubotGazebo::message_publish(void)
{
    nubot_common::GazeboInfo _gazebo_info;
    _gazebo_info.header.stamp=ros::Time::now();
    for(int i=0; i<model_count_;i++)
    {
        // ball info
        if(model_states_.name[i].compare(0, ball_name_.size(), ball_name_) == 0)
        {
            geometry_msgs::Pose  _ball_pose = model_states_.pose[i];
            geometry_msgs::Twist _ball_twist = model_states_.twist[i];
            _gazebo_info.ballinfo.header.stamp=ros::Time::now();
            _gazebo_info.ballinfo.header.seq++;
            _gazebo_info.ballinfo.pos.x=_ball_pose.position.x*M2CM_CONVERSION;
            _gazebo_info.ballinfo.pos.y=_ball_pose.position.y*M2CM_CONVERSION;
            _gazebo_info.ballinfo.velocity.x=_ball_twist.linear.x*M2CM_CONVERSION;
            _gazebo_info.ballinfo.velocity.y=_ball_twist.linear.y*M2CM_CONVERSION;
        }
        // robots info
        else if(model_states_.name[i].compare(0, cyan_pre_.size(), cyan_pre_)==0)
        {
            int _robot_id = atoi(model_states_.name[i].substr(cyan_pre_.size(),1).c_str());

            nubot_common::RobotInfo _robot_info;
            geometry_msgs::Pose  _robot_pose  = model_states_.pose[i];
            geometry_msgs::Twist _robot_twist = model_states_.twist[i];
            ignition::math::Quaterniond _rot_qua(_robot_pose.orientation.w, _robot_pose.orientation.x,
                                                 _robot_pose.orientation.y, _robot_pose.orientation.z);
            double _heading_theta = _rot_qua.Yaw();
            _robot_info.header.stamp  =ros::Time::now();
            _robot_info.header.seq++;
            _robot_info.robotID       = _robot_id;
            _robot_info.pos.x         = _robot_pose.position.x*M2CM_CONVERSION;
            _robot_info.pos.y         = _robot_pose.position.y*M2CM_CONVERSION;
            _robot_info.heading.theta = _heading_theta;
            _robot_info.vrot          = _robot_twist.angular.z;
            _robot_info.vtrans.x      = _robot_twist.linear.x*M2CM_CONVERSION;
            _robot_info.vtrans.y      = _robot_twist.linear.y*M2CM_CONVERSION;
            _robot_info.isvalid       = is_robot_valid(_robot_pose.position.x,_robot_pose.position.y);

            _gazebo_info.robotinfo.push_back(_robot_info);
        }
        // opponent info
        if(model_states_.name[i].compare(0, mag_pre_.size(), mag_pre_)==0)
        {
            int _opponent_id = atoi(model_states_.name[i].substr(mag_pre_.size(),1).c_str());

            nubot_common::OpponentInfo _opponent_info;
            geometry_msgs::Pose  _opponent_pose  = model_states_.pose[i];
            geometry_msgs::Twist _opponent_twist = model_states_.twist[i];
            ignition::math::Quaterniond _rot_qua(_opponent_pose.orientation.w, _opponent_pose.orientation.x,
                                                 _opponent_pose.orientation.y, _opponent_pose.orientation.z);
            double _heading_theta = _rot_qua.Yaw();
            _opponent_info.header.stamp  = ros::Time::now();
            _opponent_info.header.seq++;
            _opponent_info.opponentID    = _opponent_id;
            _opponent_info.pos.x         = _opponent_pose.position.x*M2CM_CONVERSION;
            _opponent_info.pos.y         = _opponent_pose.position.y*M2CM_CONVERSION;
            _opponent_info.heading.theta = _heading_theta;
            _opponent_info.vrot          = _opponent_twist.angular.z;
            _opponent_info.vtrans.x      = _opponent_twist.linear.x*M2CM_CONVERSION;
            _opponent_info.vtrans.y      = _opponent_twist.linear.y*M2CM_CONVERSION;
            _opponent_info.isvalid       = is_robot_valid(_opponent_pose.position.x,_opponent_pose.position.y);

            _gazebo_info.opponentinfo.push_back(_opponent_info);
        }
    }
    gazebo_info_pub_.publish(_gazebo_info);
}

//for test our code
void NubotGazebo::nubot_locomotion(ignition::math::Vector3d linear_vel_vector, ignition::math::Vector3d angular_vel_vector)
{
    desired_trans_vector_ = linear_vel_vector;
    desired_rot_vector_ = angular_vel_vector;

    robot_model_->SetLinearVel(desired_trans_vector_);
    robot_model_->SetAngularVel(desired_rot_vector_);
    judge_nubot_stuck_ = 1;
}

void NubotGazebo::vel_cmd_CB(const nubot_common::VelCmd::ConstPtr& cmd)
{
    msgCB_lock_.lock();

    if(flip_cord_)
    {
        Vx_cmd_ = -cmd->Vx * CM2M_CONVERSION;
        Vy_cmd_ = -cmd->Vy * CM2M_CONVERSION;
    }
    else
    {
        Vx_cmd_ = cmd->Vx * CM2M_CONVERSION;
        Vy_cmd_ = cmd->Vy * CM2M_CONVERSION;
    }
    w_cmd_  = cmd->w;
    ignition::math::Vector3d Vx_nubot = Vx_cmd_ * kick_vector_world_;
    ignition::math::Vector3d Vy_nubot = Vy_cmd_ * (ignition::math::Vector3d(0,0,1).Cross(kick_vector_world_));    // velocity with reference to nubot
    ignition::math::Vector3d linear_vector = Vx_nubot + Vy_nubot;
    ignition::math::Vector3d angular_vector(0,0,w_cmd_);

    //    ROS_FATAL("%s vel_cmd_CB():linear_vector:%f %f %f angular_vector:0 0 %f",model_name_.c_str(),
    //                    linear_vector.x, linear_vector.y, linear_vector.z, angular_vector.z);
    if(can_move_)
        nubot_locomotion(linear_vector, angular_vector);

    msgCB_lock_.unlock();
}

bool NubotGazebo::ball_handle_control_service(nubot_common::BallHandle::Request  &req,
                                              nubot_common::BallHandle::Response &res)
{
    srvCB_lock_.lock();

    dribble_req_ = req.enable ? 1 : 0;         // FIXME. when robot is stucked, req.enable=2
    res.BallIsHolding = get_is_hold_ball();

    // ROS_FATAL("%s dribble:[enable holding]:[%d %d]",model_name_.c_str(), (int)req.enable, (int)res.BallIsHolding);
    srvCB_lock_.unlock();
    return true;
}

bool NubotGazebo::shoot_control_servive( nubot_common::Shoot::Request  &req,
                                         nubot_common::Shoot::Response &res )
{
    srvCB_lock_.lock();

    force_ = double(req.strength);
    mode_ = int(req.ShootPos);
    if(force_ > 15.0)
    {
        //ROS_FATAL("Kick ball force(%f) is too great.", force_);
        force_ = 15.0;
    }
    if( force_ )
    {
        if(get_is_hold_ball())
        {
            dribble_req_ = false;
            shot_req_ = true;
            //ROS_INFO("%s shoot_service: ShootPos:%d strength:%f",model_name_.c_str(), mode_, force_);
            res.ShootIsDone = 1;
        }
        else
        {
            shot_req_ = false;
            res.ShootIsDone = 0;
            //ROS_INFO("%s shoot_service(): Cannot kick ball. angle error:%f distance error: %f. ",
            //                            model_name_.c_str(), angle_error_degree_, nubot_football_vector_length_);
        }
    }
    else
    {
        shot_req_ = false;
        res.ShootIsDone = 1;
        //ROS_ERROR("%s shoot_control_service(): Kick-mechanism charging complete!",model_name_.c_str());
    }

    //ROS_INFO("%s shoot: [strength pos shootisdone]:[%f %d %d]",
    //            model_name_.c_str(), force_, mode_, (int)res.ShootIsDone);

    srvCB_lock_.unlock();
    return true;
}

void NubotGazebo::dribble_ball(void)
{

#if 1
    ignition::math::Quaterniond    target_rot = robot_state_.pose.orient;
    ignition::math::Vector3d       relative_pos = kick_vector_world_* 0.43;
    ignition::math::Vector3d       target_pos;
    if(flip_cord_)
        target_pos = -(robot_state_.pose.position + relative_pos);
    else
        target_pos = robot_state_.pose.position + relative_pos;

    target_pos.Z()=0.12;
    //ROS_INFO("target pos:%f %f %f",target_pos.x, target_pos.y, target_pos.z);
    ignition::math::Pose3d          target_pose(target_pos, target_rot);
    ball_model_->SetLinearVel(ignition::math::Vector3d(0,0,0));
    ball_model_->SetWorldPose(target_pose);
    ball_state_.twist.linear = robot_state_.twist.linear;
#endif
#if 0
    const static double desired_nubot_football_vector_length =  dribble_distance_thres_;
    if(ball_state_.pose.position.z > 0.3)   // if football is in the air, cannot dribble
    {
        ROS_ERROR("dribble_ball(): ball is in the air at %f; return!", ball_state_.pose.position.z);
        return;
    }

    math::Vector3     nubot_linear_vel = nubot_model_->GetWorldLinearVel();
    math::Vector3     nubot_angular_vel = nubot_model_->GetWorldAngularVel();
    nubot_linear_vel.z=0; nubot_angular_vel.x=0; nubot_angular_vel.y=0;
    // Set up the direction from nubot to football. Let vector lies in x-y plane
    nubot_football_vector_.z = 0;                             // don't point to the air
    math::Vector3     perpencular_vel = nubot_angular_vel.Cross(nubot_football_vector_);
    math::Vector3     football_vel = nubot_linear_vel + perpencular_vel;
    football_model_->SetLinearVel(football_vel);

    ROS_INFO("%s dribble_ball(): dribbling ball. ball vel:%f %f", model_name_.c_str(),football_vel.x, football_vel.y);
#endif
}

void NubotGazebo::kick_ball(int mode, double vel=20.0)
{
    ignition::math::Vector3d kick_vector_planar(kick_vector_world_.X(), kick_vector_world_.Y(), 0.0);
    ignition::math::Vector3d vel_vector;

    if(mode == RUN)
    {
        double vel2 = vel * 2.3;;                         //FIXME. CAN TUNE
        if(flip_cord_)
            vel_vector = -kick_vector_planar * vel2;
        else
            vel_vector = kick_vector_planar * vel2;

        ball_model_->SetLinearVel(vel_vector);
        ROS_INFO("kick ball vel:%f vel2:%f",vel, vel2);
    }
    else if(mode == FLY)
    {
        // math formular: y = a*x^2 + b*x + c;
        //  a = -g/(2*vx*vx), c = 0, b = kick_goal_height/D + g*D/(2.0*vx*vx)
        //  mid_point coordinates:[-b/(2*a), (4a*c-b^2)/(4a) ]

        static const double g = 9.8, kick_goal_height = goal_height - 0.40;      // FIXME: can be tuned
        nubot::DPoint point1(robot_state_.pose.position.X(),robot_state_.pose.position.Y());
        nubot::DPoint point2(robot_state_.pose.position.X() + kick_vector_world_.X(),
                             robot_state_.pose.position.Y() + kick_vector_world_.Y());
        nubot::DPoint point3(ball_state_.pose.position.X(),ball_state_.pose.position.Y());
        nubot::Line_ line1(point1, point2);
        nubot::Line_ line2(1.0, 0.0, kick_vector_world_.X()>0 ? -goal_x : goal_x);         // nubot::Line_(A,B,C);

        nubot::DPoint crosspoint;
        line1.crosspoint(line2,crosspoint);
        double D = crosspoint.distance(point3);
        double vx_thres = D*sqrt(g/2/kick_goal_height);
        double vx = vx_thres/2.0;//>vel ? vel : vx_thres/2.0;                            // initial x velocity.CAN BE TUNED
        double b = kick_goal_height/D + g*D/(2.0*vx*vx);

        ROS_INFO("%s crosspoint:(%f %f) vx: %f", model_name_.c_str(),
                 crosspoint.x_, crosspoint.y_, vx);
        if( fabs(crosspoint.y_) < 10)
        {
            ignition::math::Vector3d kick_vector;
            if(flip_cord_)
                kick_vector = ignition::math::Vector3d(-vx*kick_vector_world_.X(), -vx*kick_vector_world_.Y(), b*vx);
            else
                kick_vector = ignition::math::Vector3d(vx*kick_vector_world_.X(), vx*kick_vector_world_.Y(), b*vx);
            ball_model_->SetLinearVel(kick_vector);
        }
        else
            ROS_FATAL("CANNOT SHOOT. crosspoint.y is too big!");
    }
    else
    {
        ROS_ERROR("%s kick_ball(): Incorrect mode!", model_name_.c_str());
    }
}

bool NubotGazebo::get_is_hold_ball(void)
{
    bool near_ball, allign_ball;
    ignition::math::Vector3d norm = nubot_ball_vec_;
    norm.Z(0.0);
    norm.Normalize();
    kick_vector_world_.Z(0.0);
    angle_error_degree_ = get_angle_PI(kick_vector_world_,norm)*(180/PI);

    allign_ball = (angle_error_degree_ <= dribble_angle_thres_/2.0
                   && angle_error_degree_ >= -dribble_angle_thres_/2.0) ?
                1 : 0;
    near_ball = nubot_ball_vec_len_ <= dribble_distance_thres_ ?
                1 : 0;

    //ROS_INFO("%s get_is_hold_ball(): angle error:%f(thres:%f) distance_error:%f(thres:%f)",
    //         model_name_.c_str(),  angle_error_degree_, dribble_angle_thres_,
    //         nubot_football_vector_length_, dribble_distance_thres_);
    return (near_ball && allign_ball);
}

bool NubotGazebo::get_nubot_stuck(void)
{
    static int time_count=0;
    static bool last_time_stuck=0;
    static const int time_limit = 40;
    static bool is_stuck;

    if(judge_nubot_stuck_)
    {
        judge_nubot_stuck_ = 0;
        static const double scale = 0.5;                                    // FIXME. Can tune
        double desired_trans_length = desired_trans_vector_.Length();
        double desired_rot_length   = desired_rot_vector_.Z()>0 ? desired_rot_vector_.Z() : -desired_rot_vector_.Z();
        double actual_trans_length  = robot_state_.twist.linear.Length();
        double actual_rot_length    = robot_state_.twist.angular.Z()>0 ? robot_state_.twist.angular.Z() : -robot_state_.twist.angular.Z();

        //ROS_INFO("%s time_count:%d, last_time_stuck:%d",model_name_.c_str(), time_count, last_time_stuck);
        //ROS_INFO("desired_trans_len:%f actual_trans_len:%f",desired_trans_length,actual_trans_length);
        //ROS_INFO("desired_rot_len:%f actual_rot_len:%f",desired_rot_length, actual_rot_length);

        if(actual_trans_length < desired_trans_length * scale)
        {
            if(last_time_stuck)
                time_count++;
            else
                time_count = 0;

            last_time_stuck = 1;
            if(time_count > time_limit)
            {
                // ROS_INFO("get_nubot_stuck(): desired_trans:%f actual_trans:%f", desired_trans_length, actual_trans_length);
                // ROS_INFO("%s get_nubot_stuck(): cannot translate!", model_name_.c_str());
                time_count = 0;
                is_stuck = 1;
            }
        }
        else if(actual_rot_length < desired_rot_length * scale)
        {
            if(last_time_stuck)
                time_count++;
            else
                time_count = 0;

            last_time_stuck = 1;
            if(time_count > time_limit)
            {
                // ROS_INFO("desired_rot:%f actual_rot:%f", desired_rot_length, actual_rot_length);
                // ROS_ERROR("%s get_nubot_stuck(): cannot rotate!", model_name_.c_str());
                time_count = 0;
                is_stuck = 1;
            }
        }
        else
        {
            last_time_stuck = 0;
            is_stuck = 0;
        }

        return is_stuck;
    }
    else
    {
        //ROS_FATAL("%s judge_nubot_stuck_flag not set!", model_name_.c_str());
        return 0;
    }
}

void NubotGazebo::update_child()
{
    msgCB_lock_.lock(); // lock access to fields that are used in ROS message callbacks
    srvCB_lock_.lock();
    /* delay in model_states messages publishing
     * so after receiving model_states message, then nubot moves. */
    if(update_model_info())
    {
        /********** EDIT BEGINS **********/

        nubot_be_control();
        // nubot_test();

        /**********  EDIT ENDS  **********/
    }
    srvCB_lock_.unlock();
    msgCB_lock_.unlock();
}

void NubotGazebo::nubot_be_control(void)
{
    static nubot_common::Dribble di;
    if(robot_state_.pose.position.Z() < 0.2)          // not in the air
    {
        can_move_ = true;
        if(dribble_req_ && get_is_hold_ball() && match_mode_ != STOPROBOT)
        {
            dribble_ball();
            if(!is_dribble_)                         // only send once
            {
                if(dribbleId_client_.exists())
                {
                    if(!flip_cord_)
                        di.request.AgentId = AgentID_;      // cyan robots
                    else
                        di.request.AgentId = AgentID_ + 5;  // magenta robots

                    if(dribbleId_client_.call(di))
                    {
                        is_dribble_ = true;
                        ROS_INFO("request id:%d", di.request.AgentId);
                    }
                }
            }
        }
        else if(is_dribble_ == true)                    // it means initially the robot dribbled but then could not dribble again
        {
            if(dribbleId_client_.exists())
            {
                di.request.AgentId = -1;                // clear the dribble robot id
                if(dribbleId_client_.call(di))
                    ROS_INFO("clear dribble robot id");
            }
            is_dribble_ = false;
        }

        if(shot_req_ && get_is_hold_ball() && match_mode_ != STOPROBOT)
        {
            kick_ball(mode_, force_);
            shot_req_ = false;
        }
    }
    else
    {
        ROS_FATAL("%s in the air!",model_name_.c_str());
        can_move_ = false;
    }

    message_publish();                          // publish message to world_model node
}

bool NubotGazebo::is_robot_valid(double x, double y)
{
    if(fabs(x) > 10 || fabs(y) > 7)
        return false;
    else
        return true;
}

void NubotGazebo::nubot_test(void)
{
    // dribble ball
#if 0
    nubot_locomotion(math::Vector3(5,0,0),math::Vector3(0,0,2);
            dribble_ball();
    ROS_INFO("nubot-football distance:%f",nubot_football_vector_length_);
#endif
    // kick ball
#if 0
    static bool flag=1;
    if(flag)
    {
        kick_ball(FLY, 20);
        flag = 0;
    }
#endif
    // get nubot stuck flag test
#if 0
    bool a=get_nubot_stuck();
    ROS_FATAL("%d",a);
    nubot_locomotion(math::Vector3(0,0,0),math::Vector3(0,0,1));
#endif
    //for testing velocity decay
#if 0
    static int count=0;
    math::Vector3 vel(3,0,0);
    if(count++<50)
    {
        football_model_->SetLinearVel(vel);
        nubot_model_->SetLinearVel(math::Vector3(2,0,0));
    }
    debug_msgs_.data.clear();
    double data0 = football_model_->GetWorldLinearVel().GetLength();
    debug_msgs_.data.push_back(data0);
    debug_pub_.publish(debug_msgs_);
#endif
    // for testing time duration
#if 0
    common::Time                last_update_time_;
    last_update_time_ = world_->GetSimTime();
    for(int i=0; i<50; i++)
    {
        kick_ball(goal0_pos, mode, force);
        ROS_INFO("%s is kicking ball!",model_name_.c_str());
    }
    common::Time current_time = world_->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    ROS_FATAL("kick time:%f",seconds_since_last_update);
#endif
#if 0
    double vel = nubot_model_->GetWorldLinearVel().GetLength();
    //double vel2 = nubot_state_.twist.linear.x;
    nubot_locomotion(math::Vector3(1,0,0),ZERO_VECTOR);
    //ROS_INFO("function:%f state:%f",vel,vel2);
    debug_msgs_.data.clear();
    debug_msgs_.data.push_back(vel);
    //debug_msgs_.data.push_back(vel2);
    debug_pub_.publish(debug_msgs_);
#endif
}
