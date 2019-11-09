#include "rival_gazebo.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(RivalGazebo)

RivalGazebo::RivalGazebo()
{}

RivalGazebo::~RivalGazebo()
{}

void RivalGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    world_ = _model->GetWorld();
    rival_model_ = _model;
    model_name_ = rival_model_->GetName();

    rosnode_ = new ros::NodeHandle();
    rosnode_->param("/field/length",field_length_,18.0);
    rosnode_->param("/field/width",field_width_,12.0);
    rosnode_->param<std::string>("/magenta/prefix",mag_pre_,std::string("rival"));
    AgentID_ = atoi(model_name_.substr(mag_pre_.size(),1).c_str());

    setinfo_sub_ =rosnode_->subscribe("BCI_control/reset_info", 2, &RivalGazebo::set_robot_pos, this);
    // Output info
    ROS_INFO("%s id: %d\n",model_name_.c_str(),AgentID_);
}

void RivalGazebo::set_robot_pos(const nubot_common::ResetInfo & _msg)
{
    ignition::math::Pose3d _rival_pos = ignition::math::Pose3d::Zero;
    if(fabs(_msg.opp_x[AgentID_-1])*CM2M_CONVERSION>(field_length_/2+100)||fabs(_msg.opp_y[AgentID_-1])*CM2M_CONVERSION>(field_width_/2+100))
    {
        ROS_INFO("invalid positions");
        return;
    }
    else
        _rival_pos.Set(ignition::math::Vector3d(_msg.opp_x[AgentID_-1]*CM2M_CONVERSION,_msg.opp_y[AgentID_-1]*CM2M_CONVERSION,0),ignition::math::Vector3d(0,0,_msg.opp_theta[AgentID_-1]));
    rival_model_->SetWorldPose(_rival_pos);
}
