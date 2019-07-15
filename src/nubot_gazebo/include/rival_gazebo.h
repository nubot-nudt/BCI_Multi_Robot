#ifndef RIVAL_GAZEBO_HH
#define RIVAL_GAZEBO_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nubot_common/ResetInfo.h>

#include "core.hpp"


namespace gazebo{

  class RivalGazebo : public ModelPlugin
  {
    private:

        physics::WorldPtr           world_;                // A pointer to the gazebo world.
        physics::ModelPtr           rival_model_;          // Pointer to the model

        ros::NodeHandle*            rosnode_;              // A pointer to the ROS node.
        ros::Subscriber             setinfo_sub_;
        event::ConnectionPtr        update_connection_;    // Pointer to the update event connection
        std::string                 rival_chassis_;
        std::string                 model_name_;
        std::string                 robot_namespace_;
        std::string                 mag_pre_;
        physics::LinkPtr            rival_link_;           //Pointer to the football link

        double                      vel_x_;
        double                      vel_y_;
        double                      mu_;                   // frictional coefficient
        double                      field_length_;
        double                      field_width_;
        int                         AgentID_;

        /// \brief callback function for reset ball position
        void set_robot_pos(const nubot_common::ResetInfo & _msg);

    public:
        /// \brief Constructor. Will be called firstly
        RivalGazebo();

        /// \brief Destructor
        virtual ~RivalGazebo();

    protected:
        /// \brief Load the controller.
        /// Required by model plugin. Will be called secondly
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
  };
}

#endif //! RIVAL_GAZEBO_HH
