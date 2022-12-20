#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/ModelStates.h>

#include "MotorSim.hpp"

namespace gazebo {

  class DiffInterfacePlugin : public ModelPlugin
  {
    public:
      DiffInterfacePlugin();
      virtual ~DiffInterfacePlugin();

    protected:
      virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
      virtual void Reset();

    private:
      void timerCallback(const ros::TimerEvent& event);
      void imuTimerCallback(const ros::TimerEvent& event);
      void OnUpdate(const common::UpdateInfo& info);
      void recvLeftCmd(const std_msgs::Float64ConstPtr& msg);
      void recvRightCmd(const std_msgs::Float64ConstPtr& msg);
      void recvModelStates(const gazebo_msgs::ModelStatesConstPtr& msg);

      std::shared_ptr<ros::NodeHandle> n_;
      ros::Publisher pub_twist_;
      ros::Publisher pub_imu_;
      std::shared_ptr<ros::Publisher> pub_odom_;
      ros::Subscriber sub_left_cmd_;
      ros::Subscriber sub_right_cmd_;
      ros::Subscriber sub_model_states_;
      ros::Timer timer_;
      std::shared_ptr<tf::TransformBroadcaster> broadcaster_;

      physics::ModelPtr model_;
      geometry_msgs::Twist twist_;
      sensor_msgs::Imu imu_;
      ros::Time last_model_update_stamp_;
      ros::Time left_cmd_stamp_;
      ros::Time right_cmd_stamp_;
      double left_cmd_speed_;
      double right_cmd_speed_;
      double last_gazebo_update_time_;
      event::ConnectionPtr update_connection_;
      physics::JointPtr left_wheel_joint_;
      physics::JointPtr right_wheel_joint_;
      physics::LinkPtr footprint_link_;
      diff_robot_gazebo::MotorSim::SharedPtr left_motor_;
      diff_robot_gazebo::MotorSim::SharedPtr right_motor_;
      bool ground_truth_mode_;

      static constexpr double MAX_MOTOR_SPEED = 17.0;
  };

  GZ_REGISTER_MODEL_PLUGIN(DiffInterfacePlugin)

}
