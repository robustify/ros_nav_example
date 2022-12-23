// Header file for the class
#include "DeadReckoning.hpp"

// Namespace matches ROS package name
namespace dead_reckoning {

  // Constructor with global and private node handle arguments
  DeadReckoning::DeadReckoning(ros::NodeHandle n, ros::NodeHandle pn) {
    parent_frame = "odom";
    child_frame = "base_footprint";
    x = 0;
    y = 0;
    psi = 0;
    sample_time = 0.02;

    sub_twist = n.subscribe("diff_robot/twist", 1, &DeadReckoning::recvTwist, this);
    timer = n.createTimer(ros::Duration(sample_time), &DeadReckoning::timerCallback, this);
  }

  void DeadReckoning::timerCallback(const ros::TimerEvent& event) {
    // Integrate vehicle kinematics state space model one step with the latest speed and yaw rate data
    double v = twist_data.twist.linear.x;
    double pdot = twist_data.twist.angular.z;
    x += sample_time * v * cos(psi);
    y += sample_time * v * sin(psi);
    psi += sample_time * pdot;

    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.frame_id = parent_frame;
    transform_msg.header.stamp = event.current_real;
    transform_msg.child_frame_id = child_frame;

    // Copy dead reckoning estimate into transform message
    transform_msg.transform.translation.x = x;
    transform_msg.transform.translation.y = y;
    transform_msg.transform.translation.z = 0;

    // Populate quaternion corresponding to current yaw angle
    tf2::Quaternion q;
    q.setRPY(0, 0, psi);
    tf2::convert(q, transform_msg.transform.rotation);

    // Update TF transform with the latest dead reckoning iteration
    broadcaster.sendTransform(transform_msg);
  }

  void DeadReckoning::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg) {
    twist_data = *msg;
  }
}
