#include "DiffInterfacePlugin.hpp"

namespace gazebo {

DiffInterfacePlugin::DiffInterfacePlugin()
{
}

void DiffInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  // Gazebo initialization
  left_wheel_joint_ = model->GetJoint("left_wheel_joint");
  right_wheel_joint_ = model->GetJoint("right_wheel_joint");
  footprint_link_ = model->GetLink("base_footprint");
  model_ = model;
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DiffInterfacePlugin::OnUpdate, this, _1));
  right_wheel_joint_->SetParam("fmax", 0, 99999.0);
  left_wheel_joint_->SetParam("fmax", 0, 99999.0);

  // SDF parameters
  if (sdf->HasElement("ground_truth")) {
    sdf->GetElement("ground_truth")->GetValue()->Get(ground_truth_mode_);
  } else {
    ground_truth_mode_ = false;
  }

  // ROS initialization
  n_ = std::make_shared<ros::NodeHandle>(model->GetName());

  left_motor_ = std::make_shared<diff_robot_gazebo::MotorSim>(MAX_MOTOR_SPEED, 8.0, 8.0);
  right_motor_ = std::make_shared<diff_robot_gazebo::MotorSim>(MAX_MOTOR_SPEED, 8.0, 8.0);

  sub_left_cmd_ = n_->subscribe("left_speed_cmd", 1, &DiffInterfacePlugin::recvLeftCmd, this);
  sub_right_cmd_ = n_->subscribe("right_speed_cmd", 1, &DiffInterfacePlugin::recvRightCmd, this);
  sub_model_states_ = n_->subscribe("/gazebo/model_states", 1, &DiffInterfacePlugin::recvModelStates, this);

  pub_twist_ = n_->advertise<geometry_msgs::TwistStamped>("twist", 1);
  pub_imu_ = n_->advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  if (ground_truth_mode_) {
    pub_odom_ = std::make_shared<ros::Publisher>(n_->advertise<nav_msgs::Odometry>("ground_truth_odom", 1));
    broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  }

  timer_ = n_->createTimer(ros::Duration(0.01), &DiffInterfacePlugin::timerCallback, this);
}

void DiffInterfacePlugin::OnUpdate(const common::UpdateInfo& info) {
  ros::Time current_ros_time = ros::Time::now();
  double current_gazebo_update_time = info.simTime.Double();
  double dt = current_gazebo_update_time - last_gazebo_update_time_;
  last_gazebo_update_time_ = current_gazebo_update_time;

  if (dt > 1.0){
    return;
  }

  double left_actual;
  double right_actual;

  if (((current_ros_time - left_cmd_stamp_).toSec() >= 0.25) || ((current_ros_time - right_cmd_stamp_).toSec() >= 0.25)) {
    left_actual = left_motor_->iterate(0.0, dt);
    right_actual = right_motor_->iterate(0.0, dt);
  }else{
    left_actual = left_motor_->iterate(left_cmd_speed_, dt);
    right_actual = right_motor_->iterate(right_cmd_speed_, dt);
  }

  left_wheel_joint_->SetParam("vel", 0, left_actual);
  right_wheel_joint_->SetParam("vel", 0, right_actual);
}

void DiffInterfacePlugin::recvLeftCmd(const std_msgs::Float64ConstPtr& msg)
{
  left_cmd_speed_ = std::clamp(msg->data, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  left_cmd_stamp_ = ros::Time::now();
}

void DiffInterfacePlugin::recvRightCmd(const std_msgs::Float64ConstPtr& msg)
{
  right_cmd_speed_ = std::clamp(msg->data, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  right_cmd_stamp_ = ros::Time::now();
}

void DiffInterfacePlugin::recvModelStates(const gazebo_msgs::ModelStatesConstPtr& msg)
{
  // Extract pose and twist from model state
  geometry_msgs::Twist global_twist;
  geometry_msgs::Pose global_pose;
  for (size_t i=0; i<msg->name.size(); i++){
    if (!msg->name[i].compare(model_->GetName())){
      global_twist = msg->twist[i];
      global_pose = msg->pose[i];
    }
  }

  // Transform twist into local frame to simulate speed and yaw rate sensors
  double cpsi = global_pose.orientation.w * global_pose.orientation.w - global_pose.orientation.z * global_pose.orientation.z;
  double spsi = 2.0 * global_pose.orientation.w * global_pose.orientation.z;
  geometry_msgs::Twist new_twist;
  new_twist.linear.x = global_twist.linear.x * cpsi + global_twist.linear.y * spsi;
  new_twist.linear.y = -global_twist.linear.x * spsi + global_twist.linear.y * cpsi;
  new_twist.linear.z = global_twist.linear.z;
  new_twist.angular.z = global_twist.angular.z;

  // Compute acceleration and generate IMU message
  ros::Time current_stamp = ros::Time::now();
  double dt = (current_stamp - last_model_update_stamp_).toSec();
  if (dt < 1.0 && dt > 1e-9){
    imu_.linear_acceleration.x = (new_twist.linear.x - twist_.linear.x) / dt;
    imu_.linear_acceleration.y = new_twist.linear.x * new_twist.angular.z;
    imu_.linear_acceleration.z = -9.81;

    imu_.angular_velocity.x = global_twist.angular.x;
    imu_.angular_velocity.y = global_twist.angular.y;
    imu_.angular_velocity.z = global_twist.angular.z;
  }

  twist_ = new_twist;
  last_model_update_stamp_ = current_stamp;

}

void DiffInterfacePlugin::timerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = event.current_real;
  twist_msg.header.frame_id = footprint_link_->GetName();
  twist_msg.twist = twist_;
  pub_twist_.publish(twist_msg);

  imu_.header.stamp = event.current_real;
  imu_.header.frame_id = footprint_link_->GetName();
  pub_imu_.publish(imu_);


  if (ground_truth_mode_) {
    tf::StampedTransform pose;
    pose.frame_id_ = "world";
    pose.child_frame_id_ = footprint_link_->GetName();
    pose.stamp_ = event.current_real;
    pose.setOrigin(tf::Vector3(footprint_link_->WorldPose().Pos().X(), footprint_link_->WorldPose().Pos().Y(), footprint_link_->WorldPose().Pos().Z()));
    pose.setRotation(tf::Quaternion(footprint_link_->WorldPose().Rot().X(), footprint_link_->WorldPose().Rot().Y(), footprint_link_->WorldPose().Rot().Z(), footprint_link_->WorldPose().Rot().W()));
    double vehicle_speed = footprint_link_->RelativeLinearVel().X();
    double yaw_rate = footprint_link_->RelativeAngularVel().Z();

    if (pub_odom_ != nullptr) {
      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "world";
      odom_msg.child_frame_id = footprint_link_->GetName();
      odom_msg.header.stamp = event.current_real;
      odom_msg.pose.pose.position.x = pose.getOrigin().x();
      odom_msg.pose.pose.position.y = pose.getOrigin().y();
      odom_msg.pose.pose.position.z = pose.getOrigin().z();
      odom_msg.pose.pose.orientation.w = pose.getRotation().w();
      odom_msg.pose.pose.orientation.x = pose.getRotation().x();
      odom_msg.pose.pose.orientation.y = pose.getRotation().y();
      odom_msg.pose.pose.orientation.z = pose.getRotation().z();
      odom_msg.twist.twist.linear.x = vehicle_speed;
      odom_msg.twist.twist.angular.z = yaw_rate;
      pub_odom_->publish(odom_msg);
    }

    if (broadcaster_ != nullptr) {
      broadcaster_->sendTransform(pose);
    }
  }
}

void DiffInterfacePlugin::Reset()
{
}

DiffInterfacePlugin::~DiffInterfacePlugin()
{
  n_->shutdown();
}

}