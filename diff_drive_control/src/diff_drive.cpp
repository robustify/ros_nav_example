#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_left;
ros::Publisher pub_right;

void recvTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  std_msgs::Float64 left_wheel_command;
  std_msgs::Float64 right_wheel_command;

  double track_width = 1.0;
  double wheel_radius = 0.2;
  left_wheel_command.data = (msg->linear.x - track_width * msg->angular.z / 2) / wheel_radius;
  right_wheel_command.data = (msg->linear.x + track_width * msg->angular.z / 2) / wheel_radius;

  pub_left.publish(left_wheel_command);
  pub_right.publish(right_wheel_command);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_drive_control");
  ros::NodeHandle n;

  pub_left = n.advertise<std_msgs::Float64>("diff_robot/left_speed_cmd", 1);
  pub_right = n.advertise<std_msgs::Float64>("diff_robot/right_speed_cmd", 1);
  ros::Subscriber sub_twist = n.subscribe("twist_cmd", 1, recvTwist);

  ros::spin();
}
