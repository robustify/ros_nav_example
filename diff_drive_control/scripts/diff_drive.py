#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class DiffDrive:
    def __init__(self):
        self.sub_twist = rospy.Subscriber('twist_cmd', Twist, callback=self.recv_twist)
        self.pub_left = rospy.Publisher('diff_robot/left_speed_cmd', Float64, queue_size=1)
        self.pub_right = rospy.Publisher('diff_robot/right_speed_cmd', Float64, queue_size=1)

    def recv_twist(self, msg):
        left_speed_msg = Float64()
        right_speed_msg = Float64()

        track_width = 1.0
        wheel_radius = 0.2
        left_speed_msg.data = (msg.linear.x - track_width * msg.angular.z / 2) / wheel_radius
        right_speed_msg.data = (msg.linear.x + track_width * msg.angular.z / 2) / wheel_radius

        self.pub_left.publish(left_speed_msg)
        self.pub_right.publish(right_speed_msg)


if __name__ == '__main__':
    rospy.init_node('diff_drive_control')
    node_instance = DiffDrive()
    rospy.spin()