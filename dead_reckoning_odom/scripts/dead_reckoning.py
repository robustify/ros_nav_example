#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import TwistStamped, TransformStamped
import tf
from tf2_ros import TransformBroadcaster


class DeadReckoning:
    def __init__(self):
        # Initialize variables
        self.twist_data = TwistStamped()
        self.x = 0
        self.y = 0
        self.psi = 0
        self.sample_time = 0.02

        # Load frame IDs from parameters
        self.parent_frame = 'odom'
        self.child_frame = 'base_footprint'

        self.sub_twist = rospy.Subscriber('diff_robot/twist', TwistStamped, self.recv_twist)
        self.timer = rospy.Timer(rospy.Duration(self.sample_time), self.timer_cb)
        self.broadcaster = TransformBroadcaster()

    def timer_cb(self, event):
        # TODO: Integrate vehicle navigation model one step with the latest speed and yaw rate data


        # TODO: Populate and broadcast a TF frame with the position and orientation from the model
        transform_msg = TransformStamped()

    def recv_twist(self, msg):
        self.twist_data = msg


if __name__ == '__main__':
    rospy.init_node('dead_reckoning')
    node_instance = DeadReckoning()
    rospy.spin()
