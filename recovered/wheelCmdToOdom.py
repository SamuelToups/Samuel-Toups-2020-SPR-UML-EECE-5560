#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from odometry_hw.msg import DistWheel, Pose2D

class wheelCmdToOdom:
    def __init__(self):
        rospy.Subscriber("/ademonicduckofsomesort/wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.cmd_to_odom_callback)
        self.pub = rospy.Publisher('/ademonicduckofsomesort/wheel_cmd_to_odom_node/odom', DistWheel, queue_size=10)
        self.last_cmd_time = rospy.get_rostime()
        self.last_cmd_vel_left = 0.0
        self.last_cmd_vel_right = 0.0
        self.left_correction = 0.5892
        self.right_correction = 0.6468
        self.max_time_step = 0.1
    def cmd_to_odom_callback(self, cmd):
        dist_wheel_msg = DistWheel()
        time_step = (cmd.header.stamp - self.last_cmd_time).to_sec()
        if time_step <= self.max_time_step:
            dist_wheel_msg.dist_wheel_left = self.last_cmd_vel_left * self.left_correction * time_step
            dist_wheel_msg.dist_wheel_right = self.last_cmd_vel_right * self.right_correction * time_step
            self.pub.publish(dist_wheel_msg)
        self.last_cmd_time = cmd.header.stamp
        self.last_cmd_vel_left = cmd.vel_left
        self.last_cmd_vel_right = cmd.vel_right
        self.last_cmd_time = cmd.header.stamp

if __name__ == '__main__':
    rospy.init_node('wheel_cmd_to_odom', anonymous=True)
    wheelCmdToOdom()
    rospy.spin()
