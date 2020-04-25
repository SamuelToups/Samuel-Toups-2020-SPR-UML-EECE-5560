#!/usr/bin/env python

import rospy
import math
from odometry_hw.msg import DistWheel, Pose2D

class odomToPose:
    def __init__(self):
        rospy.Subscriber("dist_wheel", DistWheel, self.callback)
        self.pub = rospy.Publisher('pose', Pose2D, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.L = 0.00625
        self.delta_theta_correction = 90.0 / 20.0
    
    def callback(self, data):
        pose_message = Pose2D()
	delta_s = ( data.dist_wheel_left + data.dist_wheel_right ) / 2.0
        delta_theta = ( data.dist_wheel_right - data.dist_wheel_left ) / ( 2.0 * self.L ) # * self.delta_theta_correction
        delta_x = delta_s * math.cos( self.theta + ( delta_theta / 2.0 ) )
        delta_y = delta_s * math.sin( self.theta + ( delta_theta / 2.0 ) )
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        if self.theta > 360.0:
            self.theta = self.theta - 360.0
        if self.theta < 0.0:
            self.theta = self.theta + 360.0
        pose_message.x = self.x
        pose_message.y = self.y
        pose_message.theta = self.theta
        self.pub.publish(pose_message)

if __name__ == '__main__':
    rospy.init_node('odom_to_pose', anonymous=True)
    odomToPose()
    rospy.spin()
