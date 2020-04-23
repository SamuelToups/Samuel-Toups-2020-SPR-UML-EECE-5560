#!/usr/bin/env python

import rospy
import math
import std_msgs.msg
from duckietown_msgs.msg import LanePose, BoolStamped, Twist2DStamped

class pidData:
    def __init__(self):
        self.prev_error = 0.0
        self.integral = 0.0
        #self.dt = 1.0 / 1000.0
        self.prev_time = rospy.Time.now()
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0

class laneController:
    def __init__(self):
        rospy.Subscriber("/ademonicduckofsomesort/lane_controller_node/switch", BoolStamped, self.mode_switch_cb)
        rospy.Subscriber("/ademonicduckofsomesort/lane_filter_node/lane_pose", LanePose, self.lane_pose_cb)
        self.pub = rospy.Publisher("/ademonicduckofsomesort/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.phi_pid = pidData()
        #phi_pid tuning data
        self.d_pid = pidData()
        #d_pid tuning data
        self.prev_time = rospy.Time.now()
        self.max_dt = 0.1
        self.mode = True
    
    def mode_switch_cb(self, data):
        self.mode = data.data
    
    def lane_pose_cb(self, data):
        if not self.mode:
            return
        
        dt = (data.header.stamp - self.prev_time).to_sec
        self.prev_time = data.header.stamp
        if dt > self.max_dt:
            return
        
        control_message = Twist2DStamped()
        
        phi_error = data.phi
        self.phi_pid.integral = self.phi_pid.integral + phi_error * self.phi_pid.dt
        phi_derivative = ( phi_error - self.phi_pid.prev_error ) / self.phi_pid.dt
        control_message.omega = self.phi_pid.Kp * phi_error + self.phi_pid.Ki * self.phi_pid.integral + self.phi_pid.Kd * phi_derivative
        self.phi_pid.prev_error = phi_error
        
        #d_error = data.d
        #self.d_pid.integral = self.d_pid.integral + d_error * self.d_pid.dt
        control_message.v = 0
        
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        
        control_message.header = h
        self.pub.Publish(control_message)

if __name__ == '__main__':
    rospy.init_node('mp2_lane_controller_node', anonymous=True)
    laneController()
    rospy.spin()
