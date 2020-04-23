#!/usr/bin/env python

import rospy
import time
import math
import std_msgs.msg
from duckietown_msgs.msg import LanePose, BoolStamped, Twist2DStamped

class pidData:
    def __init__(self):
        self.prev_error = 0.0
        self.integral = 0.0
        #self.dt = 1.0 / 1000.0
        self.prev_time = rospy.Time.now()
        self.Kp = 3.0
        self.Ki = 0.0
        self.Kd = 0.0

class laneController:
    def __init__(self):
        rospy.logwarn("Custom Lane Controller")
        rospy.Subscriber("~switch", BoolStamped, self.mode_switch_cb)
        rospy.Subscriber("~lane_pose", LanePose, self.lane_pose_cb, "lane_filter", queue_size=1)
        self.pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.phi_pid = pidData()
        #phi_pid tuning data
        self.d_pid = pidData()
        #d_pid tuning data
        self.prev_time = rospy.Time.now()
        self.max_dt = 1.0
        self.mode = False

    def mode_switch_cb(self, data):
        self.mode = data.data

    def lane_pose_cb(self, data, pose_source):
        rospy.logwarn("CUSTOM LANE CONTROLLER: INSIDE LANE POSE CALLBACK")
        if not self.mode:
            rospy.logwarn("CUSTOM LANE CONTROLLER: RETURNING BECAUSE WRONG MODE")
            return

        dt = (data.header.stamp - self.prev_time).to_sec()
        self.prev_time = data.header.stamp
        if dt > self.max_dt:
            rospy.logwarn("CUSTOM LANE CONTROLLER: dt is %s" % dt)
            rospy.logwarn("CUSTOM LANE CONTROLLER: RETURNING BECAUSE TOO MUCH TIME")
            return

        rospy.logwarn("CUSTOM LANE CONTROLLER: PAST RETURN POINTS")
        control_message = Twist2DStamped()

        phi_error = data.phi
        self.phi_pid.integral = self.phi_pid.integral + phi_error * dt
        phi_derivative = ( phi_error - self.phi_pid.prev_error ) / dt
        control_message.omega = self.phi_pid.Kp * phi_error + self.phi_pid.Ki * self.phi_pid.integral + self.phi_pid.Kd * phi_derivative
        self.phi_pid.prev_error = phi_error

        #d_error = data.d
        #self.d_pid.integral = self.d_pid.integral + d_error * dt
        control_message.v = 0.0

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()

        control_message.header = h
        self.pub.publish(control_message)
        rospy.logwarn("CUSTOM LANE CONTROLLER: MESSAGE PUBLISHED")


if __name__ == '__main__':
    rospy.init_node('~mp2_lane_controller_node', anonymous=True)
    laneController()
    rospy.spin()
#!/usr/bin/env python

import rospy
import time
import math
import std_msgs.msg
from duckietown_msgs.msg import LanePose, BoolStamped, Twist2DStamped

class pidData:
    def __init__(self):
        self.prev_error = 0.0
        self.integral = 0.0
        #self.dt = 1.0 / 1000.0
        self.prev_time = rospy.Time.now()

        self.Kp = 3.0
        self.Ki = 0.0
        self.Kd = 0.0

class laneController:
    def __init__(self):
        rospy.logwarn("Custom Lane Controller")
        rospy.Subscriber("~switch", BoolStamped, self.mode_switch_cb)
        rospy.Subscriber("~lane_pose", LanePose, self.lane_pose_cb, "lane_filter", queue_size=1)
        self.pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.phi_pid = pidData()
        #phi_pid tuning data
        self.d_pid = pidData()
        #d_pid tuning data
        self.prev_time = rospy.Time.now()
        self.max_dt = 1.0
        self.mode = False

    def mode_switch_cb(self, data):
        self.mode = data.data

    def lane_pose_cb(self, data, pose_source):
        rospy.logwarn("CUSTOM LANE CONTROLLER: INSIDE LANE POSE CALLBACK")
        if not self.mode:
            rospy.logwarn("CUSTOM LANE CONTROLLER: RETURNING BECAUSE WRONG MODE")
            return

        dt = (data.header.stamp - self.prev_time).to_sec()
        self.prev_time = data.header.stamp
        if dt > self.max_dt:
            rospy.logwarn("CUSTOM LANE CONTROLLER: dt is %s" % dt)
            rospy.logwarn("CUSTOM LANE CONTROLLER: RETURNING BECAUSE TOO MUCH TIME")
            return

        rospy.logwarn("CUSTOM LANE CONTROLLER: PAST RETURN POINTS")
        control_message = Twist2DStamped()

        phi_error = data.phi
        self.phi_pid.integral = self.phi_pid.integral + phi_error * dt
        phi_derivative = ( phi_error - self.phi_pid.prev_error ) / dt
        control_message.omega = self.phi_pid.Kp * phi_error + self.phi_pid.Ki * self.phi_pid.integral + self.phi_pid.Kd * phi_derivative
        self.phi_pid.prev_error = phi_error

        #d_error = data.d
        #self.d_pid.integral = self.d_pid.integral + d_error * dt
        control_message.v = 0.0

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()

        control_message.header = h
        self.pub.publish(control_message)
        rospy.logwarn("CUSTOM LANE CONTROLLER: MESSAGE PUBLISHED")


if __name__ == '__main__':
    rospy.init_node('~mp2_lane_controller_node', anonymous=True)
    laneController()
    rospy.spin()

