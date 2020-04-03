#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32

class pidController:
    def __init__(self):
        rospy.Subscriber("error", Float32, self.callback)
        self.pub = rospy.Publisher('control_input', Float32, queue_size=10)
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 1.0 / 1000.0
        self.Kp = 0.3
        self.Ki = 0.0
        self.Kd = 0.6
    def callback(self, data):
        control_message = Float32()
        error = data.data
        self.integral = self.integral + error * self.dt
        derivative = ( error - self.prev_error ) / self.dt
        control_message.data = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        self.pub.publish(control_message)

if __name__ == '__main__':
    rospy.init_node('hw7_pid_controller', anonymous=True)
    pidController()
    rospy.spin()

