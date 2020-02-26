#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped

class openLoopController:
    
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/ademonicduckofsomesort/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.trim = rospy.get_param('/ademonicduckofsomesort/kinematics_node/trim')


if __name__ == "__main__":
    try:
        n = 0
        rospy.init_node('open_loop_control', anonymous=True)
        opc = openLoopController()
        rate = rospy.Rate(0.5)
        cmd1 = Twist2DStamped()
        cmd1.header = Header()
        cmd1.header.stamp = rospy.get_rostime()
        cmd1.header.seq = 1
        cmd1.v = 0.5
        cmd1.omega = 0.0
        
        cmd2 = Twist2DStamped()
        cmd2.header = Header()
        cmd2.header.stamp = rospy.get_rostime()
        cmd2.header.seq = 2
        cmd2.v = 0.0
        cmd2.omega = 8.0
        
        cmd3 = Twist2DStamped()
        cmd3.header = Header()
        cmd3.header.stamp = rospy.get_rostime()
        cmd3.header.seq = 3
        cmd3.v = 0.5
        cmd3.omega = 0.0
        
        cmd4 = Twist2DStamped()
        cmd4.header = Header()
        cmd4.header.stamp = rospy.get_rostime()
        cmd4.header.seq = 4
        cmd4.v = 0.0
        cmd4.omega = 0.0
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            opc.cmd_pub.publish(cmd1)
            rospy.sleep(0.2)
            opc.cmd_pub.publish(cmd2)
            rospy.sleep(0.202)
            opc.cmd_pub.publish(cmd3)
            rospy.sleep(0.2)
            opc.cmd_pub.publish(cmd4)
            rospy.sleep(1.)
    except rospy.ROSInterruptException:
        pass
