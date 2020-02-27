#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped

class openLoopController:
    
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/ademonicduckofsomesort/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)


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
        cmd4.omega = -8.0
        
        cmd5 = Twist2DStamped()
        cmd5.header = Header()
        cmd5.header.stamp = rospy.get_rostime()
        cmd5.header.seq = 1
        cmd5.v = 0.5
        cmd5.omega = 0.0
        
        cmd6 = Twist2DStamped()
        cmd6.header = Header()
        cmd6.header.stamp = rospy.get_rostime()
        cmd6.header.seq = 1
        cmd6.v = -0.5
        cmd6.omega = 0.0
        
        cmd7 = Twist2DStamped()
        cmd7.header = Header()
        cmd7.header.stamp = rospy.get_rostime()
        cmd7.header.seq = 1
        cmd7.v = 0.0
        cmd7.omega = 8.0
        
        cmd8 = Twist2DStamped()
        cmd8.header = Header()
        cmd8.header.stamp = rospy.get_rostime()
        cmd8.header.seq = 1
        cmd8.v = 0.5
        cmd8.omega = 0.0
        
        cmd9 = Twist2DStamped()
        cmd9.header = Header()
        cmd9.header.stamp = rospy.get_rostime()
        cmd9.header.seq = 1
        cmd9.v = 0.0
        cmd9.omega = 0.0
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            opc.cmd_pub.publish(cmd1)
            rospy.sleep(0.5)
            opc.cmd_pub.publish(cmd2)
            rospy.sleep(0.2)
            opc.cmd_pub.publish(cmd3)
            rospy.sleep(0.5)
            opc.cmd_pub.publish(cmd4)
            rospy.sleep(0.2)
            opc.cmd_pub.publish(cmd5)
            rospy.sleep(0.5)
            opc.cmd_pub.publish(cmd9)
            rospy.sleep(0.1)
            opc.cmd_pub.publish(cmd6)
            rospy.sleep(0.5)
            opc.cmd_pub.publish(cmd7)
            rospy.sleep(0.2)
            opc.cmd_pub.publish(cmd8)
            rospy.sleep(0.5)
            opc.cmd_pub.publish(cmd9)
            rospy.sleep(5.0)
    except rospy.ROSInterruptException:
        pass
