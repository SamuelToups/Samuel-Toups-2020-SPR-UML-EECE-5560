#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class chatterLogger:
    def __init__(self):
        rospy.Subscriber("chatter", String, self.callback)
        self.pub = rospy.Publisher('changedChatter', String, queue_size=10)
    
    def callback(self, data):
        new_str = "MESSAGE:" + data.data
        rospy.loginfo(new_str)
        self.pub.publish(new_str)

if __name__ == '__main__':
    rospy.init_node('chatterLogger', anonymous=True)
    chatterLogger()
    
    rospy.spin()
