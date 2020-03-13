#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from homework3.msg import numberedstring

class chatterCounter:
    msgs_count = 0
    def __init__(self):
        msgs_count = 0
        rospy.Subscriber("chatter", String, self.callback)
        self.pub = rospy.Publisher('chatterCount', numberedstring, queue_size=10)
    
    def callback(self, data):
        numstr_message = numberedstring()
        numstr_message.seq = self.msgs_count
        self.msgs_count += 1
        numstr_message.text = data.data
        self.pub.publish(numstr_message)

if __name__ == '__main__':
    rospy.init_node('chatterCounter', anonymous=True)
    chatterCounter()
    rospy.spin()
