#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorFilterNode:
    def __init__(self):
        rospy.Subscriber("image_cropped", Image, self.filter_cb)
        self.bridge = CvBridge()
        self.white_pub = rospy.Publisher("image_white", Image, queue_size=10)
        self.yellow_pub = rospy.Publisher("image_yellow", Image, queue_size=10)
        #self.white_threshold =
        #self.yellow_threshold =
    def filter_cb(self, cropped):
        cv_bgr8_image = self.bridge.imgmsg_to_cv2(cropped, "bgr8")
        cv_hsv_image = cv2.cvtColor(cv_bgr8_image, cv2.COLOR_BGR2HSV)
        
        cv_mono8_white_image = cv2.inRange(cv_hsv_image, (0, 0, 128), (180, 100, 255))
        ros_white_image = self.bridge.cv2_to_imgmsg(cv_mono8_white_image, "mono8")
        
        cv_mono8_yellow_image = cv2.inRange(cv_hsv_image, (15,100,100),(35,255,255))
        ros_yellow_image = self.bridge.cv2_to_imgmsg(cv_mono8_yellow_image, "mono8")
        
        self.white_pub.publish(ros_white_image)
        self.yellow_pub.publish(ros_yellow_image)

if __name__ == "__main__":
    rospy.init_node("color_filter_node", anonymous=True)
    color_filter = ColorFilterNode()
    rospy.spin()


