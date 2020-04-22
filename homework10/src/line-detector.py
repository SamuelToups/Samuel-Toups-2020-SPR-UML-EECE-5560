#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineDetectorNode:
    def __init__(self):
        rospy.Subscriber("image_cropped", Image, self.cropped_cb)
        rospy.Subscriber("image_white", Image, self.white_cb)
        rospy.Subscriber("image_yellow", Image, self.yellow_cb)
        self.bridge = CvBridge()
