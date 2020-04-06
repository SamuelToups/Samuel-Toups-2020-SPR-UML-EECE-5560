#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCropper:
    def __init__(self):
        rospy.Subscriber("image", Image, self.crop_cb)
        self.pub = rospy.Publisher("image_cropped", Image, queue_size=10)
        self.bridge = CvBridge()
    def crop_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img_dim = cv_img.shape
        cv_img_height = cv_img_dim[0]
        cv_img_width = cv_img_dim[1]
        cv_img_depth = cv_img_dim[2]
        print("image dimensions: ", cv_img_dim)
        cv_cropped = cv_img[cv_img_height/2:]
        ros_cropped = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")
        self.pub.publish(ros_cropped)

if __name__ == "__main__":
    rospy.init_node("image_cropper", anonymous=True)
    img_crop = ImageCropper()
    rospy.spin()
