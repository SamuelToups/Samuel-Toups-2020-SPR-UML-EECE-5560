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
        #self.canny_pub = rospy.Publisher("image_canny", Image, queue_size=10)
        #self.edges_white_pub = rospy.Publisher("image_edges_white", Image, queue_size=10)
        #self.edges_yellow_pub = rospy.Publisher("image_edges_yellow", Image, queue_size=10)
        self.white_pub = rospy.Publisher("image_lines_white", Image, queue_size=10)
        self.yellow_pub = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
        self.all_pub = rospy.Publisher("image_lines_all", Image, queue_size=10)
        self.bridge = CvBridge()
        self.cropped_image = None
        self.white_image = None
        self.yellow_image = None
    
    def cropped_cb(self, cropped):
        self.cropped_image = self.bridge.imgmsg_to_cv2(cropped, "bgr8")
        if self.cropped_image is not None and self.white_image is not None and self.yellow_image is not None:
            self.process_images()
    
    def white_cb(self, white):
        self.white_image = self.bridge.imgmsg_to_cv2(white, "mono8")
        if self.cropped_image is not None and self.white_image is not None and self.yellow_image is not None:
            self.process_images()
    
    def yellow_cb(self, yellow):
        self.yellow_image = self.bridge.imgmsg_to_cv2(yellow, "mono8")
        if self.cropped_image is not None and self.white_image is not None and self.yellow_image is not None:
            self.process_images()
    
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
    
    def process_images(self):
        edges_all_image = cv2.Canny(self.cropped_image, 0, 255)
        #ros_canny_debug = self.bridge.cv2_to_imgmsg(edges_all_image, "mono8")
        #self.canny_pub.publish(ros_canny_debug)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        
        white_image_erode = cv2.erode(self.white_image, kernel)
        yellow_image_erode = cv2.erode(self.yellow_image, kernel)
        
        white_image_erode_dilate = cv2.dilate(white_image_erode, kernel)
        yellow_image_erode_dilate = cv2.dilate(yellow_image_erode, kernel)
        
        edges_white_image = cv2.bitwise_and(edges_all_image, edges_all_image, mask = white_image_erode_dilate)
        #ros_edges_white_debug = self.bridge.cv2_to_imgmsg(edges_white_image, "mono8")
        #self.edges_white_pub.publish(ros_edges_white_debug)
        
        edges_yellow_image = cv2.bitwise_and(edges_all_image, edges_all_image, mask = yellow_image_erode_dilate)
        #ros_edges_yellow_debug = self.bridge.cv2_to_imgmsg(edges_yellow_image, "mono8")
        #self.edges_yellow_pub.publish(ros_edges_yellow_debug)
        
        lines_white = cv2.HoughLinesP(edges_white_image, 1, np.pi/180, 2, 2, 1)
        lines_yellow = cv2.HoughLinesP(edges_yellow_image, 1, np.pi/180, 5, 5, 2)
        
        lines_white_image = self.output_lines(self.cropped_image, lines_white)
        lines_yellow_image = self.output_lines(self.cropped_image, lines_yellow)
        lines_all_image = self.output_lines(lines_white_image, lines_yellow)
        
        ros_lines_white_image = self.bridge.cv2_to_imgmsg(lines_white_image, "bgr8")
        ros_lines_yellow_image = self.bridge.cv2_to_imgmsg(lines_yellow_image, "bgr8")
        ros_lines_all_image = self.bridge.cv2_to_imgmsg(lines_all_image, "bgr8")
        
        self.white_pub.publish(ros_lines_white_image)
        self.yellow_pub.publish(ros_lines_yellow_image)
        self.all_pub.publish(ros_lines_all_image)
        self.cropped_image = None
        self.white_image = None
        self.yellow_image = None


if __name__ == "__main__":
    rospy.init_node("line_detector_node", anonymous=True)
    line_detector = LineDetectorNode()
    rospy.spin()
