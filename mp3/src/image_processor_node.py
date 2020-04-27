#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
#from duckietown_msgs.msg import SegmentList

class ImageProcessor:
    def __init__(self):
        rospy.Subscriber("anti_instagram_node/corrected_image/compressed", CompressedImage, self.lanefilter_cb, queue_size=1, buf_size=2**24)
        self.image_pub = rospy.Publisher("image_lines_all", Image, queue_size=1)
        #ideally the line detector node would publish only the data about the lines, and only publish the images when in debug mode
        #self.lines_pub = rospy.Publisher("segment_list", SegmentList, queue_size=1)
        self.bridge = CvBridge()

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

    def lanefilter_cb(self, msg):
        #crop
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        cv_img_dim = cv_img.shape
        cv_img_height = cv_img_dim[0]
        cv_img_width = cv_img_dim[1]
        cv_img_depth = cv_img_dim[2]
        cv_cropped = cv_img[cv_img_height/2:]

        #color filter
        cv_hsv_image = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(cv_hsv_image, (0,0,192), (180, 100, 255))
        yellow_mask = cv2.inRange(cv_hsv_image, (15,100,100), (35,255,255))

        #line detection
        edges_all_image = cv2.Canny(cv_cropped, 0, 255)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

        white_mask_erode = cv2.erode(white_mask, kernel)
        yellow_mask_erode = cv2.erode(yellow_mask, kernel)\

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))

        white_mask_erode_dilate = cv2.dilate(white_mask_erode, kernel)
        yellow_mask_erode_dilate = cv2.dilate(yellow_mask_erode, kernel)

        edges_white_image = cv2.bitwise_and(edges_all_image, edges_all_image, mask=white_mask_erode_dilate)
        edges_yellow_image = cv2.bitwise_and(edges_all_image, edges_all_image, mask=yellow_mask_erode_dilate)

        lines_white = cv2.HoughLinesP(edges_white_image, 1, np.pi/180, 2, 2, 1)
        lines_yellow = cv2.HoughLinesP(edges_yellow_image, 1, np.pi/180, 2, 2, 1)

        lines_white_image = self.output_lines(cv_cropped, lines_white)
        lines_yellow_image = self.output_lines(cv_cropped, lines_yellow)
        lines_all_image = self.output_lines(lines_white_image, lines_yellow)

        ros_lines_all_image = self.bridge.cv2_to_imgmsg(lines_all_image, "bgr8")

        #if debug mode:
        self.image_pub.publish(ros_lines_all_image)

        #append the line segments white and for yellow
        #self.lines_pub.publish(line_segments_all)

if __name__ == "__main__":
    rospy.init_node("line_detector_node", anonymous=True)
    image_processor_node = ImageProcessor()
    rospy.spin()




