#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class image_converter:

    def __init__(self):
        rospy.init_node('ContourVision', anonymous=True)
        # Published Topics
        self.cx_pub = rospy.Publisher("state", Float64, queue_size=10)
        self.cam_pub = rospy.Publisher("setpoint", Float64, queue_size=10)
        self.turn_pub = rospy.Publisher("turn_angle", Float64, queue_size=10)
        self.obj_pub = rospy.Publisher("blind", Bool, queue_size=10)
        # Subscribed Topics
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)
        self.enable = 1 # 1 for testing, 0 for running
        self.pid_sub = rospy.Subscriber("pid_start", Bool, self.callback0)

    def callback0(self, data): #pid_start
        self.enable = data.data

    def callback(self, data):
        if (self.enable):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # Resizing Frame  For camera frame 640 X 480, image will be 64 x 48
            cam_width = np.size(cv_image, 1) / 10
            cam_height = np.size(cv_image, 0) / 10
            hough_img = cv2.resize(cv_image, (cam_width, cam_height), interpolation=cv2.INTER_AREA)

            # Blur Filters
            hough_img = cv2.GaussianBlur(hough_img, (5, 5), 0)
            #hough_img = cv2.medianBlur(hough_img, 5)
            #hough_img = cv2.equalizeHist(hough_img)

            # Converting Image to HSV and Grayscale Images
            kernel = np.ones((5, 5), np.uint8)
            gray = cv2.cvtColor(hough_img, cv2.COLOR_BGR2GRAY)
            #img_dilation = cv2.dilate(gray, kernel, iterations= 1)
            gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)

            # Daylight
            mask_white = cv2.inRange(gray, 180, 255)
            # Night
            #mask_white = cv2.inRange(gray, 220, 255)
            # Lab - Night
            #mask_white = cv2.inRange(gray, 200, 255)

            output = cv2.bitwise_and(gray, mask_white)

            # Declarion of cam center's width
            width = np.size(output, 1) / 2
            height = np.size(output, 0) / 2

            #output = output[height:np.size(output, 0), 0:np.size(output, 1)]
            #cv_image = cv_image[(np.size(cv_image, 0)) / 2:np.size(cv_image, 0), 0:np.size(cv_image, 1)]

            # Image Cropping - not needed anymore
            # Upper Half

            # Thresholding image post segmentation into binary values
            threshold = 12
            _, thresh = cv2.threshold(output, threshold, 255, cv2.THRESH_BINARY)

            # To view segmentation output
            #cv2.imshow("Segmentation", output)
            #cv2.waitKey(3)

            # To view thresholding output
            #cv2.imshow("Thresholding", thresh)
            #cv2.waitKey(3)

            inter_angle = 0		# Intersection Angle

            blind = Bool()		# True when no line has been detected

            # Contouring Image
            _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            cx = width
            cy = 0

            if len(contours)!=0:
                b = False
                lcnt = contours[0]
                larea = cv2.contourArea(contours[0])
                for cnt in contours:	# Sorting contour areas found
                    area = cv2.contourArea(cnt)
                    #print(area)
                    if area > larea:
                        lcnt = cnt
                        larea = area
                if larea > 25:
                    cv2.drawContours(output, lcnt, -1, (0, 255, 0), 3)
                    M = cv2.moments(lcnt)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    print("Contour X: %s" % cx)		# State Value
                    #print(width)  # Setpoint Value
                    #print("Contour Y: %s" % cy)
                else:
                    blind = True
                    print("No road found!")
            else:
                blind = True
                print("No contours!")

            #  To draw state point for contours mode
            #cv2.circle(cv_image, (int(cx*10), int(cy*10)), 1, (255, 0, 0), 1)

            # To view state point on image
            #cv2.imshow("Road Seg", cv_image)
            #cv2.waitKey(3)

            try:
                self.turn_pub.publish(inter_angle)
                self.cx_pub.publish(cx * 10)		# scaled down value * 10
                self.cam_pub.publish(width * 10)	# scaled down value * 10
                self.obj_pub.publish(blind)
            except CvBridgeError as e:
                print(e)

def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Shutting down")


if __name__=='__main__':
    main(sys.argv)
