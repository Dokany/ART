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
		rospy.init_node('JunctionDetector', anonymous=True)
		# Published Topics
		self.junction = rospy.Publisher("junction", Bool, queue_size=10)
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

			# Show Camera Feed
			#cv2.imshow("Junction Cam", cv_image)
			#cv2.waitKey(3)

			# Resizing Frame  For camera frame 640 X 480, image will be 64 x 48
			cam_width = np.size(cv_image, 1) / 10
			cam_height = np.size(cv_image, 0) / 10
			img = cv2.resize(cv_image, (cam_width, cam_height), interpolation=cv2.INTER_AREA)
			img = img[(np.size(img, 0)) / 2:, :np.size(img, 1)]

			# Blur Filters
			img = cv2.GaussianBlur(img, (5, 5), 0)
			#img = cv2.medianBlur(img, 5)
			# img = cv2.equalizeHist(img)

			# Converting Image to HSV
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

			# Daylight - Shade Yellow Masks
			lower_yellow = np.array([15, 40, 80])
			upper_yellow = np.array([45, 255, 255])
			# Night Yellow Masks
			#lower_yellow = np.array([18, 100, 70])
			#upper_yellow = np.array([112, 255, 255])
            #Night with lights
			#lower_yellow = np.array([19, 100, 70])
			#upper_yellow = np.array([112, 255, 255])

			mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
			seg_img = cv2.bitwise_and(hsv, hsv, mask=mask_yellow)

			# Thresholding image post segmentation into binary values
			threshold = 20
			gray = cv2.cvtColor(seg_img, cv2.COLOR_BGR2GRAY)
			_, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

			# To view segmentation seg_img
			#cv2.imshow("Segmentation", seg_img)
			#cv2.waitKey(3)

			# To view thresholding seg_img
			#cv2.imshow("Thresholding", thresh)
			#cv2.waitKey(3)

			junction = Bool()		# True when no line has been detected

			# Contouring Image
			_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			if len(contours)!=0:
				lcnt = contours[0]
				larea = cv2.contourArea(contours[0])
				for cnt in contours:	# Sorting contour areas found
					area = cv2.contourArea(cnt)
					#print(area)
					if area > larea:
						lcnt = cnt
						larea = area

				if larea > 100:
					junction = True
					print("Junction Detected!")
					#cv2.drawContours(cv_image, lcnt, -1, (0, 255, 0), 3)
					M = cv2.moments(lcnt)
					cx = int(M['m10'] / M['m00'])
					cy = int(M['m01'] / M['m00'])
				else:
					junction = False
					print("No Junctions!")
			else:
				junction = False
				print("No Junction Contours!")

			#  To draw state point for contours mode
			#cv2.circle(cv_image, (int(cx*10), int(cy*10)), 1, (255, 30, 0), 1)

			# To view state point on image
			#cv2.imshow("Junction Center", cv_image)
			#cv2.waitKey(3)

			try:
				self.junction.publish(junction)
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
