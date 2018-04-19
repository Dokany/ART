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
		self.cx_pub = rospy.Publisher("state", Float64, queue_size=10)
		self.cam_pub = rospy.Publisher("setpoint", Float64, queue_size=10)
		self.turn_pub = rospy.Publisher("turn_angle", Float64, queue_size=10)
		self.obj_pub = rospy.Publisher("lidar_stop", Bool, queue_size=100)
		self.bridge = CvBridge()
		self.enable = 1
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)

	def callback(self, data):
		if (self.enable):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			except CvBridgeError as e:
				print(e)

			# Resizing Frame
			small_img = cv2.resize(cv_image, (64, 48), interpolation=cv2.INTER_AREA)

			# Blur Filters
			# small_img = cv2.GaussianBlur(small_img, (5, 5), 0)
			# small_img = cv2.medianBlur(small_img, 5)
			# small_img = cv2.equalizeHist(small_img)

			# Grayscale Image
			gray_img = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)

			# White Daylight
			lower_white = np.array([0, 0, 0])
			upper_white = np.array([0, 0, 255])

			# White Night
			#lower_white = np.array([0, 0, 0])
			#upper_white = np.array([10, 0, 255])

			# Yellow Segmentation
			# lower_yellow = np.array([20, 100, 100])
			# upper_yellow = np.array([30, 255, 255])
			# lower_yellow = np.array([20, 100, 100])
			# upper_yellow = np.array([30, 255, 255])

			#mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

			# Night
			#mask_white = cv2.inRange(gray_img, 60, 255)
			# Daylight
			#mask_white = cv2.inRange(gray_img, 200, 255)
			# Lab - Night
			mask_white = cv2.inRange(gray_img, 198, 255)

			#mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
			output = cv2.bitwise_and(gray_img, mask_white)

			# Declarion of cam center's width
			width = np.size(output, 1) / 2
			height = np.size(output, 0) / 2

			# Thresholding instead of canny
			threshold = 12
			#output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
			_, output = cv2.threshold(output, threshold, 255, cv2.THRESH_BINARY)        # threshold image

			cv2.imshow("Segmentation", output)
			cv2.waitKey(3)

			blind = Bool()
			hough_cx = width
			inter_angle = 0

			lines = cv2.HoughLinesP(output, 1, np.pi / 200, 25, minLineLength=10, maxLineGap=5)
			# 4, 180, 45, 20, 95

			if lines!=None:
				px = 0
				py = 0
				#p3x = sys.maxint
				count = 0
				angle = 0
				for line in lines:
					for x1, y1, x2, y2 in line:
						# if np.abs(x1 - x2) > 5:
						# 	angle += np.degrees(np.arctan(np.abs(x1-x2)/np.abs(y1-y2))) if np.abs(y1-y2) != 0 else 90
						# 	#cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
						# else:
						px += (x1 + x2)
						py += (y1 + y2)
						count += 1
						cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

				if count != 0:
					hough_cx = px / (count * 2)  # state point
					#hough_cx = hough_cx - (hough_cx % 10)
					hough_cy = py / (count * 2)
					print("Hough X: %s" % hough_cx)  # Hough lines state point
					#print(width)  # Setpoint
					#cv2.line(cv_image, (p1, p2), (p3, p4), (255, 0, 0), 2)
					cv2.circle(cv_image, (int(hough_cx), int(hough_cy)), 1, (0, 0, 255), 1)  # draw state center for hough mode
					#cv2.circle(cv_image, (int(width), int(height)), 1, (0, 0, 255), 1)  # draw setpoint center
					blind = False
				else:
					print("No state found")
					blind = True

				if len(lines) != count:
					inter_angle = angle / (len(lines) - count)
				print("Intersection Detected: %f degrees" % inter_angle)

			else:
				print("No road detected!")
				blind = True

			cv2.imshow("Road Seg", small_img)
			cv2.waitKey(3)

			try:
				self.turn_pub.publish(inter_angle)
				self.cx_pub.publish(hough_cx)
				#self.cx_pub.publish(width)     # For pid optimization
				self.cam_pub.publish(width)
				self.obj_pub.publish(blind)
			except CvBridgeError as e:
				print(e)


def main(args):
	ic = image_converter()
	rospy.init_node('Line', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
		print("Shutting down")


if __name__=='__main__':
	main(sys.argv)
