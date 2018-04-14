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
		#self.image_pub = rospy.Publisher("seg_output", Image, queue_size=10)
		self.obj_pub = rospy.Publisher("lidar_stop", Bool, queue_size=100)
		#self.sign_pub = rospy.Publisher("sign_start", Bool, queue_size=10)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)
		self.enable = 1
		self.pid_sub = rospy.Subscriber("pid_start", Bool, self.callback0)

	def callback0(self, data):
		self.enable = 1

	def callback(self, data):
		if (self.enable):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				#cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
				#cv_image = cv2.medianBlur(cv_image, 5)
				# equ = cv2.equalizeHist(cv_image)
				# cv_image = np.hstack((cv_image, equ))  # stacking images side-by-side
				# cv_image = cv2.bilateralFilter(cv_image,9,50,50)
			except CvBridgeError as e:
				print(e)

			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

			# lower_yellow = np.array([25, 50, 50])
			# upper_yellow = np.array([32, 255, 255])
			# Night-time
			# lower_green = np.array([60, 50, 50])
			# upper_green = np.array([95, 255, 255])

			# Day-time
			# lower_green = np.array([45, 50, 50])
			# upper_green = np.array([85, 255, 255])

			# White Daylight
			lower_white = np.array([0, 0, 0])
			upper_white = np.array([0, 0, 255])

			# White Night
			#lower_white = np.array([0, 0, 0])
			#upper_white = np.array([10, 0, 255])

			# original values
			# lower_yellow = np.array([20, 100, 100])
			# upper_yellow = np.array([30, 255, 255])
			lower_yellow = np.array([20, 100, 100])
			upper_yellow = np.array([30, 255, 255])

			mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
			# Night
			#mask_white = cv2.inRange(gray, 65, 255)
			# Daylight
			mask_white = cv2.inRange(gray, 200, 255)
			# Lab - Night
			#mask_white = cv2.inRange(gray, 200, 255)

			mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
			output = cv2.bitwise_and(gray, mask_white)

			# mask = cv2.inRange(hsv, lower_white, upper_white)
			# output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

			# Region of Interest
			# defining a blank mask to start with
			roi = np.zeros_like(output)

			# defining a 3 channel or 1 channel color to fill the mask with depending on the input image
			if len(output.shape) > 2:
				channel_count = output.shape[2]  # i.e. 3 or 4 depending on your image
				ignore_mask_color = (255,) * channel_count
			else:
				ignore_mask_color = 255

			# # Verticies of region of interest
			# imshape = output.shape
			# lower_left = [imshape[1] / 9, imshape[0]]
			# lower_right = [imshape[1] - imshape[1] / 9, imshape[0]]
			# top_left = [imshape[1] / 2 - imshape[1] / 8, imshape[0] / 2 + imshape[0] / 10]
			# top_right = [imshape[1] / 2 + imshape[1] / 8, imshape[0] / 2 + imshape[0] / 10]
			# vertices = [np.array([lower_left, top_left, top_right, lower_right], dtype=np.int32)]

			# lower_left = [imshape[1] / 3, imshape[0]]
			# lower_right = [imshape[1] - imshape[1] / 3, imshape[0]]
			# top_left = [imshape[1] / 2 - imshape[1] / 8, imshape[0] / 2 + imshape[0] / 3]
			# top_right = [imshape[1] / 2 + imshape[1] / 8, imshape[0] / 2 + imshape[0] / 3]
			# vertices = [np.array([lower_left, top_left, top_right, lower_right], dtype=np.int32)]

			# Declarion of cam center's width
			width = np.size(output, 1) / 2
			height = np.size(output, 0) / 2

			#output = output[0:height, 0:np.size(output, 1)]
			#cv_image = cv_image[0:height, 0:np.size(cv_image, 1)]

			output = output[height:np.size(output, 0), 0:np.size(output, 1)]
			cv_image = cv_image[height:np.size(cv_image, 0), 0:np.size(cv_image, 1)]

			# filling pixels inside the polygon defined by "vertices" with the fill color
			#cv2.fillPoly(roi, vertices, ignore_mask_color)

			# Hough Lines Edge Detection
			#output_canny = cv2.Canny(output, 50, 150)
			output_canny = cv2.Canny(output, 40, 80)

			# returning the image only where mask pixels are nonzero
			output_canny = cv2.bitwise_and(output_canny, output)

			#cv2.imshow("Segmentation", output)
			#cv2.waitKey(3)
			#
			hough_cx = width + 10
			inter_angle = 0
			#
			b = Bool()
			# s = Bool()
			#
			lines = cv2.HoughLinesP(output_canny, 10, np.pi / 180, 15, minLineLength=10, maxLineGap=1)
			# 4, 45, 20, 95

			if lines!=None:
				#print(lines)
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
					cv2.circle(cv_image, (int(width), int(height)), 1, (0, 0, 255), 1)  # draw setpoint center
					b = False
				else:
					print("No state found")
					b = True

				if len(lines) != count:
					inter_angle = angle / (len(lines) - count)
				print("Intersection Detected: %f degrees" % inter_angle)

			else:
				print("No road detected!")
				b = True

			# #Contouring
			# _, contours, hierarchy = cv2.findContours(output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			#
			# cx = width
			# cy = 0
			#
			#
			# if len(contours)!=0:
			# 	b = False
			# 	lcnt = contours[0]
			# 	larea = cv2.contourArea(contours[0])
			# 	for cnt in contours:  # sort!!
			# 		area = cv2.contourArea(cnt)
			# 		if area > larea:
			# 			lcnt = cnt
			# 			larea = area
			# 	if larea > 600:
			# 		cv2.drawContours(output, lcnt, -1, (0, 255, 0), 3)
			# 		M = cv2.moments(lcnt)
			# 		cx = int(M['m10'] / M['m00'])
			# 		cy = int(M['m01'] / M['m00'])
			# 		#print(width)
			# 		print("Contour X: %s" % cx)
			#  		#print(width)  # Setpoint
			# 		#print("Contour Y: %s" % cy)
			# 	else:
			# 		b = True
			# 		print("No road found!")
			# else:
			# 	b = True
			# 	print("No road found!")

				# if larea > 50000:
				# 	obj_detected = True
				# else:
				# 	obj_detected = False
				#
				# if larea > 30000:
				# 	sign_detected = False
				# else:
				# 	sign_detected = True

				# print(larea, obj_detected)
				#x, y, w, h = cv2.boundingRect(lcnt)
			# Daylight
			# output = output[y + 25:y + h - 30, x + 20:x + w - 20]

			# Night-time
			# output = output[y + 25:y + h, x + 20:x + w - 20]
			# top, bottom, left, right

			#cv2.circle(cv_image, (int(cx), int(cy)), 1, (255, 0, 0), 1)  # draw state center for contours mode

			cv2.imshow("Road Seg", cv_image)
			cv2.waitKey(3)
			#s = sign_detected

			try:
				self.turn_pub.publish(inter_angle)
				#self.cx_pub.publish(cx)
				self.cx_pub.publish(hough_cx)
				#self.cx_pub.publish(width)     # For pid optimization
				self.cam_pub.publish(width)
				self.obj_pub.publish(b)
				#self.sign_pub.publish(s)
				# self.cx_pub.publish(0)
				# self.cam_pub.publish(0)
				#self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "8UC1"))
			except CvBridgeError as e:
				print(e)


def main(args):
	ic = image_converter()
	rospy.init_node('VISION', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
		print("Shutting down")


if __name__=='__main__':
	main(sys.argv)
