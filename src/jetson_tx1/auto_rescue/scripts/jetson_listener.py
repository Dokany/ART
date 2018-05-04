#!/usr/bin/env python

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import serial
import string
import time
import math

from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class jetson_listener:

    def __init__(self):
        rospy.init_node('Listener', anonymous=True)
        self.wheels_publish = rospy.Publisher("wheels_speed",Vector3Stamped, queue_size=10)
        self.lidar_sub = rospy.Subscriber("firefighter_dispatch", Bool, self.callback0)
        self.angles_sub = rospy.Subscriber("imu_angles",Vector3Stamped, self.callback1)
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.start = 0
        self.angle = 0
        self.rotate = 0

    def callback1(self, data):
        self.roll = data.vector.x
        self.pitch = data.vector.y
        self.yaw = data.vector.z
        if (self.yaw < 0):
            self.yaw = 360 + self.yaw

    def callback0(self, data):
        self.start = data.data

    def work(self):
        rate = rospy.Rate(50)
        self.yaw = 0
        dir = left = right = 0
        while not rospy.is_shutdown():
            if(self.start or self.rotate):
                print("dispatched")
                if (self.rotate == 0):
                    self.angle = self.yaw
                    self.rotate = 1
                else:
                    print('current = %f, yaw = %f' % (self.angle, self.yaw))

                    if (abs(self.angle - self.yaw) <= 90):
                        left=right = 30
                        dir = 1

                    else:
                        left=right=0
                        self.rotate = 0
                        self.start = 0
                v3 = Vector3()
                v = Vector3Stamped()
                head = Header()
                head.stamp = rospy.Time.now()
                v3.x = left
                v3.y = right
                v3.z = dir
                v.header = head
                v.vector = v3
                try:
				    self.wheels_publish.publish(v)

                except Exception as e:
                    print(e)
                rate.sleep()


def main(args):
    jl = jetson_listener()
    jl.work()


if __name__ == '__main__':
    main(sys.argv)