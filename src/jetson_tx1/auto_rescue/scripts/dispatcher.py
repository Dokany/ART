#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import serial
import string
import time
import math
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class dispatcher:

    def __init__(self):
        rospy.init_node('FF_DISPATCHER', anonymous=True)
        self.ff_publish = rospy.Publisher("firefighter_dispatch",Bool, queue_size=10)
        self.frame_id = "angles"
        self.flag = 0
        self.imu_sub = rospy.Subscriber("barcode",String, self.callback)

    def callback(self,data):
        if(self.flag == 0):
            b = Bool()
            b= True
            try:
                print ("dispatching")
                self.ff_publish.publish(b)
            except Exception as e:
                print(e)
            #self.flag=1

def main(args):
    dis = dispatcher()
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
