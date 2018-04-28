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
import message_filters
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

IDLE_STATE = 0
DRIVE_STATE = 1
BUILDING_STATE = 2
STRAIGHT = 8
RIGHT = 9
LEFT = 10
PID_RATIO = 80.0/1000.0
class master_control:

    def __init__(self):
        rospy.init_node('MASTER_CONTROL_jet', anonymous=True)
        self.wheels_publish = rospy.Publisher("jetson/wheels_speed",Vector3Stamped, queue_size=10)
        self.pid_publish = rospy.Publisher("jetson/pid_start", Bool, queue_size=10)
        self.angles_sub = rospy.Subscriber("jetson/imu_angles",Vector3Stamped, self.callback1)
        self.lw_sub = rospy.Subscriber("jetson/left_wheel/control_effort", Float64, self.callback2)
        self.rw_sub = rospy.Subscriber("jetson/right_wheel/control_effort", Float64, self.callback3)
        self.sd_pub = rospy.Publisher("sign_direction_pi", Float32, self.callback4)
        self.disp_pub = rospy.Publisher("dispatch_pi", Bool,  queue_size=10)
        self.bar_sub = rospy.Subscriber("barcode", String, self.callback0)
        self.cam_sub = rospy.Subscriber("jetson/blind", Bool, self.callback5)
        self.req_sub = rospy.Subscriber("pi_request", Bool, self.callback6)
        self.pred_sub = rospy.Subscriber("prediction", String, self.callback7)
        self.ml_sub = rospy.Subscriber("/darknet_ros/class_detected", Vector3,  self.callback8)
        self.roll = 0
        self.upper = 0
        self.pid = 0
        self.pitch = 0
        self.yaw = 0
        self.rotate = 0
        self.angle = 0
        self.cnt = 0
        self.start = 0
        self.rd = 0
        self.stop = 0
        self.current_state = 0
        self.next_state = 0
        self.lmotor = 0
        self.rmotor=0
        self.direction = 1
        self.dispatch = 1
        self.blind = 0
        self.pi_dispatch = 0
        self.barcode = ""
        self.prediction = ""
        self.pi_requestx = -1
        self.pi_requesty = -1
        self.p_direction = 0
        self.class1 = self.class2 = self.class3 = -1
    #def getDirection(self, pi_id, x, y):

    def callback4(self,data):
        self.direction=data.data

    def callback0(self, data):  # barcode
        self.barcode = data.data
        print("GOT:", self.barcode)

    def callback5(self, data): #blind
        self.blind=data.data
        if(self.blind):
            print ("BLIIIINNNDDD")

    def callback6(self, data): #pi_request
        # self.pi_requestx=data.x
        # self.pi_requesty=data.y
        try:
            f = Float32()
            f.data=1
        except Exception as e:
            print(e)

    def callback7(self, data): #pi_request
        self.prediction = data.data

    def callback1(self,data): #imu

        self.roll = data.vector.x
        self.pitch = data.vector.y
        self.yaw = data.vector.z
        if (self.yaw < 0):
            self.yaw = 360 + self.yaw
        #self.work()

    def callback2(self, data): #left
        # if(data.data<0):
        # 	self.lmotor=0
        # else:
        self.lmotor=int((data.data)*PID_RATIO)

        #self.work()

    def callback3(self, data): #right
        # if (data.data< 0):
        # 	self.rmotor = 0
        # else:
        self.rmotor = int((data.data) * PID_RATIO)
        #self.work()

    def callback8(self, data): #ml
        self.class1 = data.x
        self.class2 = data.y
        self.class3 = data.z

    def work(self):
        rate = rospy.Rate(50)
        demo = 0
        while not rospy.is_shutdown():
            self.upper = 0
            self.current_state=self.next_state
            print (self.current_state)
            left = 0
            right = 0
            dir = 0
            self.pi_dispatch = 0
            self.pid = 1
            if(self.current_state == IDLE_STATE):
                if(self.blind == 0):
                    self.next_state = DRIVE_STATE
                    dir = 0
                else:
                    self.next_state = IDLE_STATE


            elif(self.current_state == DRIVE_STATE):
                if(self.blind):
	                self.next_state = IDLE_STATE
                elif (self.class1 == -1):
                    self.next_state = DRIVE_STATE
                    left = 40
                    right = 40
                    if(self.lmotor>=0):
                        left += self.lmotor
                    else:
                        right -=self.lmotor
                    if(left<0):
                        left = 0
                    if(right<0):
                        right = 0
                    self.rmotor = self.lmotor = 0
                    dir = 0
                else:
                    self.next_state = BUILDING_STATE
                    self.class1 = -1

            else: #BUILDING STATE
                print("Got as class: ",self.class1)
                #if(self.class1!=-1):
                f = Bool()
                f = True
                try:
                    self.disp_pub.publish(f)
                except Exception as e:
                    print(e)
                self.pi_requestx = self.pi_requesty = -1
                self.next_state = BUILDING_STATE
                # if(self.prediction==""):
                #     self.enable_ml = 1
                #     self.next_state = BUILDING_STATE
                # else:
                #     self.disp_pub = 1
                #self.next_state  = DRIVE_STATE
                demo = 1





            #if(self.pi_requestx !=-1 or self.pi_requesty!=-1):#handle request TEMP FOR DEMO
                # self.p_direction = (self.direction+1)
                # f = Float32()
                # f = 1
                # try:
                #     self.sd_pub.publish(f)
                # except Exception as e:
                #     print(e)
                # self.pi_requestx = self.pi_requesty = -1


            v3 = Vector3()
            v = Vector3Stamped()
            self.cnt+=1
            head = Header()
            head.stamp = rospy.Time.now()
            head.seq= self.cnt
            v3.x=left
            v3.y=right
            v3.z=dir
            v.header = head
            v.vector=v3
            pid_bool = Bool()
            pid_bool = self.pid
            db = Bool()
            db = self.pi_dispatch

            #print ("left=%d, right=%d"%(left,right))
            try:
                self.wheels_publish.publish(v)
                self.pid_publish.publish(pid_bool)
                #self.disp_pub.publish(db)
            except Exception as e:
                print(e)
            rate.sleep()




def main(args):
    ic = master_control()
    ic.work()


if __name__ == '__main__':
    main(sys.argv)
