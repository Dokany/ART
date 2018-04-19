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
ROTATE_STATE = 2
INTERSECTION_STATE = 3
STRAIGHT = 8
RIGHT = 9
LEFT = 10
PID_RATIO = 40.0/1000.0
class master_control:

    def __init__(self):
        rospy.init_node('MASTER_CONTROL', anonymous=True)
        self.wheels_publish = rospy.Publisher("pi/wheels_speed",Vector3Stamped, queue_size=10)
        self.pid_publish = rospy.Publisher("pi/pid_start", Bool, queue_size=10)
        self.angles_sub = rospy.Subscriber("pi/imu_angles",Vector3Stamped, self.callback1)
        self.lw_sub = rospy.Subscriber("pi/left_wheel/control_effort", Float64, self.callback2)
        self.rw_sub = rospy.Subscriber("pi/right_wheel/control_effort", Float64, self.callback3)
        self.sd_sub = rospy.Subscriber("sign_direction_pi", Float32, self.callback4)
        self.disp = rospy.Subscriber("dispatch_pi", Bool, self.callback0)
        self.blind = rospy.Subscriber("pi/blind", Bool, self.callback5)
        self.cam_pub = rospy.Publisher("pi/camera", Bool, queue_size = 10)
        self.req_pub = rospy.Publisher("pi_request", Vector3, queue_size = 10)
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
        self.direction = -1
        self.dispatch = 0
        self.blind = 0
        self.switch_cam = 0

    def callback0(self, data): #dispatch
        self.dispatch=data.data

    def callback5(self, data): #blind
        self.blind=data.data
        if(self.blind):
            print ("BLIIIINNNDDD")

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

    def callback4(self, data):  # direction
        self.direction=int(data.data)
        #self.work()

    def work(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            self.upper = 0
            self.current_state=self.next_state
            print (self.current_state)
            left = 0
            right = 0
            dir = 0
            self.pid = 0
            if(self.current_state == IDLE_STATE):
                if(self.dispatch):
                    self.next_state = DRIVE_STATE
                    self.pid = 1
                    self.dispatch = 0
                    dir = 0
                else:
                    self.next_state = IDLE_STATE


            elif(self.current_state == DRIVE_STATE):
                if (self.blind == 0):
                    self.next_state = DRIVE_STATE
                    left = 50 - self.lmotor
                    right = 50 + self.lmotor
                    if(left<0):
                        left = 0
                    if(right<0):
                        right = 0
                    if(self.lmotor==0 and self.rmotor==0):
                        left+=5
                        right+=5
                    self.rmotor = self.lmotor = 0
                    if(self.upper):
                        #rospy.sleep(0.2)
                        self.upper = 0
                    dir = 0
                elif(self.switch_cam):
                    self.switch_cam=0
                    rospy.sleep(2)
                    self.upper = 1
                else:

                    rospy.sleep(0.7)
                    self.next_state = INTERSECTION_STATE
                    try:
                        v=Vector3()
                        self.req_pub.publish(v)
                    except Exception as e:
                        print(e)
                    #print("DIRECTION : ",self.direction)
                    # if(self.direction==0):
                    #
                    #     self.switch_cam = 1
                    #     self.next_state = DRIVE_STATE
                    # else:
                    #     dir=self.direction
                    #     self.start=0
                    #     self.next_state = ROTATE_STATE
                    #
                    # self.request = 0
                self.pid = 1

            elif(self.current_state == INTERSECTION_STATE):
                if(self.direction==-1):
                    self.next_state= INTERSECTION_STATE
                elif(self.direction==0):
                    self.next_state=DRIVE_STATE
                else:
                    self.next_state = ROTATE_STATE
                    left=right=0
                    dir=self.direction
                    self.start=0
                self.direction=-1
            else: #ROTATE
                if(self.start==0):
                    self.angle=self.yaw
                    self.start = 1
                    dir = self.direction
                    self.next_state = ROTATE_STATE
                else:
                    #print ('current = %f, yaw = %f'%(self.angle, self.yaw))


                    if(abs(self.angle-self.yaw)>=90):
                        self.next_state = DRIVE_STATE
                        self.switch_cam = 1
                        self.pid = 1
                        #rospy.sleep(1)

                    else:
                        self.next_state = ROTATE_STATE
                        left = 50
                        right = 50
                        self.pid = 1
                        dir = self.direction
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
            cam_bool = Bool()
            cam_bool = self.upper
            #print ("left=%d, right=%d"%(left,right))
            try:
                self.wheels_publish.publish(v)
                self.cam_pub.publish(cam_bool)
                self.pid_publish.publish(pid_bool)

            except Exception as e:
                print(e)
            rate.sleep()




def main(args):
    ic = master_control()
    ic.work()


if __name__ == '__main__':
    main(sys.argv)
