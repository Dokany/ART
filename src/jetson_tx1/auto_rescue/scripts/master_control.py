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
LOST_STATE = 3


INSTR_WAIT = 0
INSTR_RIGHT = 1
INSTR_LEFT = 2
INSTR_STRAIGHT = 3
INSTR_DEST_LEFT = 4
INSTR_DEST_RIGHT = 5
INSTR_PARK = 6

PID_RATIO = 80.0/1000.0


class master_control:

    def __init__(self):
        rospy.init_node('MASTER_CONTROL', anonymous=True)
        self.wheels_publish = rospy.Publisher("jetson/wheels_speed",Vector3Stamped, queue_size=10)
        self.pid_publish = rospy.Publisher("jetson/pid_start", Bool, queue_size=10)
        self.sd_pub_ff = rospy.Publisher("reply_ff", Float32,  queue_size=10)
        self.sd_pub_amb = rospy.Publisher("reply_amb", Float32,  queue_size=10)
        self.sd_pub_pol = rospy.Publisher("reply_pol", Float32,  queue_size=10)
        self.disp_pub_ff = rospy.Publisher("dispatch_ff", Bool,  queue_size=10)
        self.disp_pub_amb = rospy.Publisher("dispatch_amb", Bool,  queue_size=10)
        self.disp_pub_pol = rospy.Publisher("dispatch_pol", Bool,  queue_size=10)
              
        self.bar_sub = rospy.Subscriber("barcode", String, self.callback0)
        self.cam_sub = rospy.Subscriber("jetson/blind", Bool, self.callback1)
        self.req_sub = rospy.Subscriber("pi_request", Bool, self.callback2)
        self.ml_sub = rospy.Subscriber("/darknet_ros/class_detected", Vector3,  self.callback3)
        self.lw_sub = rospy.Subscriber("jetson/control_effort", Float64, self.callback4)  
        
        self.roll = 0
        self.upper = 0
        self.yaw = 0
        self.pid = 0
        self.pitch = 0
        self.rotate = 0
        self.angle = 0
        self.cnt = 0
        self.start = 0
        self.rd = 0
        self.stop = 0
        self.current_state = 0
        self.next_state = 0
        self.motor=0
        self.direction = 1
        self.dispatch = 1
        self.blind = 0
        self.pi_dispatch = 0
        self.barcode = ""
        self.pi_request = 0
        self.p_direction = 0
        self.class1 = self.class2 = self.class3 = -1
        
    def callback0(self,data):# barcode
        self.barcode=data.data

    def callback1(self, data): #blind 
        self.blind=data.data
        if(self.blind):
            print ("BLIIIINNNDDD")

    def callback2(self, data): #pi_request
        self.pi_request = data.data
        #add ML logic and send reply self.sd_pub_ff(_amb or _pol)


    def callback3(self, data): #ml
        self.class1 = data.x
        self.class2 = data.y
        self.class3 = data.z
                
    def callback4(self, data): #motor
        self.motor = int((data.data) * PID_RATIO)

    

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
                else:
                    self.next_state = IDLE_STATE

            elif(self.current_state == LOST_STATE):
                print("LOST")
                self.next_state = LOST_STATE

            elif(self.current_state == DRIVE_STATE):
                if(self.blind):
                    self.next_state = LOST_STATE
                elif (self.barcode == ""):
                    self.next_state = DRIVE_STATE
                    left = 40
                    right = 40
                    if(self.motor>=0):
                        left += self.motor
                    else:
                        right -=self.motor
                    if(left<0):
                        left = 0
                    if(right<0):
                        right = 0
                    self.motor = 0
                    dir = 0
                else:
                    self.next_state = BUILDING_STATE
                    self.class1 = self.class2 = self.class3 = -1

            else: #BUILDING STATE
                rospy.sleep(0.5)
                if(self.class1!=-1):#no emergency
                    self.next_state = DRIVE_STATE
                else:
                    print("Got :",self.class1)
                    #dispatch
                    b = Bool()
                    b = True
                    try:
                        if(self.class1==0):
                            self.disp_pub_ff.publish(b)
                        elif(self.class1==1):
                            self.disp_pub_amb.publish(b)
                        else:
                            self.disp_pub_pol.publish(b)
                    except Exception as e:
                        print(e)
                    self.class1 = self.class2 = self.class3 = -1
                    self.next_state = DRIVE_STATE
                

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
            try:
                self.wheels_publish.publish(v)
                self.pid_publish.publish(pid_bool)
            except Exception as e:
                print(e)
            rate.sleep()




def main(args):
    ic = master_control()
    ic.work()


if __name__ == '__main__':
    main(sys.argv)
