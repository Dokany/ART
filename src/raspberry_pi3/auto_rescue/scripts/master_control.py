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
NODE_STATE = 2
PARK_STATE = 3
ROTATE_STATE = 4
BUILDING_STATE = 5
LOST_STATE = 6
INTERSECTION_STATE = 7
 

PID_RATIO = 40.0/1000.0

INSTR_WAIT = 0
INSTR_RIGHT = 1
INSTR_LEFT = 2
INSTR_STRAIGHT = 3
INSTR_DEST_LEFT = 4
INSTR_DEST_RIGHT = 5
INSTR_PARK = 6

class master_control:

    def __init__(self):
        rospy.init_node('MASTER_CONTROL', anonymous=True)
        self.wheels_publish = rospy.Publisher("pi/wheels_speed",Vector3Stamped, queue_size=10)
        self.pid_publish = rospy.Publisher("pi/pid_start", Bool, queue_size=10)        
        self.sd_sub = rospy.Subscriber("reply_pi", Float32, self.callback0)
        self.angles_sub = rospy.Subscriber("pi/imu_angles",Vector3Stamped, self.callback1)
        self.pid_sub = rospy.Subscriber("pi/control_effort", Float64, self.callback2)
        self.disp = rospy.Subscriber("dispatch_pi", Bool, self.callback3)
        self.blind = rospy.Subscriber("pi/blind", Bool, self.callback4)
        self.cam_pub = rospy.Publisher("pi/camera", Bool, queue_size = 10)
        #NEW
        self.req_pub = rospy.Publisher("pi_request", Bool, queue_size = 10)
        self.junction = rospy.Subscriber("pi/junction", Bool, self.callback5)
             
    def callback0(self, data):  # reply
        self.reply=int(data.data)
        print("instruction got: ", data.data)

    def callback1(self,data): #imu
        self.roll = data.vector.x
        self.pitch = data.vector.y
        self.yaw = data.vector.z
        if (self.yaw < 0):
            self.yaw = 360 + self.yaw

    def callback2(self, data): #left
        self.motor=int((data.data)*PID_RATIO)
        
    def callback3(self, data): #dispatch
        self.dispatch=data.data
        print("dispatched!")

    def callback4(self, data): #blind
        self.blind=data.data
        if(self.blind):
            print ("BLIND")
            
    def callback5(self, data): #junction
        self.junction=data.data
        if(self.blind):
            print ("JUNCTION")
            

    def work(self):
        rate = rospy.Rate(50)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0        
        self.upper = 0        
        self.rotate = 0
        self.angle = 0        
        self.start = 0
        self.current_state = 0
        self.next_state = 0
        self.prev_unique_state = 0
        self.motor=0
        self.reply = -1
        self.dispatch = 0
        self.blind = 0
        self.direction = 0
        self.junction = 0
        self.visit_done = 0
        dir = 0
        while not rospy.is_shutdown():
            if(self.next_state != self.current_state): #keep track of last (unique) state
                self.prev_unique_state = self.current_state
            self.current_state=self.next_state
            print (self.current_state)
            left = 0
            right = 0
            self.pid = 0
            self.upper = 0
            self.request = 0

            if(self.current_state == IDLE_STATE):
                if(self.dispatch==0):
                    self.next_state = IDLE_STATE
                else:                    
                    self.reply = -1 #initialize reply as invalid
                    self.next_state = DRIVE_STATE
                    
            elif(self.current_state = LOST_STATE):
                print("LOST! HELP! JOHN EL SABAB")
                
            elif(self.current_state == DRIVE_STATE):
                if(self.blind):
                    if(self.reply==-1): #no junction
                        self.next_state = LOST_STATE
                    else:#intersection
                        rospy.sleep(0.7) 
                        self.upper=1
                        self.next_state = INTERSECTION_STATE
                        
                elif(self.junction):
                    self.next_state = NODE_STATE
                    self.request = 1
                else:
                    self.pid = 1
                    left = 50 - self.lmotor
                    right = 50 + self.lmotor
                    dir = 0
                    if(left<0):
                        left = 0
                    if(right<0):
                        right = 0
                    self.next_state = DRIVE_STATE

            elif(self.current_state == NODE_STATE):
                if(self.reply<=0): #no reply yet or asked to wait
                    self.next_state = NODE_STATE
                elif(self.reply == INSTR_DEST_LEFT or self.reply==INSTR_DEST_RIGHT):
                    self.next_state = BUILDING_STATE
                    if(self.reply == INSTR_DEST_LEFT):
                        dir = 1
                    else:
                        dir = 2
                elif(self.reply == INSTR_PARK):
                    self.next_state = PARK_STATE
                    self.start = 0
                else:
                    dir = 0
                    self.next_state = DRIVE_STATE


            elif(self.current_state == BUILDING_STATE):
                if(self.prev_unique_state == NODE_STATE):
                    self.next_state = ROTATE_STATE
                elif(self.visit_done ==0):
                    if(dir==1):
                        dir = 2
                    else:
                        dir = 1
                    self.next_state = ROTATE_STATE
                    self.visit_done = 1
                    
                else:
                    self.next_state = NODE_STATE                            
                    self.request = 1
                    self.reply = -1
                    
            elif(self.current_state == INTERSECTION_STATE):
                if(self.reply==3): #straight
                    self.reply = -1
                    if(self.upper == 0):
                        self.upper= 1
                        self.next_state = INTERSECTION_STATE
                    else:
                        self.next_state=DRIVE_STATE
                        self.upper = 0
                else:
                    self.next_state = ROTATE_STATE
                    left=right=0
                    self.start=0
                
            elif(self.current_state == PARK_STATE):
                #PARK
                
            else: #ROTATE
                if(self.start==0):
                    self.angle=self.yaw
                    self.start = 1
                    self.next_state = ROTATE_STATE
                    
                else:
                    if(abs(self.angle-self.yaw)>=90):
                        self.next_state = self.prev_unique_state                      
                    else:
                        self.next_state = ROTATE_STATE
                        left = 50
                        right = 50
                        
            v3 = Vector3()
            v = Vector3Stamped()
            head = Header()
            head.stamp = rospy.Time.now()
            v3.x=left
            v3.y=right
            v3.z=dir
            v.header = head
            v.vector=v3
            pid_bool = Bool()
            pid_bool = self.pid
            cam_bool = Bool()
            cam_bool = self.upper
            req_bool = Bool()
            req_bool = self.request
            try:
                self.wheels_publish.publish(v)
                self.cam_pub.publish(cam_bool)
                self.req_pub.publish(req_bool)
                self.pid_publish.publish(pid_bool)

            except Exception as e:
                print(e)
            rate.sleep()




def main(args):
    ic = master_control()
    ic.work()


if __name__ == '__main__':
    main(sys.argv)
