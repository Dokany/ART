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
INSTR_LEFT = 1
INSTR_RIGHT = 2
INSTR_STRAIGHT = 3
INSTR_DEST_LEFT = 4
INSTR_DEST_RIGHT = 5
INSTR_PARK = 6

class master_control:

    def __init__(self):
        rospy.init_node('MASTER_CONTROL', anonymous=True)
        self.wheels_publish = rospy.Publisher("pi_fire/wheels_speed",Vector3Stamped, queue_size=10)
        self.pid_publish = rospy.Publisher("pi_fire/pid_start", Bool, queue_size=10)
        self.sd_sub = rospy.Subscriber("reply_fire", Float32, self.callback0)
        self.angles_sub = rospy.Subscriber("pi_fire/imu_angles",Vector3Stamped, self.callback1)
        self.pid_sub = rospy.Subscriber("pi_fire/control_effort", Float64, self.callback2)
        self.disp = rospy.Subscriber("dispatch_fire", Bool, self.callback3)
        self.blind = rospy.Subscriber("pi_fire/blind", Bool, self.callback4)
        self.cam_pub = rospy.Publisher("pi_fire/camera", Bool, queue_size = 10)
        #NEW
        self.req_pub = rospy.Publisher("pi_request_fire", Bool, queue_size = 10)
        self.junction = rospy.Subscriber("pi_fire/junction", Bool, self.callback5)
             
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
        if(self.junction):
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
        self.direction = -1
        self.current_state = 0
        self.next_state = 0
        self.prev_unique_state = 0
        self.motor=0
        self.reply = 0
        self.blind = 0
        self.rotate_ret = 0
        self.dispatch = 0
        self.pid = 0
        self.junction = 0
        self.visit_done = 0
        self.prev_right = self.prev_left = 0
        dir = 0
        self.dummy_node = self.dummy_inter = 0
        while not rospy.is_shutdown():
            self.current_state = self.next_state
            if(self.next_state != self.prev_unique_state): #keep track of last (unique) state
                self.prev_unique_state = self.current_state
                print ("STATE:",self.current_state)
            left = 0
            right = 0
            self.pid = 1
            self.request = 0
            if(self.current_state == IDLE_STATE):
                if(self.dispatch == 0):
                    self.next_state = IDLE_STATE
                    self.dispatch = 0
                else:                    
                    #self.reply = -1 #initialize reply as invalid
                    self.next_state = DRIVE_STATE
                    
            elif(self.current_state == LOST_STATE):
                print("LOST!")
                break
                
            elif(self.current_state == DRIVE_STATE):
                if(self.blind):
                    print ("dir at drive blind", self.direction)
                    if(self.direction==-1): #no junction
                        self.next_state = IDLE_STATE
                    else:#intersection
                        right = self.prev_right
                        left = self.prev_left
                        self.upper = 0
                        self.dummy_inter = 0
                        self.next_state = INTERSECTION_STATE


                elif(self.junction):
                    if(self.dummy_node==7):
                        self.next_state = NODE_STATE
                        self.request = 1
                        self.dummy_node = 0
                        self.junction=0
                    else:
                        print("pid inside junc")
                        self.dummy_node+=1
                        self.pid = 1
                        right = 50
                        left = 50
                        if (self.motor >= 0):
                            left -= self.motor
                        else:
                            right += self.motor
                        dir = 0
                        if (left < 0):
                            left = 0
                        if (right < 0):
                            right = 0
                        self.next_state = DRIVE_STATE


                else:
                    self.pid = 1
                    self.upper = 1
                    right = 50
                    left = 50
                    if(self.motor>=0):
                        left -= self.motor
                    else:
                        right +=self.motor
                    dir = 0
                    if(left<0):
                        left = 0
                    if(right<0):
                        right = 0
                    self.next_state = DRIVE_STATE

            elif(self.current_state == NODE_STATE):
                print("self.reply",self.reply)
                if(self.reply<=0): #no reply yet or asked to wait
                    self.next_state = NODE_STATE
                elif(self.reply == INSTR_DEST_LEFT or self.reply==INSTR_DEST_RIGHT):
                    self.next_state = BUILDING_STATE
                    if(self.reply == INSTR_DEST_LEFT):
                        self.direction = 1
                    else:
                        self.direction = 2
                elif(self.reply == INSTR_PARK):
                    self.next_state = PARK_STATE
                    self.start = 0
                else:
                    self.direction = self.reply
                    self.next_state = DRIVE_STATE


            elif(self.current_state == BUILDING_STATE):
                if(self.prev_unique_state == NODE_STATE):
                    self.next_state = ROTATE_STATE
                    self.rotate_ret = NODE_STATE
                elif(self.visit_done ==0):
                    if(dir==1):
                        dir = 2
                    else:
                        dir = 1
                    self.next_state = ROTATE_STATE
                    self.rotate_ret = NODE_STATE
                    self.visit_done = 1
                    
                else:
                    self.next_state = NODE_STATE                            
                    self.request = 1
                    self.reply = -1
                    
            elif(self.current_state == INTERSECTION_STATE):
                print (self.direction)
                if(self.direction<=3 or self.direction>=1): #straight
                    if(self.upper == 0 or (self.upper==1 and self.dummy_inter>0 and self.dummy_inter<65)):
                        self.upper= 1
                        self.dummy_inter+=1

                        left = self.prev_left
                        right = self.prev_right
                        print("at int", right, left)
                        dir = 0
                        self.next_state = INTERSECTION_STATE
                    else:
                        if(self.direction==3):
                            self.next_state=DRIVE_STATE
                        else:
                            self.next_state = ROTATE_STATE
                            self.rotate_ret = DRIVE_STATE
                            dir = self.direction
                        
                        self.upper = 0
                else:
                    self.next_state = ROTATE_STATE
                    left=right=0
                    self.start=0
                
            elif(self.current_state == PARK_STATE):
                #PARK
                pass
                
            else: #ROTATE
                if(self.start==0):
                    self.angle=self.yaw
                    self.start = 1
                    self.next_state = ROTATE_STATE
                    
                else:
                    print("yaw=%f, current=%f"%(self.yaw,self.angle))
                    if(abs(self.angle-self.yaw)>=80):
                        self.next_state =  self.rotate_ret
                        print("return to state %d from rot"%(self.rotate_ret))
                        self.reply = INSTR_DEST_RIGHT
                        self.upper = 1
                    else:
                        self.next_state = ROTATE_STATE
                        left = 40
                        right = 40
                        dir=self.direction
                        print("rotating ", self.direction)


            self.prev_right = right
            self.prev_left = left
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
