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
import networkx as nx
import random

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

POLICE_POS = 0
AMBULANCE_POS = 0
FIREFIGHTER_POS = "6"

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
        self.req_sub = rospy.Subscriber("pi_request_ff", Bool, self.callback2)
        #self.req_sub = rospy.Subscriber("pi_request_amb", Bool, self.callback5)
        #self.req_sub = rospy.Subscriber("pi_request_pol", Bool, self.callback6)
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
        self.ff_target = self.pol_target =  self.amb_target = -1
    def callback0(self,data):# barcode
        self.barcode=data.data

    def callback1(self, data): #blind
        self.blind=data.data
        if(self.blind):
            print ("BLIIIINNNDDD")

    def callback2(self, data): #ff_request
        #add ML logic and send reply self.sd_pub_ff(_amb or _pol)
        if(len(self.ff_current_path)==0): #no path calculated yet
            dagu1_array = []
            dagu2_array = []
            if (self.dagu_pos["ambulance"]!=None):

                for path in nx.all_simple_paths(self.FG, source=FIREFIGHTER_POS, target=self.ff_target):
                    dagu1_array.append(path)
                for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["ambulance"], target=self.amb_target):
                    dagu2_array.append(path)
                self.ff_current_path, self.amb_current_path = self.get_combined_shortest_path(dagu1_array,dagu2_array)
            else:

                self.ff_current_path = nx.shortest_path(self.FG, source=self.dagu_pos["ambulance"], target=self.amb_target)
            index = 0

        else:
            index  = self.ff_current_path.index(self.dagu_pos["firefighter"])

        index+=2

        reply = 0
        if(index>=len(self.ff_current_path)):
            reply = INSTR_DEST_LEFT
            self.ff_target = FIREFIGHTER_POS
        else:
            next_pos = self.ff_current_path[index]
            intersection = self.ff_current_path[index-1]
            previous = self.ff_current_path[index-2]
            dir = self.get_direction(previous, intersection, next_pos)
            self.dagu_pos["firefighter"] =  self.FG[next_pos]

            if(dir == "right"):
                reply = INSTR_RIGHT
            elif(dir == "left"):
                reply = INSTR_LEFT
            elif(dir=="straight"):
                reply = INSTR_STRAIGHT
            else:
                reply = 0


    def callback3(self, data): #ml
        self.class1 = data.x
        self.class2 = data.y
        self.class3 = data.z

    def callback4(self, data): #motor
        self.motor = int((data.data) * PID_RATIO)


    ##### Graph handling
    def get_direction(self,previous, intersection, next):

        list_temp = []
        index = -1

        if 'list' in self.FG[intersection]:
            list_temp = self.FG[intersection]['list']
            list_temp = list_temp.split()

        index = list_temp.index(previous)

        if ((index + 1) % 4 == list_temp.index(next)):
            return "left"

        if ((index + 2) % 4 == list_temp.index(next)):
            return "straight"

        if ((index + 3) % 4 == list_temp.index(next)):
            return "right"

        return "unknown"

    def generate_n_shift(self,x, arr, duplicate_index):

        temp = arr[:]
        for i in range(0, x):
            #         print("ARR SIZE = ", len(arr))
            #         print("ARRAYYYYYY = ", arr)
            #         print("DUPLICATE INDEX INSIDE FUNCTION PLEASE ", duplicate_index)

            temp.insert(duplicate_index, temp[duplicate_index])
        return temp


    def remove_duplicates(self,l):
        s = []
        for i in l:
            if i not in s:
                s.append(i)

        return s

    def get_combined_shortest_path(self,dagu1_list, dagu2_list):
        minimum_path1 = []
        minimum_path2 = []
        flag = 1
        minimum_cost = 10000
        generate_num = 0
        generate_bound = 25

        for path1 in dagu1_list:
            for path2 in dagu2_list:
                minimum = min(len(path1), len(path2))
                for i in range(0, minimum):
                    if path1[i] == path2[i] or flag == 0:
                        flag = 0
                        if not (path2[i - 1] == path2[i] or path2[i - 1] == path1[i] or path2[i] == path2[
                            i - 2] or generate_num >= generate_bound):
                            print("path1[i-1]", path1[i - 1])
                            print("path1[i]", path1[i])
                            print(self.generate_n_shift(1, path2, i - 1))
                            dagu2_list.append(self.generate_n_shift(1, path2, i - 1))
                            generate_num = generate_num + 1
                            break
                #                     dagu1_list = generate_possibilities(dagu1_list,1,i-1)

                if flag == 0:
                    print("Eliminate Path1", path1)
                    print("Eliminate Path2", path2)
                    print("--------------------------------------")
                else:
                    print("Insert Path1", path1)
                    print("Insert Path2", path2)
                    print("Cost = ", len(path1) + len(path2))
                    cost = len(path1) + len(path2)
                    if cost < minimum_cost:
                        minimum_path1 = path1
                        minimum_path2 = path2
                        minimum_cost = cost

                    print("--------------------------------------")
                flag = 1

        print("Minimum Path1", minimum_path1)
        print("Minimum Path2", minimum_path2)
        print("Cost = ", minimum_cost)
        return minimum_path1, minimum_path2

    def generate_graph(self):
        # Creating The Graph
        self.FG = nx.Graph()
        self.FG.add_edges_from( [
                    
                    ('1','i1'),
                    ('1','i2'),
                    ('2','i2'),
                    ('2','i3'),
                    ('3','i3'),
                    ('3','i4'),
                    
                    ('4','i4'),
                    ('4','i5'),
                    ('5','i3'),
                    ('5','i6'),
                    ('6','i2'),
                    ('6','i7'),
    
                    ('7','i1'),
                    ('7','i8'),
                    ('8','i8'),
                    ('8','i7'),
                    ('9','i7'),
                    ('9','i6'),
    
                    ('10','i6'),
                    ('10','i5'),
                    ('11','i5'),
                    ('11','i11'),
                    ('12','i6'),
                    ('12','i10'),
    
                    ('13','i7'),
                    ('13','i9'),
                    ('14','i9'),
                    ('14','i10'),
                    ('15','i10'),
                    ('15','i11'),

                    ('16','i11'),
                    ('16','i12'),
                    ('17','i10'),
                    ('17','i13'),
                    ('18','i9'),
                    ('18','i14'),
                    ('19','i13'),
                    ('19','i14'),
                    ('20','i12'),
                    ('20','i13'),
                  
                   ])

        self.FG.node['i1']['list'] = '-1 -1 7 1'
        self.FG.node['i2']['list'] = '-1 1 6 2'
        self.FG.node['i3']['list'] = '-1 2 5 3'
        self.FG.node['i4']['list'] = '-1 3 4 -1'
        self.FG.node['i5']['list'] = '4 10 11 -1'
        self.FG.node['i6']['list'] = '5 9 12 10'
        self.FG.node['i7']['list'] = '6 8 13 9'

        self.FG.node['i8']['list'] = '7 -1 -1 8'
        self.FG.node['i9']['list'] = '13 -1 18 14'
        self.FG.node['i10']['list'] = '12 14 17 15'
        self.FG.node['i11']['list'] = '11 15 16 -1'
        self.FG.node['i12']['list'] = '16 20 -1 -1'
        self.FG.node['i13']['list'] = '17 19 -1 20'
        self.FG.node['i14']['list'] = '18 s1 -1 19'

        #self.FG.node['i1']['previous'] = '3'

        self.dagu_pos = {'firefighter': None, 'ambulance': None, 'police': None}
        self.amb_current_path = []
        self.pol_current_path = []
        self.ff_current_path = []

    def work(self):
        rate = rospy.Rate(50)
        demo = 0
        print("0")
        self.prev_barcode = "aaa"
        self.dispatch = 0
        self.generate_graph()
        while not rospy.is_shutdown():
            self.upper = 0
            self.current_state=self.next_state
            if(self.next_state!=self.current_state):
                print (self.next_state)
            left = 0
            self.dispatch = 0
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
                    self.next_state = IDLE_STATE
                elif (self.barcode == "" or (self.barcode==self.prev_barcode)):
                    self.next_state = DRIVE_STATE
                    left = 35
                    right = 35
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
                    print("GOT QRCODE")
                    self.prev_barcode = self.barcode
                    self.barcode = ""
                    rospy.sleep(0.5)

            else: #BUILDING STATE

                rospy.sleep(2)
                if(self.class1==-1):#no emergency
                    self.next_state = DRIVE_STATE
                else:
                    print("dispatching ",self.class1)
                    #dispatch
                    b = Bool()
                    b = True
                    if(self.class1==0):
                        self.dispatch = 1
                        self.ff_target = "18"
                        self.dagu_pos['firefighter'] = FIREFIGHTER_POS #CHANGE
                    elif(self.class1==1):
                        self.dispatch =2
                        self.amb_target = self.barcode
                        self.dagu_pos['ambulance'] = AMBULANCE_POS #CHANGE
                    else:
                        self.dispatch = 3
                        self.pol_target = self.barcode
                        self.dagu_pos['police'] = POLICE_POS #CHANGE
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
                if(self.dispatch>0):
                    dib = Bool()
                    dib = True
                    if(self.dispatch==0):
                        self.disp_pub_ff.publish(dib)
                    elif(self.dispatch == 2):
                        self.disp_pub_amb.publish(dib)
                    else:
                        self.disp_pub_pol.publish(dib)

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
