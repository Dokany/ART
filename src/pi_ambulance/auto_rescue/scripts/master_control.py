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

PID_RATIO = 20.0 / 1000.0

INSTR_WAIT = 0
INSTR_LEFT = 2
INSTR_RIGHT = 1
INSTR_STRAIGHT = 3
INSTR_DEST_LEFT = 4
INSTR_DEST_RIGHT = 5
INSTR_PARK = 6
INSTR_BACK_STRAIGHT = 7
INSTR_BACK_LEFT = 8
INSTR_BACK_RIGHT = 9

class master_control:

    def __init__(self):
        rospy.init_node('MASTER_CONTROL', anonymous=True)
        self.wheels_publish = rospy.Publisher("pi_amb/wheels_speed",Vector3Stamped, queue_size=10)
        self.pid_publish = rospy.Publisher("pi_amb/pid_start", Bool, queue_size=10)
        self.sd_sub = rospy.Subscriber("reply_amb", Float32, self.callback0)
        self.angles_sub = rospy.Subscriber("pi_amb/imu_angles",Vector3Stamped, self.callback1)
        self.pid_sub = rospy.Subscriber("pi_amb/control_effort", Float64, self.callback2)
        self.disp = rospy.Subscriber("dispatch_amb", Bool, self.callback3)
        self.blind = rospy.Subscriber("pi_amb/blind", Bool, self.callback4)
        self.cam_pub = rospy.Publisher("pi_amb/camera", Bool, queue_size = 10)
        #NEW
        self.req_pub = rospy.Publisher("pi_request_amb", Bool, queue_size = 100)
        self.junction = rospy.Subscriber("pi_amb/junction", Bool, self.callback5)

    def callback0(self, data):  # reply
        self.reply = int(data.data)

    def callback1(self, data):  # imu
        self.roll = data.vector.x
        self.pitch = data.vector.y
        self.yaw = data.vector.z
        if (self.yaw < 0):
            self.yaw = 360 + self.yaw

    def callback2(self, data):  # left
        self.motor = int((data.data) * PID_RATIO)

    def callback3(self, data):  # dispatch

        if (data.data):
            # print("dispatched!")
            self.dispatch = data.data

    def callback4(self, data):  # blind
        self.blind = data.data
        if (self.blind):
            print("BLIND")

    def callback5(self, data):  # junction
        self.junction = data.data
        if (self.junction):
            print("JUNCTION")

    def work(self):
        rate = rospy.Rate(30)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.upper = 0
        self.rotate = 0
        self.angle = 0
        self.dispatched = 0
        self.start = 0
        self.direction = -1
        self.node_already = 0
        self.current_state = 0
        self.turn = 0
        self.next_state = 0
        self.prev_unique_state = 0
        self.motor = 0
        self.reply = 0
        self.blind = 0
        self.rotate_ret = 0
        self.dispatch = 0
        self.pid = 0
        self.junction = 0
        self.parking = 1
        self.visit_done = 0
        self.prev_right = self.prev_left = 0
        dir = 0
        self.dummy_node = self.dummy_inter = 0
        while not rospy.is_shutdown():
            if (self.next_state != self.current_state):  # keep track of last (unique) state
                self.prev_unique_state = self.current_state
                print("STATE:", self.current_state)

            self.current_state = self.next_state
            left = 0
            right = 0
            self.pid = 1
            self.request = 0
            if (self.current_state == IDLE_STATE):
                if (self.blind or self.dispatch == 0):
                    self.next_state = IDLE_STATE
                else:
                    # self.reply = -1 #initialize reply as invalid
                    # print("Dispatched!!!")
                    self.next_state = DRIVE_STATE
                    self.dispatch = 0

            elif (self.current_state == LOST_STATE):
                print("LOST!")
                break

            elif (self.current_state == DRIVE_STATE):
                if (self.blind):
                    print("dir at drive blind", self.direction)
                    if (self.direction == -1):  # no junction
                        # print("NO JUNCTION")
                        self.next_state = IDLE_STATE
                    else:  # intersection
                        print("INNTERRRRSECTIONNNN!!!")
                        right = self.prev_right
                        left = self.prev_left
                        self.upper = 1
                        self.dummy_inter = 0
                        self.next_state = INTERSECTION_STATE


                elif (self.junction):
                    if (self.dummy_node == 8):  # 10 daylight, 4 with leds
                        self.next_state = NODE_STATE
                        self.request = 1
                        self.dummy_node = 0
                        self.junction = 0
                    else:
                        print("pid inside junc")
                        self.dummy_node += 1
                        self.pid = 1
                        right = 40
                        left = 40
                        if (self.motor >= 0):
                            left -= self.motor
                        else:
                            right += self.motor
                        dir = 0
                        if (left < 0):
                            left = 0
                        if (right < 0):
                            right = 0
                        self.node_already = 1
                        self.next_state = DRIVE_STATE


                else:
                    self.pid = 1
                    right = 40
                    left = 40
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

            elif (self.current_state == NODE_STATE):
                # if(self.node_already):
                #     self.next_state=DRIVE_STATE
                # else:
                self.node_already = 1
                if (self.reply > 0):
                    print("self.reply", self.reply)
                if (self.reply <= 0):  # no reply yet or asked to wait
                    self.next_state = NODE_STATE
                    # self.request = 1
                elif (self.reply == INSTR_DEST_LEFT or self.reply == INSTR_DEST_RIGHT):
                    self.next_state = LOST_STATE
                    if (self.reply == INSTR_DEST_LEFT):
                        self.direction = 1
                    else:
                        self.direction = 2
                elif (self.reply == INSTR_PARK):
                    self.next_state = PARK_STATE
                    self.start = 0
                elif (self.reply >= INSTR_BACK_STRAIGHT and self.reply <= INSTR_BACK_RIGHT):
                    print("Back.........")
                    if (self.turn <= 1):
                        self.start = 0
                        self.next_state = ROTATE_STATE
                        self.rotate_ret = NODE_STATE
                        self.turn += 1
                    else:
                        self.turn = 0
                        self.next_state = DRIVE_STATE
                        if (self.reply == INSTR_BACK_RIGHT):
                            self.reply = INSTR_RIGHT
                        elif (self.reply == INSTR_BACK_STRAIGHT):
                            self.reply = 3
                        else:
                            self.reply = INSTR_LEFT
                        self.upper = 0
                        self.dummy_node = 0

                    self.direction = self.reply
                else:
                    self.direction = self.reply
                    self.next_state = DRIVE_STATE
                    self.pid = 1
                    self.upper = 0


            elif (self.current_state == BUILDING_STATE):
                if (self.prev_unique_state == NODE_STATE):
                    self.next_state = ROTATE_STATE
                    self.start = 0
                    self.rotate_ret = NODE_STATE
                elif (self.visit_done == 0):
                    if (dir == 1):
                        dir = 2
                    else:
                        dir = 1
                    self.next_state = ROTATE_STATE
                    self.rotate_ret = NODE_STATE
                    self.visit_done = 1

                else:
                    self.next_state = NODE_STATE
                    self.request = 1
                self.reply = 0

            elif (self.current_state == INTERSECTION_STATE):
                self.node_already = 0
                # print("INTERSECTION STAAAAAATE")
                if (self.dummy_inter >= 0 and self.dummy_inter < 24):  # 26 leds, 20 daylight
                    self.upper = 1
                    self.pid = 1
                    self.dummy_inter += 1
                    if (self.direction == 3 or self.direction == 0):
                        if (self.dummy_inter >= 5):
                            self.dummy_inter = 100
                    left = 35
                    right = 35
                    # print("at int", right, left)
                    dir = 0
                    self.next_state = INTERSECTION_STATE
                else:
                    print("DIRECTION AT INT", self.direction)
                    if (self.direction == 0 or self.direction == 3):
                        self.next_state = DRIVE_STATE
                        self.dummy_inter = 0
                    else:
                        self.next_state = ROTATE_STATE
                        self.start = 0
                        self.rotate_ret = DRIVE_STATE
                        dir = self.direction
                        print("rotating ", dir)
                        self.dummy_inter = 0

                    self.upper = 1
                    self.reply = -1

            elif (self.current_state == PARK_STATE):
                print("PAAAAAARK")
                if (self.blind):
                    if (self.parking <= 2):
                        self.start = 0
                        self.next_state = ROTATE_STATE
                        self.rotate_ret = PARK_STATE
                        self.direction = 1
                        self.parking += 1
                    else:
                        self.next_state = IDLE_STATE
                        self.dispatch = 0
                else:
                    print("pid inside park")
                    self.pid = 1
                    right = 40
                    left = 40
                    if (self.motor >= 0):
                        left += self.motor
                    else:
                        right -= self.motor
                    dir = 0
                    if (left < 0):
                        left = 0
                    if (right < 0):
                        right = 0
                    if (self.parking > 2):
                        self.next_state = IDLE_STATE
                        self.dispatch = 0
                    else:
                        self.next_state = PARK_STATE


            else:  # ROTATE
                if (self.start == 0):
                    self.angle = self.yaw
                    self.start = 1
                    self.next_state = ROTATE_STATE

                else:
                    # print("yaw=%f, current=%f" % (self.yaw, self.angle))
                    if (np.cos(np.radians(self.angle - self.yaw)) <= 0.1):
                        self.next_state = self.rotate_ret
                        # if(self.rotate_ret==INTERSECTION_STATE):
                        #     self.direction = -1
                        # print("return to state %d from rot"%(self.rotate_ret))
                        self.upper = 1
                    else:
                        self.next_state = ROTATE_STATE
                        left = 40
                        right = 40
                        dir = self.direction
                        if (dir == 3):
                            dir = 0
                        if (dir >= INSTR_BACK_STRAIGHT):
                            dir = 1
                        # print("rotating ", self.direction)

            if (right):
                self.prev_right = right
            if (left):
                self.prev_left = left
            v3 = Vector3()
            v = Vector3Stamped()
            head = Header()
            head.stamp = rospy.Time.now()
            v3.x = left
            v3.y = right
            v3.z = dir
            v.header = head
            v.vector = v3
            pid_bool = Bool()
            pid_bool = self.pid
            cam_bool = Bool()
            cam_bool = self.upper
            req_bool = Bool()
            req_bool = self.request
            try:
                self.wheels_publish.publish(v)
                self.cam_pub.publish(cam_bool)
                if (self.request == 1):
                    print("requesting from jetson")
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
