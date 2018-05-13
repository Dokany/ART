#!/usr/bin/env python


# roslib.load_manifest('my_package')
import sys
import roslib
import rospy
import networkx as nx
import timeit
from geometry_msgs.msg import *
from std_msgs.msg import *
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
INSTR_BACK_STRAIGHT = 7
INSTR_BACK_LEFT = 8
INSTR_BACK_RIGHT = 9

PID_RATIO = 80.0 / 1000.0

POLICE_POS = 0
AMBULANCE_POS = '9'
FIREFIGHTER_POS = '5'


class master_control:

	def __init__(self):
		rospy.init_node('MASTER_CONTROL', anonymous=True)
		self.wheels_publish = rospy.Publisher("jetson/wheels_speed", Vector3Stamped, queue_size=10)
		self.pid_publish = rospy.Publisher("jetson/pid_start", Bool, queue_size=10)
		self.sd_pub_ff = rospy.Publisher("reply_ff", Float32, queue_size=10)
		self.sd_pub_amb = rospy.Publisher("reply_amb", Float32, queue_size=10)
		self.sd_pub_pol = rospy.Publisher("reply_pol", Float32, queue_size=10)
		self.disp_pub_ff = rospy.Publisher("dispatch_ff", Bool, queue_size=10)
		self.disp_pub_amb = rospy.Publisher("dispatch_amb", Bool, queue_size=10)
		self.disp_pub_pol = rospy.Publisher("dispatch_pol", Bool, queue_size=10)

		self.bar_sub = rospy.Subscriber("barcode", String, self.callback0)
		self.cam_sub = rospy.Subscriber("jetson/blind", Bool, self.callback1)
		self.req_sub_ff = rospy.Subscriber("pi_request_ff", Bool, self.callback2)
		self.req_sub_amb = rospy.Subscriber("pi_request_amb", Bool, self.callback5)
		self.req_sub_pol = rospy.Subscriber("pi_request_pol", Bool, self.callback6)
		self.ml_sub = rospy.Subscriber("/darknet_ros/class_detected", Vector3, self.callback3)
		self.lw_sub = rospy.Subscriber("jetson/control_effort", Float64, self.callback4)
		self.disp_ff = self.disp_amb = self.disp_pol = 0
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
		self.motor = 0
		self.direction = 1
		self.blind = 0
		self.barcode = ""
		self.ff_current_path = self.amb_current_path = self.pol_current_path= []
		self.ff_safe = self.amb_safe = self.pol_safe = 1
		self.ff_request = self.amb_request = self.pol_request = 0
		self.ff_reply = self.amb_reply = self.pol_reply = -1
		self.p_direction = 0
		self.class1 = self.class2 = self.class3 = -1
		self.ff_target = self.pol_target = self.amb_target = -1
		self.ff_index = self.amb_index = self.pol_index = -1


	def callback0(self, data):  # barcode
		self.barcode = data.data

	def callback1(self, data):  # blind
		self.blind = data.data
		if (self.blind):
			print ("BLIIIINNNDDD")

	def checkSafety(self):
		return self.amb_safe and self.pol_safe and self.ff_safe

	def checkRequests(self):
		return self.amb_request or self.pol_request or self.ff_request

	def checkDispatch(self):
		return self.disp_pol or self.disp_amb or self.disp_ff

	def getReplies(self):
		#print("trying to get replies")
		self.ff_reply = self.amb_reply = self.pol_reply = -1
		if(self.ff_request and self.ff_target==-1):
			self.ff_request = 0
			self.ff_reply = INSTR_PARK
		if(self.amb_request and self.amb_target==-1):
			self.amb_request = 0
			self.amb_reply = INSTR_PARK
		if(self.pol_request and self.pol_target==-1):
			self.pol_request = 0
			self.pol_reply = INSTR_PARK
		#print("check requests", self.checkRequests())
		#print("check safety", self.checkSafety())
		if (self.checkSafety() == False):
			#print(self.amb_safe,self.pol_safe,self.ff_safe)
			return False
		#print("all safe")
		if(self.checkRequests() == False):
			return False
		#print("after request")
		if(self.dagu_pos["ambulance"]==None and self.dagu_pos["firefighter"]==None and self.dagu_pos["police"]==None):
			return False

		if(self.checkDispatch()):
			if(self.ff_request):
				self.disp_ff=0
			if(self.pol_request):
				self.disp_pol = 0
			if(self.amb_request):
				self.disp_amb = 0
			return False
		#print("there is a request")
		print("computing paths")
		start_time = timeit.default_timer()
		if ((self.ff_request and len(self.ff_current_path) == 0) or (self.pol_request and len(self.pol_current_path) == 0) or (self.amb_request and len(self.amb_current_path) == 0)):
			#or (self.ff_index+2>=len(self.ff_current_path)) or (self.amb_index+2>=len(self.amb_current_path)) or (self.ff_index+2>=len(self.ff_current_path))):

			#print("ffffirst")
			amb_path = []
			pol_path = []
			ff_path = []
			if (self.dagu_pos["ambulance"] != None and self.dagu_pos["firefighter"] != None and self.dagu_pos["police"] != None):
				amb_path = []
				pol_path = []
				ff_path = []

				if (self.dagu_pos["ambulance"] != None):
					for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["ambulance"], target=self.amb_target):
						amb_path.append(path)

				if (self.dagu_pos["firefighter"] != None):
					for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["firefighter"],target=self.ff_target):
						ff_path.append(path)

				if (self.dagu_pos["police"] != None):
					for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["police"], target=self.pol_target):
						pol_path.append(path)
				self.ff_current_path, self.amb_current_path, self.pol_current_path = self.get_combined_shortest_path_three(ff_path, amb_path, pol_path, 6)

			elif (self.dagu_pos["ambulance"] != None and self.dagu_pos["firefighter"] != None):
				for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["ambulance"], target=self.amb_target):
					amb_path.append(path)
				for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["firefighter"], target=self.ff_target):
					ff_path.append(path)
				self.amb_current_path, self.ff_current_path = self.get_combined_shortest_path_two(amb_path, ff_path, 6)

			elif (self.dagu_pos["ambulance"] != None and self.dagu_pos["police"] != None):
				for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["ambulance"], target=self.amb_target):
					amb_path.append(path)
				for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["police"], target=self.pol_target):
					pol_path.append(path)
				self.amb_current_path, self.pol_current_path = self.get_combined_shortest_path_two(amb_path, pol_path,6)

			elif (self.dagu_pos["police"] != None and self.dagu_pos["firefighter"] != None):
				for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["firefighter"], target=self.ff_target):
					ff_path.append(path)
				for path in nx.all_simple_paths(self.FG, source=self.dagu_pos["police"], target=self.pol_target):
					pol_path.append(path)
				self.pol_current_path, self.ff_current_path = self.get_combined_shortest_path_two(pol_path, ff_path, 6)

			else:
				if (self.dagu_pos["police"] != None):
					self.pol_current_path = nx.shortest_path(self.FG, source=self.dagu_pos["police"],target=self.pol_target)
				elif (self.dagu_pos["ambulance"] != None):
					#print("CORRRREEEEEECTT IFFFFF")
					self.amb_current_path = nx.shortest_path(self.FG, source=self.dagu_pos["ambulance"],target=self.amb_target)
				else:
					self.ff_current_path = nx.shortest_path(self.FG, source=self.dagu_pos["firefighter"],target=self.ff_target)
			self.ff_index = self.amb_index = self.pol_index = 0
		#print("in replies")
		#print("amb index", self.amb_index)

		print("amb pos ", self.dagu_pos["ambulance"])
		print("pol pos ", self.dagu_pos["police"])
		print("ff pos ", self.dagu_pos["firefighter"])
		elapsed = timeit.default_timer() - start_time
		print("Graph elapsed time", elapsed)
		self.ff_index += 1
		self.amb_index += 1
		self.pol_index += 1

		if(self.ff_request == 0):
			self.ff_index-=1
		else:
			self.ff_index += 1
			if (self.ff_index >= len(self.ff_current_path)):
				if (self.dagu_pos["firefighter"] != FIREFIGHTER_POS):
					self.ff_reply = INSTR_DEST_LEFT
					self.ff_target = FIREFIGHTER_POS

					self.ff_reply = 0
				else:
					self.ff_reply = INSTR_PARK
					self.ff_target = -1
				self.ff_current_path = []
				
			elif (self.ff_current_path[self.ff_index-1] == self.ff_current_path[self.ff_index - 2]):  # waiting
				self.ff_reply = 0
				self.ff_request = 1
				
			else:
				next_pos = self.ff_current_path[self.ff_index]
				intersection = self.ff_current_path[self.ff_index - 1]
				previous = self.ff_current_path[self.ff_index - 2]
				dir = self.get_direction(previous, intersection, next_pos, self.dagu_prev_pos["firefighter"])

				self.dagu_pos["firefighter"] = next_pos
				self.dagu_prev_pos["firefighter"] = previous

				if (dir == "right"):
					self.ff_reply = INSTR_RIGHT
				elif (dir == "left"):
					self.ff_reply = INSTR_LEFT
				elif (dir == "straight"):
					self.ff_reply = INSTR_STRAIGHT
				elif(dir=="back_straight"):
					self.ff_reply = INSTR_BACK_STRAIGHT
				elif(dir=="back_right"):
					self.ff_reply = INSTR_BACK_RIGHT
				elif(dir=="back_left"):
					self.ff_reply = INSTR_BACK_LEFT
				else:
					self.ff_reply = 0

		if(self.amb_request == 0):
			self.amb_index-=1

		else:
			self.amb_index += 1
			if (self.amb_index >= len(self.amb_current_path)):
				if (self.dagu_pos["ambulance"] != AMBULANCE_POS):

					self.amb_reply = 0
					self.amb_target = AMBULANCE_POS

				else:
					self.amb_reply = INSTR_PARK
					self.amb_target = -1
				self.amb_current_path = []
				
			elif (self.amb_current_path[self.amb_index-1] == self.amb_current_path[self.amb_index - 2]):  # waiting
				self.amb_reply = 0
				self.amb_request = 1
				
			else:
				next_pos = self.amb_current_path[self.amb_index]
				intersection = self.amb_current_path[self.amb_index - 1]
				previous = self.amb_current_path[self.amb_index - 2]

				## ADD IT in the other two blocks
				### Add "back" to get_direction
				dir = self.get_direction(previous, intersection, next_pos, self.dagu_prev_pos["ambulance"] )
				self.dagu_pos["ambulance"] = next_pos
				self.dagu_prev_pos["ambulance"] = previous

				if (dir == "right"):
					self.amb_reply = INSTR_RIGHT
				elif (dir == "left"):
					self.amb_reply = INSTR_LEFT
				elif (dir == "straight"):
					self.amb_reply = INSTR_STRAIGHT
				elif(dir=="back_straight"):
					self.amb_reply = INSTR_BACK_STRAIGHT
				elif(dir=="back_right"):
					self.amb_reply = INSTR_BACK_RIGHT
				elif(dir=="back_left"):
					self.amb_reply = INSTR_BACK_LEFT
				else:
					self.amb_reply = 0

		if(self.pol_request == 0):
			self.pol_index-=1
		
		else:
			self.pol_index += 1
			if (self.pol_index >= len(self.pol_current_path)):
				if (self.dagu_pos["police"] != POLICE_POS):
					self.pol_reply = INSTR_DEST_LEFT
					self.pol_target = POLICE_POS

					self.pol_reply = 0
				else:
					self.pol_reply = INSTR_PARK
					self.pol_target = -1
				self.pol_current_path = []
				
			elif (self.pol_current_path[self.pol_index-1] == self.pol_current_path[self.pol_index - 2]):  # waiting
				self.pol_reply = 0
				self.pol_request = 1
				
			else:
				next_pos = self.pol_current_path[self.pol_index]
				intersection = self.pol_current_path[self.pol_index - 1]
				previous = self.pol_current_path[self.pol_index - 2]

				print("prev , next ", next_pos, self.dagu_prev_pos["police"])
				dir = self.get_direction(previous, intersection, next_pos, self.dagu_prev_pos["police"] )
				print(dir)
				self.dagu_pos["police"] = next_pos
				self.dagu_prev_pos["police"] = previous
				if (dir == "right"):
					self.pol_reply = INSTR_RIGHT
				elif (dir == "left"):
					self.pol_reply = INSTR_LEFT
				elif (dir == "straight"):
					self.pol_reply = INSTR_STRAIGHT
				elif(dir=="back_straight"):
					self.pol_reply = INSTR_BACK_STRAIGHT
				elif(dir=="back_right"):
					self.pol_reply = INSTR_BACK_RIGHT
				elif(dir=="back_left"):
					self.pol_reply = INSTR_BACK_LEFT
				else:
					self.pol_reply = 0
		if(len(self.amb_current_path)>0):
			print("amb path: ", self.amb_current_path)
		if(len(self.ff_current_path)>0):
			print("ff path: ", self.ff_current_path)
		if(len(self.pol_current_path)>0):
			print("pol path: ", self.pol_current_path)
		print("amb reply", self.amb_reply)
		print("pol reply", self.pol_reply)
		print("ff reply", self.ff_reply)
		self.ff_request = self.amb_request = self.pol_request = 0

		if(self.pol_reply>0 and self.pol_reply !=INSTR_PARK):
			self.pol_safe = 0
		else:
			if(self.pol_reply==0):
				self.pol_request =1
			self.pol_safe = 1
		if (self.ff_reply >0 and self.ff_reply !=INSTR_PARK):
			self.ff_safe = 0
		else:
			if(self.ff_reply==0):
				self.ff_request =1
			self.ff_safe = 1
		if (self.amb_reply >0 and self.amb_reply !=INSTR_PARK):
			self.amb_safe = 0
		else:
			if(self.amb_reply==0):
				self.amb_request =1
			self.amb_safe = 1
		return True

	def callback2(self, data):  # ff_request
		if(data.data == 1):
			#print("got ff request")
			self.ff_safe = 1
			self.ff_request = 1
			self.disp_ff = 0

	def callback5(self, data):  # amb_request
		if(data.data == 1):
			print("got amb request")
			self.amb_safe = 1
			self.amb_request = 1
			self.disp_amb = 0

	def callback6(self, data):  # pol_request
		if(data.data == 1):
			print("got pol request")
			self.pol_safe = 1
			self.pol_request = 1
			self.disp_pol = 0

	def callback3(self, data):  # ml
		self.class1 = data.x
		self.class2 = data.y
		self.class3 = data.z

	def callback4(self, data):  # motor
		self.motor = int((data.data) * PID_RATIO)

	##### Graph handling
	def get_direction(self, previous, intersection, nxt, history):
		#print("in get_direction",previous, intersection,nxt)
		list_temp = []
		index = -1
		#print("self.FG INT",self.FG.node[intersection])
		if 'list' in self.FG.node[intersection]:
			list_temp = self.FG.node[intersection]['list']
			#print("list_fff", list_temp)
			list_temp = list_temp.split()
		#print("list temp", list_temp)
		index = list_temp.index(previous)

		if(history == nxt):
			if ((index + 1) % 4 == list_temp.index(nxt)):
				return "back_left"

			if ((index + 2) % 4 == list_temp.index(nxt)):
				return "back_straight"

			if ((index + 3) % 4 == list_temp.index(nxt)):
				return "back_right"

		if ((index + 1) % 4 == list_temp.index(nxt)):
			return "left"

		if ((index + 2) % 4 == list_temp.index(nxt)):
			return "straight"

		if ((index + 3) % 4 == list_temp.index(nxt)):
			return "right"

		return "unknown"

	def generate_n_shift(self, x, arr, duplicate_index):

		temp = arr[:]
		for i in range(0, x):
			temp.insert(duplicate_index, temp[duplicate_index])
		return temp

	def generate_all_possible(self, dagu_list, shifts):
		all_paths = []
		for path in dagu_list:
			all_paths.append(path)
			# for i in np.arange(0,len(path),2):
			for delay in np.arange(2, shifts, 2):
				all_paths.append(self.generate_n_shift(delay, path, 0))
		return all_paths

	def check_collisions(self, path1, path2):
		collision = 0
		i = 0
		mn = min(len(path1), len(path2))
		while (collision == 0 and (i < mn)):
			if (path1[i] == path2[i]):
				collision = 1
			i += 1
		return collision

	def get_combined_shortest_path_two(self, dagu1_list, dagu2_list, shifts):
		min_cost = 1e9
		minimum_path_1 = []
		minimum_path_2 = []

		all_paths_1 = self.generate_all_possible(dagu1_list, shifts + 1)
		all_paths_2 = self.generate_all_possible(dagu2_list, shifts + 1)

		for path1 in all_paths_1:
			for path2 in all_paths_2:
				cost = len(path1) + len(path2) - 2
				if (cost > min_cost or self.check_collisions(path1, path2)):
					continue
				min_cost = cost
				minimum_path_1 = path1
				minimum_path_2 = path2
		return minimum_path_1, minimum_path_2

	def get_combined_shortest_path_three(self, dagu1_list, dagu2_list, dagu3_list, shifts):
		min_cost = 1e9
		minimum_path_1 = []
		minimum_path_2 = []
		minimum_path_3 = []

		all_paths_1 = self.generate_all_possible(dagu1_list, shifts + 1)
		all_paths_2 = self.generate_all_possible(dagu2_list, shifts + 1)
		all_paths_3 = self.generate_all_possible(dagu3_list, shifts + 1)

		for path1 in all_paths_1:
			for path2 in all_paths_2:
				if (self.check_collisions(path1, path2)):
					continue
				for path3 in all_paths_3:
					cost = len(path1) + len(path2) + len(path3) - 3
					if (cost > min_cost):
						continue
					if (self.check_collisions(path3, path2) or self.check_collisions(path1, path3)):
						continue
					min_cost = cost
					minimum_path_1 = path1
					minimum_path_2 = path2
					minimum_path_3 = path3
		return minimum_path_1, minimum_path_2, minimum_path_3

	def generate_graph(self):
		# Creating The Graph
		self.FG = nx.Graph()
		self.FG.add_edges_from([

			('1', 'i1'),
			('1', 'i2'),
			('2', 'i2'),
			('2', 'i3'),
			('3', 'i3'),
			('3', 'i4'),

			('4', 'i4'),
			('4', 'i5'),
			('5', 'i3'),
			('5', 'i6'),
			('6', 'i2'),
			('6', 'i7'),

			('7', 'i1'),
			('7', 'i8'),
			('8', 'i8'),
			('8', 'i7'),
			('9', 'i7'),
			('9', 'i6'),

			('10', 'i6'),
			('10', 'i5'),
			('11', 'i5'),
			('11', 'i11'),
			('12', 'i6'),
			('12', 'i10'),

			('13', 'i7'),
			('13', 'i9'),
			('14', 'i9'),
			('14', 'i10'),
			('15', 'i10'),
			('15', 'i11'),

			('16', 'i11'),
			('16', 'i12'),
			('17', 'i10'),
			('17', 'i13'),
			('18', 'i9'),
			('18', 'i14'),
			('19', 'i13'),
			('19', 'i14'),
			('20', 'i12'),
			('20', 'i13'),

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
		self.FG.node['i14']['list'] = '18 -1 -1 19'

		self.dagu_pos = {'firefighter': None, 'ambulance': None, 'police': None}
		self.dagu_prev_pos = {'firefighter': None, 'ambulance': None, 'police': None}

		self.amb_current_path = []
		self.pol_current_path = []
		self.ff_current_path = []

	def work(self):
		rate = rospy.Rate(30)
		self.generate_graph()
		demo = 0
		self.prev_barcode = ""
		self.disp_amb = 1
		self.disp_pol = 0
		self.disp_ff = 0
		self.amb_target ='8'
		self.dagu_pos["ambulance"] = AMBULANCE_POS
		self.pol_target ='9'
		self.ff_target = '12'
		#self.dagu_pos["police"] = POLICE_POS
		print("Ambulance")
		while not rospy.is_shutdown():
			self.upper = 0
			#print (self.current_state)
			self.current_state = self.next_state
			if (self.next_state != self.current_state):
				print (self.next_state)
			left = 0
			#self.dispatch = 0
			right = 0
			dir = 0
			self.pid = 1
			if (self.current_state == IDLE_STATE):
				if (self.blind == 0):
					self.next_state = DRIVE_STATE
				else:
					self.next_state = IDLE_STATE

			elif (self.current_state == LOST_STATE):
				print("LOST")
				self.next_state = LOST_STATE

			elif (self.current_state == DRIVE_STATE):
				if (self.blind):
					self.next_state = IDLE_STATE
				elif (self.barcode == "" or (self.barcode == self.prev_barcode)):
					self.next_state = DRIVE_STATE
					left = 35
					right = 35
					if (self.motor >= 0):
						left += self.motor
					else:
						right -= self.motor
					if (left < 0):
						left = 0
					if (right < 0):
						right = 0
					self.motor = 0
					dir = 0
				else:
					self.next_state = BUILDING_STATE
					print("GOT QRCODE", self.barcode)
					self.prev_barcode = self.barcode
					rospy.sleep(0.5)

			else:  # BUILDING STATE

				rospy.sleep(2)
				if (self.class1 == -1):  # no emergency
					self.next_state = DRIVE_STATE
				else:
					print("dispatching ", self.class1)
					# dispatch
					b = Bool()
					b = True
					if (self.class1 == 0):
						self.disp_ff = 1
						self.ff_target = self.barcode
						self.barcode = ""
						self.dagu_pos['firefighter'] = FIREFIGHTER_POS  # CHANGE
					elif (self.class1 == 1):
						self.disp_amb = 1
						self.amb_target = self.barcode
						self.barcode = ""
						self.dagu_pos['ambulance'] = AMBULANCE_POS  # CHANGE

					else:
						self.disp_pol = 1
						self.pol_target = self.barcode
						self.barcode = ""
						self.dagu_pos['police'] = POLICE_POS  # CHANGE

					self.class1 = self.class2 = self.class3 = -1
					self.next_state = DRIVE_STATE

			v3 = Vector3()
			v = Vector3Stamped()
			self.cnt += 1
			head = Header()
			head.stamp = rospy.Time.now()
			head.seq = self.cnt
			v3.x = left
			v3.y = right
			v3.z = dir
			v.header = head
			v.vector = v3
			pid_bool = Bool()
			pid_bool = self.pid
			ff_disp = Bool()
			ff_disp = self.disp_ff
			if(ff_disp):
				print("disp ff")


			amb_disp = Bool()
			amb_disp = self.disp_amb
			if(amb_disp):
				print("disp amb")

			pol_disp = Bool()
			pol_disp = self.disp_pol
			if(pol_disp):
				print("disp pol")

			try:
				self.disp_pub_ff.publish(ff_disp)
				self.disp_pub_amb.publish(amb_disp)

				self.disp_pub_pol.publish(pol_disp)
				if (self.getReplies()):
					if (self.ff_reply != -1):
						rep = Float32()
						rep = self.ff_reply
						self.sd_pub_ff.publish(rep)

					if (self.amb_reply != -1):
						rep = Float32()
						rep = self.amb_reply
						self.sd_pub_amb.publish(rep)

					if (self.pol_reply != -1):
						rep = Float32()
						rep = self.pol_reply
						self.sd_pub_pol.publish(rep)

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
