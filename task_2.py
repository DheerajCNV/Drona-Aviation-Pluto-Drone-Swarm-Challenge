#!/usr/bin/env python3

# Importing the required libraries

import time
import cv2 as cv
from cv2 import aruco
import numpy as np
import traceback
import sys
import struct
import socket
# import pandas as pd

# df = pd.DataFrame(columns=['x','y','z','roll','pitch','throttle','kp','kd','ki','setpoint','duration'])
# df_index=0

class Pluto():
	
	def __init__(self):
		
		TCP_IP = '192.168.4.1'
		TCP_PORT = 23
		self.BUFFER_SIZE = 1024

		try:
			self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			print ("Socket successfully created")
		except socket.error as err:
			print ("socket creation failed with error %s" %(err))

		self.s.connect((TCP_IP, TCP_PORT))

		
		self.calib_data_path = "camera_calibration\calib_data\MultiMatrix.npz"
		self.calib_data = np.load(self.calib_data_path)
		print(self.calib_data.files)
		self.cam_mat = self.calib_data["camMatrix"]
		print(self.cam_mat)
		self.dist_coef = self.calib_data["distCoef"]
		self.r_vectors = self.calib_data["rVector"]
		self.t_vectors = self.calib_data["tVector"]
		self.MARKER_SIZE = 7  # centimeters
		self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
		self.param_markers = aruco.DetectorParameters()
		self.detector = aruco.ArucoDetector(self.marker_dict, self.param_markers)

		self.cap = cv.VideoCapture(0,cv.CAP_DSHOW)

		fps = 60
		self.cap.set(cv.CAP_PROP_FPS, fps)

		# Set the resolution
		width = 1280
		height = 720
		self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
		self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
  
		# self.newCameraMatrix, self.roi = cv.getOptimalNewCameraMatrix(self.cam_mat, self.dist_coef, (width,height), 1, (width,height))
		# f_x, f_y, f_w, f_h= self.roi
		# # frame=frame[y:y+h,x:x+w]

		self.size = (width, height)
		print(self.size)
		# self.result = cv.VideoWriter("video_rec\drone_flight_39.avi", cv.VideoWriter_fourcc(*'MJPG'), fps, self.size)

		# [x,y,z]
		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,245]

		self.rcRoll = 1500
		self.rcPitch = 1500
		self.rcYaw = 1500
		self.rcThrottle = 1500
		self.rcAUX1 = 1500 #headfree 1300-1700
		self.rcAUX2 = 1500 #developer mode
		self.rcAUX3 = 1500 #alt hold - 1300 to 1700
		self.rcAUX4 = 1500 #arm - 1300 to 1700


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [3.9,3.9,7.5]
		self.Ki = [0,0,0.0]
		self.Kd = [31,31,30]

		self.throttle_error=0
		self.throttle_prev_error=0
		self.throttle_sum_error=0
		self.min_throttle=900
		self.max_throttle=2100


		self.roll_error=0
		self.roll_prev_error=0
		self.roll_sum_error=0
		self.min_roll=900
		self.max_roll=2100


		self.pitch_error=0
		self.pitch_prev_error=0
		self.pitch_sum_error=0
		self.min_pitch=900
		self.max_pitch=2100

		self.x=0
		self.y=0
		self.z=350
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	def screen(self):
		_ , frame = self.cap.read()
		cv.imshow("frame",frame)

	# Disarming condition of the drone
	def disarm(self):
		self.rcThrottle=1000
		self.rcYaw=1000
		self.rcRoll=1500
		self.rcPitch=1500
		self.rcAUX4 = 1100
		payload_length = 16  	#1 byte length, denotes no. of bytes in payload
		message_type = 200 		#1 byte length
		payload = [self.rcRoll,self.rcPitch,self.rcYaw,self.rcThrottle,self.rcAUX1,self.rcAUX2,self.rcAUX3,self.rcAUX4] # each element 2 bytes
		packed_data = struct.pack('<8h', *payload) #packing in little_endian
		self.command_message(payload_length,message_type,packed_data)
		time.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):
		self.disarm()
		self.rcRoll = 1500
		self.rcYaw = 2000
		self.rcPitch = 1500
		self.rcThrottle = 1000
		self.rcAUX4 = 1500

		payload_length = 16 #1 byte length, denotes no. of bytes in payload
		message_type = 200 #1 byte length
		payload = [self.rcRoll,self.rcPitch,self.rcYaw,self.rcThrottle,self.rcAUX1,self.rcAUX2,self.rcAUX3,self.rcAUX4] # each element 2 bytes
		packed_data = struct.pack('<8h', *payload) #packing in little_endian
		self.command_message(payload_length,message_type,packed_data)

		time.sleep(1)


	#For setting values
	def command_message_in(payload_length,message_type,payload):
		header=[36,77] #1 byte length
		direction=60 #60 for in, 62 for out
				
		checksum = message_type ^ payload_length
		for d in payload:
			checksum = checksum ^ d
		message = bytearray(header) + bytearray([direction, payload_length, message_type]) + bytearray(payload) + bytearray([checksum])
		# print("payload:",payload)
		# print("sent data:",message)
		return message
		# data = self.s.recv(self.BUFFER_SIZE)
		# print("received data:", data)
		# print("decoded:",list(data))
		# print()


	# MSP_SET_COMMAND
	def msp_set_cmd(self,val):
		payload_length=2
		message_type=217
		payload=[val]
		packed_data=struct.pack('<h',*payload)
		msg = self.command_message_in(payload_length,message_type,packed_data)
		return msg
		

	# MSP_SET_RAW_RC
	def msp_set_raw(self,rcRoll = 1500, rcPitch = 1500, rcYaw = 1500, rcThrottle = 1500, rcAUX1 = 1500, rcAUX2 = 1500, rcAUX3 = 1800, rcAUX4 = 1500):
		payload_length = 16 #1 byte length, denotes no. of bytes in payload
		message_type = 200 #1 byte length
		payload = [rcRoll, rcPitch, rcYaw, rcThrottle, rcAUX1, rcAUX2, rcAUX3, rcAUX4] # each element 2 bytes
		packed_data = struct.pack('<8h', *payload) #packing in little_endian
		msg = self.command_message_in(payload_length,message_type,packed_data)
		return msg


	def request_message(self, message_type):
		header=[36,77] #1 byte length
		direction=60 #60 for in, 62 for out
		payload_length=0
		checksum=message_type
		message = bytearray(header) + bytearray([direction, payload_length, message_type]) + bytearray([checksum])
		# print("sent data:",message)
		self.s.send(message)
		data = self.s.recv(self.BUFFER_SIZE)
		self.reqmess=list(data)
		# print("received data:", data)
		# print("decoded:",reqmess)
		# print()


	def takeoff(self):
		#MSP_SET_COMMAND-takeoff
		self.msp_set_cmd(1)

	def land(self):
		#MSP_SET_COMMAND-land
		self.msp_set_cmd(2)


	def pid(self):
     
		# global df
		# global df_index

		ret, frame = self.cap.read()
		# if not ret:
		# 	break
		h,  w = frame.shape[:2]
		newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(self.cam_mat, self.dist_coef, (w,h), 1, (w,h))
		frame = cv.undistort(frame, self.cam_mat, self.dist_coef, None, newCameraMatrix)
		# x,y,w,h=roi
		# frame=frame[y:y+h,x:x+w]
		gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
		marker_corners, marker_IDs, reject = self.detector.detectMarkers(
			gray_frame
		)
		self.result.write(frame)
		if (marker_corners and not((self.x>135 or self.x<-135) or (self.y>75 or self.y<-75)) ):
			rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
				marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
			)

			# print(tVec)
			total_markers = range(0, marker_IDs.size)
			for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):  # we are using only 1 marker
				self.x=tVec[i][0][0] - 26.6
				self.y=tVec[i][0][1] + 34.8
				self.z= round(tVec[i][0][2],2)          #//10+185                                      #*(100/240)                      #*(5/26)

				cv.polylines(
					frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
				)
				corners = corners.reshape(4, 2)
				corners = corners.astype(int)
				

			self.throttle_error = -(self.setpoint[2] - self.z )     #(300-round(self.z,2))

			self.rcThrottle = int(1500 + self.throttle_error*self.Kp[2] + (self.throttle_error - self.throttle_prev_error)*self.Kd[2] + self.throttle_sum_error*self.Ki[2])

			if self.rcThrottle > self.max_throttle:
				self.rcThrottle = self.max_throttle
			if self.rcThrottle < self.min_throttle:
				self.rcThrottle = self.min_throttle

			self.throttle_prev_error = self.throttle_error
			self.throttle_sum_error = (self.throttle_sum_error) + self.throttle_error

			
			print("throttle error:", self.throttle_error,end=" ")

			# self.rcThrottle=1500


			self.roll_error = -(self.setpoint[0] - self.x)  		#drone head in negative y-dir

			self.rcRoll = int(1500 + self.roll_error*self.Kp[0] + (self.roll_error - self.roll_prev_error)*self.Kd[0] + self.roll_sum_error*self.Ki[0])

			if self.rcRoll > self.max_roll:
				self.rcRoll = self.max_roll
			if self.rcRoll < self.min_roll:
				self.rcRoll = self.min_roll

			self.roll_prev_error = self.roll_error
			self.roll_sum_error = self.roll_sum_error + self.roll_error
			print("roll error:", self.roll_error,end=" ")



			self.pitch_error = (self.setpoint[1] - self.y) 			#drone head in negative y-dir

			self.rcPitch = int(1500 + self.pitch_error*self.Kp[1] + (self.pitch_error - self.pitch_prev_error)*self.Kd[1] + self.pitch_sum_error*self.Ki[1])

			if self.rcPitch > self.max_pitch:
				self.rcPitch = self.max_pitch
			if self.rcPitch < self.min_pitch:
				self.rcPitch = self.min_pitch

			self.pitch_prev_error = self.pitch_error
			self.pitch_sum_error = self.pitch_sum_error + self.pitch_error
			print("pitch error:", self.pitch_error)

			#MSP_SET_RAW
			self.msp_set_raw(self.rcRoll,self.rcPitch,self.rcYaw,self.rcThrottle,self.rcAUX1,self.rcAUX2,self.rcAUX3,self.rcAUX4)

			print("roll:", self.rcRoll,end=" ")
			print("pitch:", self.rcPitch,end=" ")
			print("throttle:",self.rcThrottle)
			# print("Drone_height_internal:",self.drone_height)

			print("x:",self.x ,end=" ")
			print("y:",self.y,end=" ")
			print("z:",self.z,2)

			# df.loc[df_index, ['x']] = self.x
			# df.loc[df_index, ['y']] = self.y
			# df.loc[df_index, ['z']] = self.z                          ### for saving the values in a csv file
			# df.loc[df_index, ['roll']] = self.rcRoll
			# df.loc[df_index, ['pitch']] = self.rcPitch
			# df.loc[df_index, ['throttle']] = self.rcThrottle

		else:
			print()
			
			if (self.x>135 or self.x<-135) or (self.y>75 or self.y<-75):
				print("out of box")
				self.land()
				print("Landing")
				time.sleep(0.5)
				return(0)
				

			else:
				print("Aruco NO")
				self.msp_set_raw(self.rcRoll,self.rcPitch,self.rcYaw,self.rcThrottle,self.rcAUX1,self.rcAUX2,self.rcAUX3,self.rcAUX4)
				print("roll:", self.rcRoll,end=" ")
				print("pitch:", self.rcPitch,end=" ")
				print("throttle:",self.rcThrottle)

			# df.loc[df_index, ['roll']] = self.rcRoll
			# df.loc[df_index, ['pitch']] = self.rcPitch
			# df.loc[df_index, ['throttle']] = self.rcThrottle

		# df_index+=1
		return(1)


	def hover(self, setpoint, hover_time):
     
		self.setpoint = setpoint
		
		self.Kp = [3.9,3.9,15]
		# self.rcThrottle = 1300
		self.rcAUX3 = 1500
		print("Hovering")
			
		cur_time = time.time()
	
		while (time.time() < cur_time + hover_time) or (self.throttle_error>=10 or self.throttle_error<=-10 or self.roll_error>=5 or self.roll_error<=-5 or self.pitch_error>=5 or self.pitch_error<=-5): 
			if(not self.pid()):
				return (0)
			time.sleep(0.004)

		return (1)
			
 
 
	def pitching(self, next_setpoint):
		
		self.rcAUX3 = 1500
		self.setpoint = next_setpoint
     
		self.Kp = [3.9,2,7.5]
		# self.rcThrottle = 1500
		print("Pitching forward")
  
		while (self.throttle_error>=10 or self.throttle_error<=-10 or self.roll_error>=5 or self.roll_error<=-5 or self.pitch_error>=5 or self.pitch_error<=-5): 
			if(not self.pid()):
				return (0)
			time.sleep(0.004)

		return(1)
		

	# def pitch_backward(self, next_setpoint):
		
	# 	self.rcAUX3 = 1500
	# 	self.setpoint = next_setpoint
		
	# 	self.Kp = [3.9,2,7.5]
	# 	# self.rcThrottle = 1500
	# 	print("Pitching backward")

	# 	while (self.throttle_error>=2 or self.throttle_error<=-2 or self.roll_error>=2 or self.roll_error<=-2 or self.pitch_error>=2 or self.pitch_error<=-2):
	# 		if(not self.pid()):
	# 			break
	# 		time.sleep(0.004)
			

	def rolling(self, next_setpoint):
		
		self.rcAUX3 = 1500
		self.setpoint = next_setpoint
			
		self.Kp = [2,3.9,7.5]
		self.rcThrottle = 1500
		print("rolling right")
				
		while (self.throttle_error>=10 or self.throttle_error<=-10 or self.roll_error>=5 or self.roll_error<=-5 or self.pitch_error>=5 or self.pitch_error<=-5):
			if(not self.pid()):
				return (0)
			time.sleep(0.004)

		return (1)
				

	# def roll_left(self, next_setpoint):
			
	# 	self.rcAUX3 = 1500
	# 	self.setpoint = next_setpoint
					
	# 	self.Kp = [2,3.9,7.5]
	# 	self.rcThrottle = 1500
	# 	print("rolling left")
						
	# 	while (self.throttle_error>=2 or self.throttle_error<=-2 or self.roll_error>=2 or self.roll_error<=-2 or self.pitch_error>=2 or self.pitch_error<=-2):
	# 		if(not self.pid()):
	# 			break
	# 		time.sleep(0.004)
						
	# 	return(1)


	def rectangle_path(self,path):
		p1=self.hover(path[0],1)
		if p1==0:
			return
		p2=self.pitching(path[1])
		if p2==0:
			return
		p3=self.rolling(path[2])
		if p3==0:
			return
		p4=self.pitching(path[3])
		if p4==0:
			return
		p=self.hover(path[3],0.5)
		


		
#############################################################################################

if __name__ == '__main__':

	e_drone = Pluto()
	start_time=time.time()

	try:
		
		e_drone.takeoff()
		time.sleep(1)
		print("1")

		#Hover
		e_drone.hover([0,0,270],3)
		
		path = [[95, 45, 270], [95, -45, 270], [-95, -45, 270], [-95, 45, 270]]
		e_drone.rectangle_path(path)

		e_drone.land()
		print("Landing after x sec")

		end_time=time.time()
		duration=end_time-start_time

  
		# e_drone.cap.release()
		# e_drone.result.release()
		# print("The Video was saved successfully")

	except:
     
		e_drone.land()
		print("Landing from except")
		time.sleep(0.1)

		end_time=time.time()
		duration=end_time-start_time

		# e_drone.cap.release()
		# e_drone.result.release()
		# print("The Video was saved successfully")
		
		# traceback.print_exc(file=sys.stdout)
		# sys.exit()