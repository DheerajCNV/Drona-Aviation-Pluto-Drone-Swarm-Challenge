# Importing the required libraries

import time
import sys
import struct
import socket

class Pluto():
	
	def __init__(self):
		print("a")

		TCP_IP = '192.168.4.1'
		TCP_PORT = 23
		self.BUFFER_SIZE = 1024

		try:
			self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			print ("Socket successfully created")
		except socket.error as err:
			print ("socket creation failed with error %s" %(err))

		self.s.connect((TCP_IP, TCP_PORT))

    
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


	# MSP_SET_MAX_ALTI
	def msp_set_max_alti(self,val):
		payload_length=2
		message_type=218
		payload=[val]
		packed_data=struct.pack('<h',*payload)
		msg = self.command_message_in(payload_length,message_type,packed_data)
		return msg


#######################
	#Getting values

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

	# MSP_MAX_ALTI
	def msp_max_alti(self,val):
		payload_length=0
		message_type=123
		payload=[0]
		packed_data=struct.pack('<h',*payload)
		msg = self.request_message(message_type)
		return msg


	#MSP_ATTITUDE
	def obs_attitude_val(self,message_type = 108, stp = 2):
			
		# att_msg = msp_attitude()
		# s.send(att_msg)
		# time.sleep(0.2)
		# recv_datab = s.recv(msg_buffer)
		# recv_data = list(recv_datab)
		
		recv_data = self.request_message(message_type)
		msgs = []

		for i in range(0,len(recv_data)):
			if i < len(recv_data)-4 and recv_data[i] == 36 and recv_data[i+1] == 77 and recv_data[i+2] == 62:
				n = recv_data[i+3]
				if i+4 >= len(recv_data):
					break
				msg_type = recv_data[i+4]
				if i+5+n >= len(recv_data):
					break
				check_sum = recv_data[i+5+n]
				message = recv_data[(i+5): (i+5+n)]
				msgs.append([msg_type, "length: " + str(n), message, "check_sum: "+ str(check_sum)])
		# print(msgs)

		req_msg_type = message_type
		req_msgs = []
		for i in msgs:
			if i[0] == req_msg_type:
				req_msgs.append(i[2])
				
		# print(req_msgs)

		# for msg in req_msgs:           (commented as one msg among all the received msgs at that instant is enough)
		rq_msg = req_msgs[0]
		values = []
		step = stp
		
		i = 0

		while i < len(rq_msg):
			
			int_list = rq_msg[i:i+step]
			byte_val = bytes(int_list)
			
			int_val = int.from_bytes(byte_val, "little", signed="True")
			
			# printing int object
			values.append(int_val)
			i = i+step
		
		val = "Roll: " + str(values[0]//10) + ", Pitch: " + str(values[1]//10) + ", Yaw: " + str(values[2])
		print(val)
		
		
	#MSP_ALTITUDE
	def obs_altitude_val(self,message_type = 109, stp = 2):
		
		msg_buffer = 1024
			
		# alti_msg = msp_altitude()
		# s.send(att_msg)
		# time.sleep(0.2)
		# recv_datab = s.recv(msg_buffer)
		# recv_data = list(recv_datab)
		recv_data = self.request_message(message_type)
		# print(recv_data)
		msgs = []

		for i in range(0,len(recv_data)):
			if i < len(recv_data)-4 and recv_data[i] == 36 and recv_data[i+1] == 77 and recv_data[i+2] == 62:
				n = recv_data[i+3]
				if i+4 >= len(recv_data):
					break
				msg_type = recv_data[i+4]
				if i+5+n >= len(recv_data):
					break
				check_sum = recv_data[i+5+n]
				message = recv_data[(i+5): (i+5+n)]
				msgs.append([msg_type, "length: " + str(n), message, "check_sum: "+ str(check_sum)])
		# print(msgs)

		req_msg_type = message_type
		req_msgs = []
		for i in msgs:
			if i[0] == req_msg_type:
				req_msgs.append(i[2])
					
		# print(req_msgs)

		# # for msg in req_msgs:           (for is commented as one msg among all the received msgs at that instant is enough)
		values = []
				
		if len(req_msgs)>0:
			rq_msg = req_msgs[0]
			
			i = 0
					
			int_list_alti = rq_msg[i:i+4]
			int_list_vario = rq_msg[i+4:]
			
			byte_val_alti = bytes(int_list_alti)
			byte_val_vario = bytes(int_list_vario)
			
			int_val_alti = int.from_bytes(byte_val_alti, "little", signed="True")
			int_val_vario = int.from_bytes(byte_val_vario, "little", signed="True")
				
			# printing int object
			values.append(int_val_alti)
			values.append(int_val_vario)
		
		# if len(values):
		#     val = "Altitude: " + str(values[0]) + ", Vario: " + str(values[1])
		#     print(val)
		#     return values
		# else:
		#     return [None, None]
		return values

		
	#MSP_RC
	def obs_rc_channel_vals(self,message_type = 105, stp = 2):
							
		recv_data = self.request_message(message_type)
		msgs = []

		for i in range(0,len(recv_data)):
			if i < len(recv_data)-4 and recv_data[i] == 36 and recv_data[i+1] == 77 and recv_data[i+2] == 62:
				n = recv_data[i+3]
				if i+4 >= len(recv_data):
					break
				msg_type = recv_data[i+4]
				if i+5+n >= len(recv_data):
					break
				check_sum = recv_data[i+5+n]
				message = recv_data[(i+5): (i+5+n)]
				msgs.append([msg_type, "length: " + str(n), message, "check_sum: "+ str(check_sum)])
		# print(msgs)

		req_msg_type = message_type
		req_msgs = []
		for i in msgs:
			if i[0] == req_msg_type:
				req_msgs.append(i[2])
					
		# print(req_msgs)
		values = []
		# for msg in req_msgs:           (commented as one msg among all the received msgs at that instant is enough)
		if len(req_msgs)>0:
			rq_msg = req_msgs[0]
		
			step = stp
			
			i = 0

			while i < len(rq_msg):
				int_list = rq_msg[i:i+step]
				byte_val = bytes(int_list)
					
				int_val = int.from_bytes(byte_val, "little", signed="False")
					
				# printing int object
				values.append(int_val)
				i = i+step
		
			print(values[0:8])   
			val = "Throttle: " + str(values[2]) #+ ", Pitch: " + str(values[1]) + ", Yaw: " + str(values[2])
			print(val)


	#MSP_RAW_IMU	
	def obs_imu_vals(self,message_type = 102, stp = 2):
							
		recv_data = self.request_message(message_type)
		msgs = []

		for i in range(0,len(recv_data)):
			if i < len(recv_data)-4 and recv_data[i] == 36 and recv_data[i+1] == 77 and recv_data[i+2] == 62:
				n = recv_data[i+3]
				if i+4 >= len(recv_data):
					break
				msg_type = recv_data[i+4]
				if i+5+n >= len(recv_data):
					break
				check_sum = recv_data[i+5+n]
				message = recv_data[(i+5): (i+5+n)]
				msgs.append([msg_type, "length: " + str(n), message, "check_sum: "+ str(check_sum)])

		req_msg_type = message_type
		req_msgs = []
		for i in msgs:
			if i[0] == req_msg_type:
				req_msgs.append(i[2])
						
		values = []
		if len(req_msgs)>0:
			rq_msg = req_msgs[-1]
			step = stp
			i = 0
			while i < len(rq_msg):
				int_list = rq_msg[i:i+step]
				byte_val = bytes(int_list)
						
				int_val = int.from_bytes(byte_val, "little", signed="False")
						
				# printing int object
				values.append(int_val)
				i = i+step
			
			# print(values)   
			# val = "Acc X: " + str(values[0]) + ", Acc Y: " + str(values[1]) + ", Acc Z: " + str(values[2]) + ", Gyro X: " + str(values[3]) + ", Gyro Y: " + str(values[4]) + ", Gyro Z: " + str(values[5]) + ", Mag X: " + str(values[6]) + ", Mag Y: " + str(values[7]) + ", Mag Z: " + str(values[8])
			# print(val)
			return values



if __name__ == '__main__':

	e_drone = Pluto()

	# while True:
		# obs_altitude_val()
		# obs_attitude_val()
		# obs_rc_channel_vals()
		# obs_imu_vals()

	#takeoff
	takeoff_cmd = e_drone.msp_set_cmd(1)
	e_drone.s.send(takeoff_cmd)
	time.sleep(1)

	#Throttle
	end_time = time.time() + 2
	while time.time()<end_time:
		throttle = e_drone.msp_set_raw(rcRoll = 1500, rcPitch = 1500, rcYaw = 1500, rcThrottle = 1800, rcAUX1 = 1500, rcAUX2 = 1500, rcAUX3 = 1800, rcAUX4 = 1500)
		e_drone.s.send(throttle)
		time.sleep(0.02)

	#Pitch forward
	end_time = time.time() + 1
	while(time.time() < end_time):
		pitch_for = e_drone.msp_set_raw(rcRoll = 1500, rcPitch = 1600, rcYaw = 1500, rcThrottle = 1700, rcAUX1 = 1500, rcAUX2 = 1500, rcAUX3 = 1500, rcAUX4 = 1500)
		e_drone.s.send(pitch_for)

	#Roll right
	end_time = time.time() + 1
	while(time.time() < end_time):
		pitch_for = e_drone.msp_set_raw(rcRoll = 1550, rcPitch = 1500, rcYaw = 1500, rcThrottle = 1700, rcAUX1 = 1500, rcAUX2 = 1500, rcAUX3 = 1500, rcAUX4 = 1500)
		e_drone.s.send(pitch_for)

	#land
	land_cmd = e_drone.msp_set_cmd(2)
	e_drone.s.send(land_cmd)

	e_drone.s.close()