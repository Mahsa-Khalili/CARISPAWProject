"""
Author:         Kevin Ta
Date:           2019 June 24th
Purpose:        This Python script should autostart when the Raspberry Pi is first powered.
                The script will initiate a UDP/TCP/BT connection with the laptop to
                wirelessly transmit frame sensor data. 
                
                Sensors:
                  1. MPU9250 - 9-Axis IMU
                  2. MPU6050 - 6-Axis IMU (Redundant)
                  3. AJ-SR04M-1 - Ultrasonic Sensor
                  4. AJ-SR04M-2 - Ultrasonic Sensor
                  5. Sony IMX219 - RaspberryPi Camera Module v2
"""

# IMPORTED LIBRARIES

import smbus
import math
import time
import datetime
import socket
import bluetooth
import os
import sys

import pickle as pkl
import numpy as np

from cobs import cobs
from multiprocessing import Process, Queue
import asyncio

# LOCALLY IMPORTED LIBRARIES
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(dir_path, 'libraries'))

import carisPAWBuffers_pb2 as carisPAWBuffers

from mpu6050 import mpu6050
from mpu9250 import mpu9250
from fusion import Fusion

from IMUSensorLib import *
from USSSensorLib import *
from PiCamSensorLib import *

# DEFINITIONS

HOST = '192.168.0.100'
PORT = 65432
BTAddress = '54:8c:a0:a4:8e:a2'

SENSOR_LIST = ['IMU_9', 'IMU_6', 'USS_DOWN', 'USS_FORW', 'PI_CAM']

ACTIVE_SENSORS = [0, 1]

# CLASSES

class ClTransferClient:
	"""
	Class for establishing wireless communications.
	"""
	
	def __init__(self, protocol = 'UDP'):
		"""
		Purpose:	Initialize various sensors 
		Passed: 	Optionally the communication protocol
		"""
		
		# Stores protocol in class variable
		self.protocol = protocol
		
		# Initiates various socket connnections based on passed protocol
		if self.protocol == 'TCP':
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.socket.settimeout(10)
			self.socket.connect((HOST, PORT))
			print('Connected.')
			
		elif self.protocol == 'UDP':
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.socket.connect((HOST, PORT))
		
		elif self.protocol =='BT':
			self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
			self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
			self.sock.bind(('',3))
			self.sock.listen(1)
			self.socket, self.address = self.sock.accept()
		
		# Create multiprocessing queue for data transfer pipeline
		self.dataQueue = Queue()
		self.runMarker= Queue()
		
		# Initiate dictionary for data acquisition instances
		self.instDAQLoop = {} 
		
		# Create sensor instances based on which sensors you activated
		for sensor in ACTIVE_SENSORS:
			if sensor == 0:
				self.instDAQLoop[SENSOR_LIST[sensor]] = ClMpu9250DAQ(self.dataQueue, self.runMarker)
			elif sensor == 1:
				self.instDAQLoop[SENSOR_LIST[sensor]] = ClMpu6050DAQ(self.dataQueue, self.runMarker)
			elif sensor == 2:
				self.instDAQLoop[SENSOR_LIST[sensor]] = ClProximitySensorDAQ(self.dataQueue, self.runMarker)
			elif sensor == 4:
				self.instDAQLoop[SENSOR_LIST[sensor]] = ClPiCameraDAQ(self.dataQueue, self.runMarker)
		
		# Send packet of active sensors
		self.socket.send(pkl.dumps(ACTIVE_SENSORS))

	def fnStart(self, frequency):
		"""
		Purpose:	Collect sensor outputs from data queue and pipe them 
					to socket to transmit to laptop
		Passed: 	Optionally the communication protocol
		"""		
		
		# Print start statement to keep track
		print('Start Process.')
		
		# Set start time for potential frequency calculations
		timeStart = time.time()
		
		# Create process dictionary to store instance start methods
		processes = {}

		# Activate various instances
		for sensor in ACTIVE_SENSORS:
			processes[SENSOR_LIST[sensor]] = Process(target=self.instDAQLoop[SENSOR_LIST[sensor]].fnRun, args = (frequency, ))
			processes[SENSOR_LIST[sensor]].start()

		# Continuously
		while True:

			# Get first entry from queue
			transmissionData = self.dataQueue.get()

			# Formats data depending on if IMU or USS
			if transmissionData[0] in ['IMU_9', 'IMU_6']:
				dataBuffer = self.fnIMUtoPB(transmissionData)
			elif transmissionData[0] in ['USS_DOWN', 'USS_FORW']:
				dataBuffer = self.fnUSStoPB(transmissionData)
			
			# Encode message using constant oversized buffering
			dataCobs = cobs.encode(dataBuffer.SerializeToString())
			
			# Send encoded message through socket
			self.socket.send(dataCobs)
			
			# Send a 0 value to signal end of message
			if self.protocol == ('TCP' or 'BT'):
				self.socket.send(b'\x00')
		
	def fnShutDown(self):
		"""
		Purpose:	Shutdown sockets
		Passed: 	None
		"""
		
		print('Closing Socket')
		self.socket.close()
		try:
			self.sock.close()
		except Exception as e:
			print(e)

	def fnIMUtoPB(self, data):
		"""
		Purpose:	Converts IMU data to protobuf message
		Passed: 	IMU data
		"""
		
		# Sets 6-axis data
		dataBuffer = carisPAWBuffers.frameUnit()
		dataBuffer.time_stamp = data[1]
		dataBuffer.acc_x =  data[2]
		dataBuffer.acc_y =  data[3]
		dataBuffer.acc_z =  data[4]
		dataBuffer.angular_x =  data[5]
		dataBuffer.angular_y =  data[6]
		dataBuffer.angular_z =  data[7]
		
		# Sets 9-axis data
		if len(data) > 8:
			dataBuffer.sensorType = carisPAWBuffers.frameUnit.IMU_9
			dataBuffer.mag_x =  data[8]
			dataBuffer.mag_y =  data[9]
			dataBuffer.mag_z =  data[10]
		else:
			dataBuffer.sensorType = carisPAWBuffers.frameUnit.IMU_6
	
		return dataBuffer
		
	def fnUSStoPB(self, data):
		"""
		Purpose:	Converts USS data to protobuf message
		Passed: 	USS data
		"""
				
		dataBuffer = carisPAWBuffers.frameUnit()
		dataBuffer.time_stamp = data[1]
		dataBuffer.USensorDownward =  data[2]
		if data[0] == 'USS_DOWN':
			dataBuffer.sensorType = carisPAWBuffers.frameUnit.USS_DOWN
	
		return dataBuffer
	

# MAIN PROGRAM

if __name__=="__main__":
	
	# Sets marker to determine when to call shutdown functions
	connectedStatus = False

	# Create dummy queues to create instances for static calibration
	dummyQueue = Queue
	dummyMarker = Queue

	# Calibration
	instMpu9250DAQ = ClMpu9250DAQ(dummyQueue, dummyMarker)
	instMpu9250DAQ.fnCalibrate()
	instMpu6050DAQ = ClMpu6050DAQ(dummyQueue, dummyMarker)
	instMpu6050DAQ.fnCalibrate()

	# Continously try to reconnect if connection fails
	while True:
		
		try:
			print('Connecting to computer...')
			instTransferClient = ClTransferClient('TCP')
			connectedStatus = True
			instTransferClient.fnStart(300)
			
		except Exception as e:
			time.sleep(1)
			# Shutdown sockets if necessary
			if connectedStatus:
				instTransferClient.runMarker.put(False)
				instTransferClient.fnShutDown()
				instTransferClient.runMarker.close()
				instTransferClient.dataQueue.close()
				connectedStatus = False
			print(e)

		pass
