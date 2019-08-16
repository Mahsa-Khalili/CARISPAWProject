"""
Author:         Kevin Ta
Date:           2019 June 24th
Purpose:        This Python library sets initial hardware registers and collects
				dat for the two IMUs.
				
				1. MPU6050 - 6 Axis IMU
				2. MPU9250 - 9 Axis IMU
"""
# IMPORTED LIBRARIES

import numpy as np
import os, sys
import time
import pickle as pkl
import operator

import asyncio

from multiprocessing import Process, Queue
import threading

# LOCALLY IMPORTED LIBRARIES

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(dir_path, 'libraries'))

import carisPAWBuffers_pb2 as frameMsgStruct

from mpu6050 import mpu6050
#~ from mpu9250 import mpu9250
from mpu9250 import *
from fusion import Fusion

#CLASSES

class ClMpu6050DAQ(threading.Timer):
	
	def __init__(self, dataQueue, runMarker):
		"""
		Purpose:	Initialize 6-axis IMU.
		Passed:		Queue for data transfer between processes.
		"""
		
		self.sensor = mpu6050(0x68)
		self.dataQueue = dataQueue
		self.runMarker = runMarker
		self.offset = np.zeros(6)
		
	def fnRetrieveData(self):
		"""
		Purpose:	Send data to main data queue for transfer with timestamp and sensor ID.
		Passed:		None
		"""
		
		#~ acc = np.multiply(self.sensor.accel - self.offset[0:3], 9.8065)
		#~ gyro = self.sensor.gyro - self.offset[3:6]
		#~ self.dataQueue.put_nowait(['IMU_6', time.time(), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]])
		timeRecorded = time.time()
		data = list(map(operator.sub,self.sensor.allSensors, self.offset[0:6]))
		data[0:3] = [a*b for a,b in zip(data[0:3], [9.8065]*3)]
		self.dataQueue.put(['IMU_6', timeRecorded, data[0], -data[1], -data[2], data[3], -data[4], -data[5]])

	
	def fnRun(self, frequency):
		"""
		Purpose:	Script that runs until termination message is sent to queue.
		Passed:		Frequency of 
		"""
		
		self.offset = pkl.load(open('IMU6050Offset.pkl', 'rb'))
		timeStart = time.time()
		
		waitTime = 1/frequency
		
		self.trigger = threading.Event()
		self.trigger.set()
		
		timerRepeat = threading.Thread(target=self.fnRunThread, args = (waitTime, ) )
		timerRepeat.start()
		
		while self.runMarker.empty():
		
			self.trigger.wait()
			self.trigger.clear()
			self.fnRetrieveData()
		
		timerRepeat.join()

	def fnRunThread(self, waitTime):
		while self.runMarker.empty():
			time.sleep(waitTime - (time.perf_counter() % waitTime))
			self.trigger.set()

	def fnCalibrate(self):
		
		noCalArray = np.zeros((3, 1000))
		
		calArray = np.zeros((9, 1000))
		for i in range(1000):
			calArray[0:6, i] = (self.sensor.accel + self.sensor.gyro)
		
		self.offset[0:6] = np.mean(calArray[0:6], axis=1)
		if self.offset[2] > 0:
			self.offset[2] = self.offset[2]  - 1 
		else:
			self.offset[2] = self.offset[2]  + 1
		print(self.offset)
		
		pkl.dump(self.offset, open('IMU6050Offset.pkl', 'wb'))

class ClMpu9250DAQ:
	
	def __init__(self, dataQueue, runMarker):
		
		
		self.Fusion = Fusion(timeDiff)
		self.offset = np.zeros(12)
		self.dataQueue = dataQueue
		self.runMarker = runMarker
		self.loop = asyncio.get_event_loop()
		self.sensor = mpu9250(self.loop) 
	
	def fnRetrieveData(self):
		
		#~ self.Fusion.update(fixImu(self.sensor.accel - self.offset[0:3]), fixImu(self.sensor.gyro - self.offset[3:6]), fixMag(np.multiply((self.sensor.mag - self.offset[6:9]), self.offset[9:12])), datetime.datetime.now())
		
		#~ acc = np.multiply(self.sensor.accel - self.offset[0:3], 9.8065)
		#~ acc = [a*b for a,b in zip(self.sensor.accel - self.offset[0:3], [9.8065]*3)]
		#~ gyro = self.sensor.gyro - self.offset[3:6]
		#~ mag = [0, 0, 0]
		#~ mag = self.sensor.mag
		
		timeRecorded = time.time()
		
		data = list(map(operator.sub, self.sensor.allSensors, self.offset[0:9]))
		#~ data = self.loop.run_until_complete(self.sensor.i2c_allSensors()) - self.offset[0:9]
		data[0:3] = [a*9.8065 for a in data[0:3]]
		self.dataQueue.put(['IMU_9', timeRecorded, -data[0], data[1], -data[2], -data[3], data[4], -data[5], -data[7], data[6], data[8]])
		
		#~ mag = [0, 0, 0]
		#~ ang = [self.Fusion.heading, self.Fusion.pitch, math.fmod((self.Fusion.roll + 180)  + 180 * 3, 2* 180) -180]
		
		#~ self.dataQueue.put_nowait(['IMU_9', time.time(), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]])
				
		#~ print('{:11.6f}, {:11.6f}, {:11.6f}'.format(self.Fusion.heading, self.Fusion.pitch, math.fmod((self.Fusion.roll + 180)  + 180 * 3, 2* 180) -180))
			
	
	def fnRun(self, frequency):
		
		self.offset = pkl.load(open('IMU9250Offset.pkl', 'rb'))
		timeStartB = time.time()
		
		waitTime = 3/frequency

		self.trigger = threading.Event()
		self.trigger.set()
		
		timerRepeat = threading.Thread(target=self.fnRunThread, args = (waitTime, ) )
		timerRepeat.start()
		
		while self.runMarker.empty():
		
			self.trigger.wait()
			self.trigger.clear()
			self.fnRetrieveData()
		
		timerRepeat.join()
		
		#~ while (self.runMarker.empty()):
			#~ self.fnRetrieveData()
			#~ time.sleep(waitTime - (time.perf_counter() % (waitTime)))

	def fnRunThread(self, waitTime):
		while self.runMarker.empty():
			time.sleep(waitTime - (time.perf_counter() % waitTime))
			self.trigger.set()
	
	def fnCalibrate(self):
		
		noCalArray = np.zeros((3, 1000))
		
		calArray = np.zeros((9, 1000))
		for i in range(1000):
			calArray[0:6, i] = (self.sensor.accel + self.sensor.gyro)
		
		self.offset[0:6] = np.mean(calArray[0:6], axis=1)
		if self.offset[2] > 0:
			self.offset[2] = self.offset[2]  - 1 
		else:
			self.offset[2] = self.offset[2]  + 1
		
		#~ self.offset[6:12] = [28.4884, 29.1631, -1.3495, 1.1899, 0.9215, 0.9308]
		#~ self.offset[6:12] = [25.5646, 28.8632, 0.7497, 1.1249, 0.9463, 0.9485]
		
		
		print(self.offset)
		
		pkl.dump(self.offset, open('IMU9250Offset.pkl', 'wb'))
		
		pass


def timeDiff(end, start):
	return ((end-start).microseconds)*1000000


if __name__ == "__main__":
	
	dataQueue9250 = Queue()
	dataQueue6050 = Queue()
	dataQueue = Queue()
	runMarker = Queue()
	frequency = 300
	
	#~ instMpu6050DAQ = ClMpu6050DAQ(dataQueue6050, runMarker)
	#~ instMpu9250DAQ = ClMpu9250DAQ(dataQueue9250, runMarker)
	instMpu6050DAQ = ClMpu6050DAQ(dataQueue, runMarker)
	instMpu9250DAQ = ClMpu9250DAQ(dataQueue, runMarker)
	
	#~ P6050 = Process(target=AsyncMpuDAQ, args = (instMpu6050DAQ, frequency, ))
	#~ P9250 = Process(target=AsyncMpuDAQ, args = (instMpu9250DAQ, frequency, ))
	P6050 = Process(target=instMpu6050DAQ.fnRun, args = (frequency, ))
	P9250 = Process(target=instMpu9250DAQ.fnRun, args = (frequency, ))
	P6050.start()
	P9250.start()
	
	counter = [ -20, -20]
	timeStamp9250 = []
	timeStamp6050 = []

	timeStart = time.time()
	
	while(time.time() < timeStart + 100):
		bufferIMU = dataQueue.get()
		if bufferIMU[0] == 'IMU_9':
			counter[0] +=1
			timeStamp9250.append(bufferIMU[1])
		elif bufferIMU[0] == 'IMU_6':
			counter[1] +=1
			timeStamp6050.append(bufferIMU[1])
		if counter[0] > 500:
			counter[0] = 0
			print('IMU_9 Frequency: {}'.format(500/(timeStamp9250[-1]-timeStamp9250[-501])))
		elif counter[1] > 500:
			counter[1] = 0
			print('IMU_6 Frequency: {}'.format(500/(timeStamp6050[-1]-timeStamp6050[-501])))
	
	runMarker.put('Close.')
	
	pass
