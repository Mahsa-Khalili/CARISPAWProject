"""
Author:         Kevin Ta
Date:           2019 June 26th
Purpose:        This Python library runs the ultrasonic sensors.
				
				1. AJ-SR04M-1 - Ultrasonic Sensor
				2. AJ-SR04M-2 - Ultrasonic Sensor
"""
# IMPORTED LIBRARIES

import RPi.GPIO as GPIO
import time, threading
import numpy as np
import os, sys

from multiprocessing import Queue

# LOCALLY IMPORTED LIBRARIES

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(dir_path, 'libraries'))

import carisPAWBuffers_pb2 as frameMsgStruct

#CLASSES

class ClProximitySensorDAQ:
	
	def __init__(self, dataQueue, runMarker):
	
		TRIG_PIN = 27                                  #Associate pin 15 to TRIG
		ECHO_PIN = 17                                  #Associate pin 14 to Echo
		distance = 0
		isInit = False

		self.dataQueue = dataQueue
		self.runMarker = runMarker

		GPIO.setwarnings(False)

		#GPIO Mode (BOARD / BCM)
		GPIO.setmode(GPIO.BCM)

		#~ print ("Distance measurement in progress")
		self.TRIG = TRIG_PIN
		self.ECHO = ECHO_PIN
		GPIO.setup(self.TRIG,GPIO.OUT)                  #Set pin as GPIO out
		GPIO.setup(self.ECHO,GPIO.IN)                   #Set pin as GPIO in
		
		self.distance = 0
	
	def fnRun(self, frequency):
		self.initialize()
		while (self.runMarker.empty()):
			self.getDistance()
			self.dataQueue.put(['USS_DOWN', time.time(), self.distance])
			time.sleep(8.0/frequency - (time.time() % (8.0/frequency)))
		GPIO.cleanup()
	
	def initialize(self):

		GPIO.output(self.TRIG, False)                 #Set TRIG as LOW
		print ("Waiting For Sensor To Settle")
		time.sleep(2)                            #Delay of 2 seconds
		self.isInit = True

	def getDistance(self):
		GPIO.output(self.TRIG, True)                  #Set TRIG as HIGH
		time.sleep(0.00001)                      #Delay of 0.00001 seconds
		GPIO.output(self.TRIG, False)                 #Set TRIG as LOW

		pulse_start = time.time() 

		while GPIO.input(self.ECHO)==0:               #Check if Echo is LOW
			pulse_start = time.time()              #Time of the last  LOW pulse

		while GPIO.input(self.ECHO)==1:               #Check whether Echo is HIGH
			pulse_end = time.time()                #Time of the last HIGH pulse 

		distance = round((pulse_end-pulse_start)*17150, 2)            #Round to two decimal points

		if distance > 0 and distance < 800000:     #Is distance within range
			#print "Distance:",distance - 0.5,"cm"  #Distance with calibration
			self.distance = distance
			
		#~ print(self.distance)
	

if __name__ == "__main__":
	
	dummyQueue = Queue()
	dummyMarker = Queue()
	
	sensor = ClProximitySensorDAQ(dummyQueue, dummyMarker)
	sensor.fnRun(200)
	
	timeStamp = []
	
	timeStart = time.time()
	
	counter = 0
	
	while(time.time() < timeStart + 100):
		bufferUSS = dummyQueue.get()
		counter +=1
		timeStamp.append(bufferUSS[1])
		if counter[0] > 50:
			counter = 0
			print('USS Frequency: {}'.format(50/(timeStamp[-1]-timeStamp[-51])))
	
