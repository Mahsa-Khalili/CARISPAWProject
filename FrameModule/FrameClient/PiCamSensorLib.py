"""
Author:         Kevin Ta
Date:           2019 August 6th
Purpose:        This Python library runs the Pi Camera.
"""
# IMPORTED LIBRARIES

from picamera import PiCamera
import time
import io, os, sys
import numpy as np
from PIL import Image

from multiprocessing import Process, Queue

# LOCALLY IMPORTED LIBRARIES

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(dir_path, 'libraries'))

import carisPAWBuffers_pb2 as frameMsgStruct

#CLASSES

class ClPiCameraDAQ:
	
	def __init__(self, dataQueue, runMarker):
	
		self.dataQueue = dataQueue
		self.runMarker = runMarker

	
	def fnRun(self, frequency):
		
		self.cam = PiCamera()
		self.cam.resolution = (1024,768)
		self.cam.framerate = 15
		self.stream = io.BytesIO()
		self.cam.start_preview(alpha=200)
		
		time.sleep(2)
		
		while (self.runMarker.empty()):
			print('1')
			self.fnGetImage()
			print('2')
			self.dataQueue.put(['PI_CAM', time.time(), Image.open(self.stream)])
			print('3')
			time.sleep(300/frequency - (time.time() % (300/frequency)))
			print('4')
	
	def fnGetImage(self):

		self.stream.seek(0)
		self.stream.truncate()
		self.cam.capture(self.stream, format='jpeg')
	

if __name__ == "__main__":
	
	dummyQueue = Queue()
	dummyMarker = Queue()
	
	frequency = 100
	
	instSensor = ClPiCameraDAQ(dummyQueue, dummyMarker)
	
	PCam = Process(target=instSensor.fnRun, args = (frequency, ))
	PCam.start()
	
	timeStamp = []
	
	timeStart = time.time()
	
	counter = 0
	
	while(time.time() < timeStart + 100):
		bufferPC = dummyQueue.get()
		image = bufferPC[2]
		image.show()
		counter +=1
		timeStamp.append(bufferPC[1])
		if counter > 5:
			counter = 0
			print('PC Frequency: {}'.format(300/(timeStamp[-1]-timeStamp[-51])))
			
	#~ camera = PiCamera()
	#~ camera.resolution = (1024,768)
	#~ camera.framerate = 15
	
	#~ stream = io.BytesIO()
	
	#~ camera.start_preview(alpha=200)
	#~ time.sleep(2)
	#~ camera.capture(stream, format='jpeg')
	#~ camera.stop_preview()
	
	#~ stream.seek(0)
	#~ image = Image.open(stream)
	#~ print('The size of the image is:')
	#~ print(image.format, image.size, image.mode)
	#~ image.show()
	
	#~ stream.seek(0)
	#~ stream.truncate()
	
	#~ camera.start_preview(alpha=200)
	#~ time.sleep(2)
	#~ camera.capture(stream, format='jpeg')
	#~ camera.stop_preview()
	
	#~ stream.seek(0)
	#~ image = Image.open(stream)
	#~ image.show()
	
	#~ input('Pause')
