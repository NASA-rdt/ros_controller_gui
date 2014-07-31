#!/usr/bin/env python

from PyQt4 import QtCore
import sys, cv2, glob, numpy as np

class CameraFeed(QtCore.QThread):

	def __init__(self,devices = [0]):
		QtCore.QThread.__init__(self)
		self.running=True
		self.captures = self.setup(devices)

	def setup(self, devices):		
		availableDevices = glob.glob('/dev/video*')
		print 'Available devices:',availableDevices
		availableCameras = self.getCameras(availableDevices)
		
		requestedCameras = self.getCameras(devices)
		if isinstance(devices[0], str):
			if devices[0].upper() == 'ALL':
				print 'Adding ALL avaliable camera feeds.'
				requestedCameras = availableCameras

		capture = []
		for camera in requestedCameras:
			for avail in availableCameras:
				#print 'is camera(%d) == avail(%d)' % (camera,avail)
				if camera == avail:
					print 'Loading ',availableDevices[availableCameras.index(avail)]
					capture.append(cv2.VideoCapture(camera))
		if len(capture) < 1:
			print 'no requested devices found.'
			return None
		return capture


	def run(self):
		while self.running and self.captures is not None:
			for capture in self.captures:
				ret,frame = capture.read()
				if frame is not None:
					height, width = frame.shape[:2]
					self.emit(QtCore.SIGNAL("newFrame(int,QString,int,int)")
						,self.captures.index(capture),QtCore.QString(frame.tostring)(),width,height)

	def getCameras(self,camArray):
		c = []
		for cam in camArray:
			if isinstance(cam, str):
				temp = ""
				for i in cam:
					if i.isdigit():
						temp+=i
				if temp != "":
					c.append(int(temp))
			elif isinstance(cam,int):
				c.append(cam)
		return c

	def getDigits(self,string):
		temp = ""
		for i in string:
			if i.isdigit():
				temp+=i
		if temp != '':
			return int(temp)
		else:
			return None
	def getFrames(self):
		frames = []
		for capture in self.captures:
			ref, frame = capture.read()
			frames.append(frame)
		return frames
	def release(self):
		for capture in self.captures:
			capture.release()
		
def toggle_example(feeds):
	TIMESTEP = 100
	count = 0
	switch = 0
	while True:
		#frames = feeds.getFrames()#
		#cv2.imshow('frame',frames[2])
		ref, frame = feeds.captures[switch].read()
		if frame is not None:
			cv2.imshow('frame',frame)
			if cv2.waitKey(33)==27:
				break
			count+=1
			if count > TIMESTEP:
				count = 0
				switch+=1;
				if switch == len(feeds.captures):
					switch = 0

	cv2.destroyAllWindows()

def side_by_side_example(feeds):
	width = 0
	height = 0
	for frame in feeds.getFrames():
		if frame is not None:
			h, w = frame.shape[:2]
			width+=w
			height=max(h,height)
	vis = np.zeros((height,width,3), np.uint8)#create empty matrix
	while True:
		#combine images images
		w0 = 0
		for frame in feeds.getFrames():
			if frame is not None:
				h, w = frame.shape[:2]
				vis[:h, w0:w0+w,:3] = frame
				w0=w
		cv2.imshow('frame',vis)
		if cv2.waitKey(33)==27:
			break

if __name__ == '__main__':
	args = sys.argv[1:]
	if len(args) < 1:
		args = [0]
		print 'No devices specified, starting default.'
	print args
	feeds = CameraFeed(args)#modify for camera number
	try:	
		if feeds.captures is None or len(feeds.captures) < 1:
			sys.exit(0)
		print 'Starting Example...'
		side_by_side_example(feeds)
		#toggle_example(feeds)
	except KeyboardInterrupt:
		feeds.release()

