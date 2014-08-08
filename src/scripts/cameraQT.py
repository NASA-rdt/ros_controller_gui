#!/usr/bin/env python

#Modified by William Baker: GSFC - Summmer 2014 
#Origin: http://rafaelbarreto.wordpress.com/2011/08/27/a-pyqt-widget-for-opencv-camera-preview/

import cv, cv2, sys, numpy, glob
 
from PyQt4 import QtCore
from PyQt4 import QtGui

#a function to add crosshairs, only works for 5,5 right now
def addCrosshair(frame, gridsize = (11,11) ):

	if gridsize[0] % 2 != 1:
		gridsize = (gridsize[0] + 1 , gridsize[1] )
	if gridsize[1] % 2 != 1:
		gridsize = (gridsize[0], gridsize[1]+1 )
	#gridsizes are definitely odd.
	h,w = frame.shape[:2]
	w = w/gridsize[0]#grid size width
	h = h/gridsize[1]#grid size height
	x = []
	y = []
	xw = w
	yh = h
	for i in range( 0, gridsize[0]-1 ):
		x.append(xw)
		xw += w
	for i in range( 0, gridsize[1]-1 ):
		y.append(yh)
		yh += h
	#print x
	#print y
	g = []
	for i in range( 0, len(x) ):
		for j in range( 0, len(y) ):
			g.append( (x[i],y[j]) )
#	print g
	#if you want this to work for grids not 10x10...
	#change these values to be relaitve.
	'''
	Wx = 1 #width between V crosshairs
	Hy = 1 #width between H crosshairs
	s = len(g)
	left = (gridsize[0]-Wx)/2
	right = left+Wx
	
	top = (gridsize[1]-Hy)/2
	bottom = top+Hy

	cv2.line(frame,(left,top),(left,bottom),(0,0,0), thickness = 2)
	cv2.line(frame,(right,top),(right,bottom),(0,0,0), thickness = 2)
	'''
	cv2.line(frame,g[14],g[44],(0,0,0), thickness = 2)
	cv2.line(frame,g[44],g[41],(0,0,0), thickness = 2)

	cv2.line(frame,g[15],g[45],(0,0,0), thickness = 2)
	cv2.line(frame,g[45],g[48],(0,0,0), thickness = 2)

	cv2.line(frame,g[51],g[54],(0,0,0), thickness = 2)
	cv2.line(frame,g[54],g[84],(0,0,0), thickness = 2)

	cv2.line(frame,g[55],g[58],(0,0,0), thickness = 2)
	cv2.line(frame,g[55],g[85],(0,0,0), thickness = 2)

	
	
	return frame
 
class OpenCVQImage(QtGui.QImage):

	def __init__(self, opencvBgrImg, scale = (False)):
		if scale[0]:
			opencvBgrImg = cv2.resize(opencvBgrImg,(scale[1],scale[2]))
		h,w = opencvBgrImg.shape[:2]
		# it's assumed the image is in BGR format
		opencvRgbImg = cv2.cvtColor(opencvBgrImg, cv2.cv.CV_BGR2RGB)
		self._imgData = opencvRgbImg.tostring()
		super(OpenCVQImage, self).__init__(self._imgData, w, h, \
			QtGui.QImage.Format_RGB888)

class CameraWidget(QtGui.QWidget):
 
	newFrame = QtCore.pyqtSignal(numpy.ndarray)
 
	def __init__(self, cameraDevice, parent=None, scale = 1.0, overlay = None):
		super(CameraWidget, self).__init__(parent)
 
		self._frame = None
		self._cameraDevice = cameraDevice
		self._cameraDevice.newFrame.connect(self._onNewFrame)
 
		device = self._cameraDevice._cameraDevice
		if device.isOpened():
			w = device.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
			h = device.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
		else:
			self._frame = cameraDevice.getNoImage()
			h,w = self._frame.shape[:2]
		self.setMinimumSize(w, h)
		self.setMaximumSize(w, h)
		self.scaled = (False,w,h)
		self.overlay = overlay
 
	@QtCore.pyqtSlot(numpy.ndarray)
	def _onNewFrame(self, frame):
		self._frame = frame.copy()#cv.CloneImage(frame)
		#test = cv2.cv.fromarray(self._frame)
		if self.overlay == 'crosshair':
			addCrosshair(self._frame)
		self.newFrame.emit(self._frame)
		self.update()
	
	#@scaled.setter
	def setWidth(self, width):
		w = self.size().width()
		h = self.size().height()
		h = int(h * float(width)/w)
		self.scaled = (True,int(width),h)
		self.setMinimumSize(width, h)
		self.setMaximumSize(width, h)
		
 
	def changeEvent(self, e):
		if e.type() == QtCore.QEvent.EnabledChange:
			if self.isEnabled():
				self._cameraDevice.newFrame.connect(self._onNewFrame)
			else:
				self._cameraDevice.newFrame.disconnect(self._onNewFrame)

	def paintEvent(self, e):
		if self._frame is None:
			return
		painter = QtGui.QPainter(self)
		image = OpenCVQImage(self._frame, scale = self.scaled)
		painter.drawImage(QtCore.QPoint(0, 0), image)

class CameraDevice(QtCore.QObject):
 
	_DEFAULT_FPS = 30
	NO_IMAGE = '/home/nasa/Pictures/button_A.png'
	newFrame = QtCore.pyqtSignal(numpy.ndarray)
 
	def __init__(self, cameraId=0, mirrored=False, parent=None, fps = 30):
		super(CameraDevice, self).__init__(parent)
 		self.cameraId = cameraId
		self.mirrored = mirrored
		self._cameraDevice = cv2.VideoCapture(cameraId)
		self._timer = QtCore.QTimer(self)
		self._timer.timeout.connect(self._queryFrame)
		self._timer.setInterval(1000/fps)#self.fps)
		self.paused = False
 	def getNoImage(self):
		return cv2.imread(self.NO_IMAGE)
	@QtCore.pyqtSlot()
	def _queryFrame(self):
		if self._cameraDevice.isOpened():
			ret, frame = self._cameraDevice.read()
			if not ret:
				print 'Unable to retrieve image. Pausing camera device.'
				self.paused = True
				self.newFrame.emit(self.getNoImage())
			#print 'isOpen'#,self.cameraId,ret
			if frame is not None:
				if self.mirrored:
					height,width = frame.shape[:2]#shape
					#create empty matrix
					mirroredFrame = numpy.zeros((height,width,3), numpy.uint8)
					cv2.flip(frame, 1, mirroredFrame)#flip
					frame = mirroredFrame#set
				self.newFrame.emit(frame)
 	
	@property
	def paused(self):
		return not self._timer.isActive()

	@property
	def frameSize(self):
		#print 'frameSize'
		w = self._cameraDevice.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
		h = self._cameraDevice.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
		return int(w), int(h)
  
	@paused.setter
	def paused(self, p):
		if p:
			self._timer.stop()
		else:
			self._timer.start()
	
	@property
	def fps(self):
		fps = int(self._cameraDevice.get(cv2.cv.CV_CAP_PROP_FPS))
		#cv.GetCaptureProperty(self._cameraDevice, cv.CV_CAP_PROP_FPS))
		if not fps > 0:
			fps = self._DEFAULT_FPS
		return fps

def getCameras(camArray):
		if len(camArray) < 1:
			print 'No device chosen, using default.'
			return [-1]
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
def findDevices( devices = None):
	availableDevices = glob.glob('/dev/video*')
	if devices == None:
		return availableDevices
	else:
		print 'Available devices:',availableDevices
		availableCameras = getCameras(availableDevices)
		requestedCameras = getCameras(devices)
		if isinstance(devices[0], str):
			if devices[0].upper() == 'ALL':
				print 'Adding ALL avaliable camera feeds.'
				requestedCameras = availableCameras

		capture = []
		for camera in requestedCameras:
			for avail in availableCameras:
				#print 'is camera(%d) == avail(%d)' % (camera,avail)
				if camera == avail:
					print availableDevices[availableCameras.index(avail)],'is ready.'
					capture.append(camera)
		return capture
	
def _main(args):
 
	@QtCore.pyqtSlot(numpy.ndarray)
	def onNewFrame(frame):#use this to modify the image before printing
		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		msg = "processed frame"
		font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 1.0, 1.0)
		tsize, baseline = cv.GetTextSize(msg, font)
		h,w = frame.shape[:2]
		tpt = (w - tsize[0]) / 2, (h - tsize[1]) / 2
		#cv.PutText(frame, msg, tpt, font, cv.RGB(255, 0, 0))
 
	
	app = QtGui.QApplication(args)
 	
	print findDevices()
	if len(args) < 1:
		args = [0]
	print findDevices(args[1:])
	cams = getCameras(args[1:])
	widgets = []
	for cam in range(len(cams)):
		cameraDevice = CameraDevice(cams[cam])
		cameraWidget = CameraWidget(cameraDevice,overlay = 'crosshair')
		cameraWidget2 = CameraWidget(cameraDevice)
		cameraWidget.setWidth(300)
		widgets.append(cameraWidget)
		cameraWidget.show()
		cameraWidget2.show()
 	
	#cameraDevice = CameraDevice(1)
	#cameraWidget1 = CameraWidget(cameraDevice)
	#cameraWidget1.show()
	#cameraDevice1 = CameraDevice(3)
	#cameraWidget2 = CameraWidget(cameraDevice1)
	#cameraWidget2.show()
 	
	sys.exit(app.exec_())


if __name__ == '__main__':
	_main(sys.argv)
