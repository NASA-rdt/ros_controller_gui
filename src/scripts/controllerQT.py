
from PyQt4 import QtGui, QtCore
import js_read

def scale(x, in_min, in_max, out_min, out_max):
	value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	value = min(value,out_max)
	value = max(value, out_min)
	return value

class Joystick():
	def __init__(self,x,y,inner_rad = 5,outer_rad = 15,scale = 32768): 
		self.center = QtCore.QPoint(x,y)
		self.value=QtCore.QPoint(0,0)
		self.inner_rad = inner_rad
		self.outer_rad = outer_rad
		self.scale = scale
	def setValue(self,index,x):
		val = scale(x,-self.scale,self.scale,-self.outer_rad,self.outer_rad)
		if index == 0:#X
			self.value.setX(-val)
		elif index == 1:#X
			self.value.setY(-val)
	def drawWidget(self, qp):
		qp.setBrush(QtGui.QColor(200, 200, 80))
		qp.setPen(QtGui.QColor(0,0,0))#QtGui.QColor(0, 0, 0))
		value = self.center.__sub__(self.value)
		qp.drawEllipse(value,self.inner_rad,self.inner_rad)#drawRect(0, 0, full, h)
	
class ControllerWidget(QtGui.QWidget):
  
    def __init__(self):      
        super(ControllerWidget, self).__init__()
        
        self.initUI()
        
    def initUI(self):
        self.setMinimumSize(200, 200)
	self.joysticks = [Joystick(46,99),Joystick(196,145)]

	fileName = '/home/nasa/Pictures/xbox-controller-hi.png'
	image = QtGui.QImage(fileName)
	if image.isNull():
		QtGui.QMessageBox.information(self, "Image Viewer",
			"Cannot load %s." % fileName)
		return
	pixmap = QtGui.QPixmap.fromImage(image)
	self.pixmap = pixmap.scaled(300, 300, QtCore.Qt.KeepAspectRatio)


    def setValue(self, index, stick, value):
	if(index < len(self.joysticks)):
		self.joysticks[index].setValue(stick,value)
	self.repaint()

#	def resizeEvent(self,e):
#		print 'resized'

    def paintEvent(self, e):
      
        qp = QtGui.QPainter()
        qp.begin(self)
        self.drawWidget(qp)
        qp.end()
      
      
    def drawWidget(self, qp):

        size = self.size()
      	qp.drawPixmap(0,0,self.pixmap)
	for stick in self.joysticks:
		stick.drawWidget(qp)
DEADZONE = 4000

class ControllerCore(QtCore.QThread):

    #This is the signal that will be emitted during the processing.
    #By including int as an argument, it lets the signal know to expect
    #an integer argument when emitting.
    updateProgress = QtCore.pyqtSignal(int,int,int)

    #You can do any extra things in this init you need, but for this example
    #nothing else needs to be done expect call the super's init
    def __init__(self,controller):
        QtCore.QThread.__init__(self)
	self.controller = controller

    #A QThread is run by calling it's start() function, which calls this run()
    #function in it's own "thread". 
    def run(self):
	for event in js_read.event_stream():#deadzone = DEADZONE):
		#print event.key,':',event.value
		if event.key == 'X1':
			self.updateProgress.emit(0,0,event.value)
		elif event.key == 'Y1':
			#print event.key,':',event.value
			self.updateProgress.emit(0,1,event.value)
		elif event.key == 'X2':
			#print event.key,':',event.value
			self.updateProgress.emit(1,0,event.value)
		elif event.key == 'Y2':
			#print event.key,':',event.value
			self.updateProgress.emit(1,1,event.value)

