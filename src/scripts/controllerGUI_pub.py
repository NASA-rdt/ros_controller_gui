#!/usr/bin/env python

import sys, time, cv2, numpy
from PyQt4 import QtGui, QtCore
from multiprocessing import Process, Queue
import CameraFeed as cf
import cameraQT as cam
import controllerQT as con
from listener import Listener

import rospy
from std_msgs.msg import Int32, String, Float64MultiArray, Float64

def scale(x, in_min, in_max, out_min, out_max):
	value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	value = min(value,out_max)
	value = max(value, out_min)
	return value

class rosProccess(QtCore.QThread):

	#This is the signal that will be emitted during the processing.
	#By including int as an argument, it lets the signal know to expect
	#an integer argument when emitting.
	getPose = QtCore.pyqtSignal(int,int,int,int,int,int)

	#You can do any extra things in this init you need, but for this example
	#nothing else needs to be done expect call the super's init
	def __init__(self,rosNode):
		QtCore.QThread.__init__(self)
		self.rosNode = rosNode
	def run(self):
		self.rosNode.main()
class SavedPosition(QtGui.QWidget):
	def __init__(self,name, position, parent=None):
		QtGui.QWidget.__init__(self, parent)
		self.name_root=name[0]
		if (len(name) > 0):
			self.name_extension = name[1]
		else:
			self.name_extension = ''
		self.name = self.name_root + self.name_extension
		self.position=position
		self.button = QtGui.QPushButton(self.name, self)
		self.button.clicked.connect(self.calluser)
	def calluser(self):
		info = 'SavedPosition_%s has position length: %d.' % (self.name,len(self.position)) 
		print(info)

class Communicate(QtCore.QObject):
    
    newPose = QtCore.pyqtSignal(Float64MultiArray)
    updateStick = QtCore.pyqtSignal(int, int, int)
    sendCommand = QtCore.pyqtSignal(int)
    sendImage = QtCore.pyqtSignal(numpy.ndarray)

class RoboGUI(QtGui.QWidget):
	
	def __init__(self):
		super(RoboGUI, self).__init__()
		self.initUI()
	
	def initUI(self):

		#ROS:
		
		self.pubCommands = rospy.Publisher('commands', Int32, queue_size=10)
		self.pubExecute = rospy.Publisher('execute', String, queue_size=10)
		self.pubGripperVal = rospy.Publisher('gripper_value', Float64, queue_size=10)
		self.pubPointEE = rospy.Publisher('ee_pose', Float64MultiArray, queue_size=10)
		self.pubJointVal = rospy.Publisher('jointVal', Int32, queue_size=10)
		#rospy.init_node('ControllerGUI', anonymous=True)
		self.savedPositions = []
		self.c = Communicate()


		self.layout = QtGui.QHBoxLayout()
		self.layoutRight = QtGui.QVBoxLayout()
#build top buttons

		buttonHome = QtGui.QPushButton('Home', self)
		buttonHome.clicked[bool].connect(self.goToPosition)
		buttonClear = QtGui.QPushButton('Clear', self)
		buttonClear.clicked[bool].connect(self.clearPosition)

		self.layoutTopButtons = QtGui.QGridLayout()
		self.layoutTopButtons.addWidget(buttonHome,0,0)
		#self.layoutTopButtons.addWidget(buttonSave,0,1)
		self.layoutTopButtons.addWidget(buttonClear,1,0)
		#self.layoutTopButtons.addWidget(self.saveName,1,1)

		self.topButtons = QtGui.QWidget()
		self.topButtons.setLayout(self.layoutTopButtons)
#build middle sliders

		sld_gripper = QtGui.QSlider(QtCore.Qt.Horizontal, self)
		sld_gripper.setFocusPolicy(QtCore.Qt.NoFocus)
		sld_gripper.setGeometry(10, 40, 300, 300)
		sld_gripper.valueChanged[int].connect(self.setGripper)

		sld_wrist = QtGui.QSlider(QtCore.Qt.Horizontal, self)
		sld_wrist.setFocusPolicy(QtCore.Qt.NoFocus)
		sld_wrist.setGeometry(10, 40, 300, 300)
		sld_wrist.valueChanged[int].connect(self.setWrist)

		self.gripperLabel = self.createLabel(self.tr("Gripper Value"))
		self.wristLabel = self.createLabel(self.tr("Wrist Roll"))
		self.layoutMiddleSliders = QtGui.QVBoxLayout()
		self.layoutMiddleSliders.addWidget(self.gripperLabel)
		self.layoutMiddleSliders.addWidget(sld_gripper)
		self.layoutMiddleSliders.addWidget(self.wristLabel)
		self.layoutMiddleSliders.addWidget(sld_wrist)
		#self.layoutMiddleSliders.addWidget(buttonSave,0,1)

		self.middleSliders = QtGui.QWidget()
		self.middleSliders.setLayout(self.layoutMiddleSliders)
#build left side
		self.leftLabel = self.createLabel(self.tr("Available Actions"))
		self.layoutLeft = QtGui.QVBoxLayout()
		self.layoutLeft.addWidget(self.leftLabel)
		self.layoutLeft.addWidget(self.topButtons)
		self.layoutLeft.addWidget(self.middleSliders)
		self.layoutMiddleSliders.addWidget(self.createLabel(self.tr("Controller Input")))
#controller image
		self.controllerImage = con.ControllerWidget()
		self.c.updateStick[int,int,int].connect(self.controllerImage.setValue)
		self.controllerImage.setGeometry(300,300,300,300)
#controller update function
		self.controllerProcess = con.ControllerCore(self.controllerImage)
		self.controllerProcess.updateProgress.connect(self.controllerImage.setValue)
		self.controllerProcess.start()	


		self.layoutLeft.addWidget(self.controllerImage)
		self.layoutLeft.addStretch(1)

		self.leftSide = QtGui.QWidget()
		self.leftSide.setLayout(self.layoutLeft)

#build right side
		self.actionLabel = self.createLabel(self.tr("Saved Positions"))
		self.buttonSave = QtGui.QPushButton('Save', self)
		self.buttonSave.clicked[bool].connect(self.savePosition)
        	self.saveName = QtGui.QLineEdit()
		label0 = QtGui.QLabel()
		label0.setFrameStyle(QtGui.QFrame.Panel | QtGui.QFrame.Raised)
		label0.setLineWidth(2)
		self.layoutRight = QtGui.QVBoxLayout()
		self.layoutRight.addWidget(self.actionLabel)
		self.layoutRight.addWidget(self.saveName)
		self.layoutRight.addWidget(self.buttonSave)
		self.layoutRight.addWidget(label0)
		self.layoutRight.addStretch(1)

		self.rightSide = QtGui.QWidget()
		self.rightSide.setLayout(self.layoutRight)
#build dividers

		label = QtGui.QLabel()
		label.setFrameStyle(QtGui.QFrame.Panel | QtGui.QFrame.Raised)
		label.setLineWidth(2)
		label2 = QtGui.QLabel()
		label2.setFrameStyle(QtGui.QFrame.Panel | QtGui.QFrame.Raised)
		label2.setLineWidth(2)
#build camViews
		self.layoutCams = QtGui.QVBoxLayout()
		print cam.findDevices()
		cams = cam.findDevices([1,2,3,4])
		self.camViews = []
		for camm in cams:
			camName = 'Video_'+str(camm)
			camLabel = self.createLabel(self.tr(camName))
			camView = cam.CameraWidget(cam.CameraDevice(camm))
			camView.setWidth(300)
			self.camViews.append(camView)
			self.layoutCams.addWidget(camLabel)
			self.layoutCams.addWidget(camView)

		self.camViewsW = QtGui.QWidget()
		self.camViewsW.setLayout(self.layoutCams)
#build layout
		self.layout.addWidget(self.leftSide)
		self.layout.addWidget(label)
		self.layout.addWidget(self.rightSide)
		self.layout.addWidget(label2)
		self.layout.addWidget(self.camViewsW)

		#elf.middleSliders = QtGui.QWidget()
		#self.middleSliders.setLayout(self.layoutMiddleSliders)


		#self.bottomImage = QtGui.QWidget()
		#self.topButtons.setLayout(self.layoutLeft)

        	self.setLayout(self.layout)
		self.setGeometry(200, 300, 1280, 640)
		self.setWindowTitle('Arm Control')
		self.changeStyle('Cleanlooks')
		self.show()
	def newPose(self, data):
		print 'got:',data
	def goToPosition(self, pressed):
		self.pubExecute.publish('test')
	def changeStyle(self, styleName):
		QtGui.QApplication.setStyle(QtGui.QStyleFactory.create(styleName))
		QtGui.QApplication.setPalette(QtGui.QApplication.style().standardPalette())
		#self.changePalette()

	def changePalette(self):
		if (self.useStylePaletteCheckBox.isChecked()):
			QtGui.QApplication.setPalette(QtGui.QApplication.style().standardPalette())
		else:
			QtGui.QApplication.setPalette(self.originalPalette)


	def getPosition(self):
		print 'generating fake joint values...'
		return [1,2,3,4]#want to return real values
	def addButton(self, name,layout=None, index = -1 ):
		print 'creating newButton'
		newPosition = SavedPosition(name,self.getPosition())
		print 'savedPositions:',len(self.savedPositions)
		if( layout is not None):
			if( index >= 0 ):
				layout.insertWidget(index,newPosition.button)
			else:
				layout.addWidget(newPosition.button)
		self.savedPositions.append(newPosition)
		return newPosition

	def createLabel(self, text):
		label = QtGui.QLabel(text)
		label.setAlignment(QtCore.Qt.AlignCenter)
		label.setMargin(2)
		label.setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Sunken)
		return label

	def setGripper(self, value):
		value = scale(value,0.0,100.0,0.0015,0.015)
		print "Setting gripper to %f" % (value)

	def setWrist(self, value):
		print "Setting wrist to %d" % (value)

	def savePosition(self, pressed):
		print "Saving current position..."
		name = self.saveName.text()
		extension = ''
		if( name != '' ):
			inc = 0
			for position in self.savedPositions:
				if name == position.name_root:
					inc+=1
			if(inc > 0):
				extension ='_'+str(inc)
			print 'field is: %s' % self.saveName.text()
		if( name == '' ):
			print 'empty field'
			name = "Saved"
			extension = '_' + str(len(self.savedPositions))
		self.addButton([name,extension],self.layoutRight,4)
	def clearPosition(self, pressed):
		print "Clearing saved positions..."
		for button in self.savedPositions:
			print "removing."
			self.layoutRight.removeWidget(button.button)
    			button.deleteLater()
    			button.button.deleteLater()
		self.savedPositions = []
	def something(self, pressed):
		if pressed:
			isPressed = "True"
		else:
			isPressed = "False"
		print "pressed: %s" % (isPressed)
def main():
	
	app = QtGui.QApplication(sys.argv)
	#app.aboutToQuit.connect(quit)
	ex = RoboGUI()
	sys.exit(app.exec_())


if __name__ == '__main__':
	main() 
