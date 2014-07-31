#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float64, Float64MultiArray, String

from xboxdrvmod import rumble, led


from PyQt4 import QtGui, QtCore


def map( x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

class Listener():
	def __init__(self,signal = None):
		self.THRESHOLD = 0.005
		self.MAX_OFFSET = 0.05
		self.MAX_RUMBLE = 250
		self.MIN_RUMBLE = 30
		#self.stopped = True;
		self.last = self.THRESHOLD
		self.savedPositions = [];
		self.pose = 0
		self.pub_goto = rospy.Publisher('ee_pose', Float64MultiArray)
		self.signal = signal

	def callback_execute(self,data):
		if data.data == "clearPose":
			self.savedPositions = []
			print 'cleared saved positions.'
		elif data.data == 'nextPose':
			print 'nextPose'
			self.pose += 1;
			if self.pose > len(self.savedPositions):
				self.pose = 0
			if self.pose < len(self.savedPositions):
				self.goTo(self.savedPositions[self.pose])
	def goTo(self,array):
		self.pub_goto.publish(Float64MultiArray(data=array))
	def callback_feedback(self,data):
		self.savedPositions.append(data.data)
		print 'have %d saved positions.' % (len(self.savedPositions))
		for pos in self.savedPositions:
			print pos
	def callback_offset(self,data):
	    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
		if data.data < self.THRESHOLD:
			if self.last > self.THRESHOLD:
				#print 'reset rumble'
				self.last = self.THRESHOLD
				rumble()
			return
		#print 'last:',last,'offset:',data.data
		if data.data > self.last:
			rospy.loginfo("Offset: %.4f",data.data)
			if self.last > self.MAX_OFFSET:
				self.last = self.MAX_OFFSET
			rumbleVal = int(map(self.last, \
				self.THRESHOLD , self.MAX_OFFSET , \
				self.MIN_RUMBLE , self.MAX_RUMBLE))
			self.last = self.last * 2
			#print 'rumbleU',rumbleVal,'lastt:',last
			rumble(rumbleVal)
		elif data.data < self.last/2:
			if self.last < self.THRESHOLD:
				self.last = self.THRESHOLD
			rumbleVal = int(map(self.last, \
				self.THRESHOLD , self.MAX_OFFSET , \
				self.MIN_RUMBLE , self.MAX_RUMBLE))
			self.last = self.last / 2
			#print 'rumbleD',rumbleVal
			rumble(rumbleVal)

	def main(self):

	    # in ROS, nodes are unique named. If two nodes with the same
	    # node are launched, the previous one is kicked off. The 
	    # anonymous=True flag means that rospy will choose a unique
	    # name for our 'talker' node so that multiple talkers can
	    # run simultaenously.
	    rospy.init_node('listener', anonymous=True)

	    rospy.Subscriber("ee_pose/offset", Float64, self.callback_offset)
	    rospy.Subscriber("ee_pose/feedback", Float64MultiArray, self.callback_feedback)
	    rospy.Subscriber("execute", String, self.callback_execute)

	    # spin() simply keeps python from exiting until this node is stopped
	    rospy.spin()
        
if __name__ == '__main__':
    listener = Listener()
    listener.main()
