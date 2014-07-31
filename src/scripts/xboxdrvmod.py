#!/usr/bin/env python

import dbus,time, threading

def led(num =13, slot = 0):
	if num > 15:
		num = 15
	if num < 0:
		num = 0
	try:
		bus = dbus.SessionBus()
		_slot = bus.get_object("org.seul.Xboxdrv", '/org/seul/Xboxdrv/ControllerSlots/%d' % slot)
		_slot.SetLed(num)
	except dbus.exceptions.DBusException as e:
		print 'Error setting led...\n',e

def rumble( left = 0, right = -1, slot = 0, pattern = None):
	if pattern is not None:
		print 'starting new thread'
		thr = threading.Thread(target = rumble_pattern, args = ( pattern, ) )
		thr.start()
		return
	if right < 0:
		right = left
	try:
		bus = dbus.SessionBus()
		_slot = bus.get_object("org.seul.Xboxdrv", '/org/seul/Xboxdrv/ControllerSlots/%d' % slot)
		print 'Rumble: (%d,%d).' % (left,right)
		_slot.SetRumble(left, right)
	except dbus.exceptions.DBusException as e:
		print 'Error setting rumble...\n',e

def rumble_pattern( pattern ):
	for step in range(len(pattern)):
		if len(pattern[step]) == 2:
			left = pattern[step][0]
			right = left
			wait = pattern[step][1]
			run = True
		elif len(pattern[step]) == 3:
			left = pattern[step][0]
			right = pattern[step][1]
			wait = pattern[step][2]
			run = True
		else:
			print 'invalid pattern format, (left,right,delay).'
			run = False
		if run:
			rumble(left,right)
			time.sleep(wait)

class pattern():
	def __init__(self,name = 'unnamed pattern'):
		self.name = name
		self.patt = []
	def add(self,state):
		if len(state) == 2 or len(state) == 3:
			self.patt.append(state)
	def clear(self):
		self.patt = []
	def run(self):
		print 'Running:',self.name
		rumble(pattern = self.patt)

if __name__ == '__main__':
	print 'Testing xboxdrvmod...'
	import time
	time.sleep(0.5)
	led(1)
	rumble(100)
	time.sleep(1)
	rumble()
