#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_demo')
import rospy

from drone_demo.msg import Navdata
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from time import time, sleep
import sys

import numpy as np
import pylab as pl

class alti_tester:
	def navdataCallback(self, msg):
		self.nd_logger(msg)
		
	def __init__(self, vzcmdb, vzdurb):
		self.reftm = time()
		self.cmd_log = {'tm':[], 'cmd':[]}
		self.nd_log = {'tm':[], 'ph':[], 'th':[], 'ps':[], 'vx':[], 'vy':[], 'vz':[], 'al':[]}
		self.twist = Twist()
		self.vzcmd = vzcmdb
		self.vzdur = vzdurb
		#print 'in constructor. vzcmd,vzdur =', self.vzcmd, self.vzdur
		self.cmdpub = rospy.Publisher('/cmd_vel', Twist)
		self.landpub = rospy.Publisher('/ardrone/land', Empty)
		self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
		self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
		self.navdatasub = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
	
	def nd_logger(self,msg):
		self.nd_log['tm'].append(time()-self.reftm)
		self.nd_log['th'].append(msg.rotY)
		self.nd_log['vz'].append(msg.vz*1000)
		self.nd_log['al'].append(msg.altd)
		#print msg.header.seq
	
	def cmd_logger(self,cmd):
		self.cmd_log['tm'].append(time()-self.reftm)
		self.cmd_log['cmd'].append(cmd*1000)
	
	def clear_twist(self):
		self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
		self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
	
	def test_seq(self):
		sleep(1)
		
		self.cmd_logger(0)
		self.takeoffpub.publish(Empty())	;print 'takeoff' #takeoff
		sleep(12); print '4'; sleep(1); print '3'; sleep(1); print '2'; sleep(1); print '1'; sleep(1)
		
		self.cmd_logger(0)
		self.twist.linear.z = self.vzcmd
		self.cmd_logger(self.vzcmd)
		self.cmdpub.publish(self.twist)		;print 'vzcmd' #set vzcmd
		sleep(self.vzdur)			#wait for vzdur
		
		self.cmd_logger(self.vzcmd)
		self.clear_twist()
		self.cmd_logger(0)
		self.cmdpub.publish(self.twist)		;print 'clear vz' #clear vz
		sleep(4)
		
		self.cmd_logger(0)
		self.landpub.publish(Empty())		;print 'land' #land
		sleep(1)
		
		if not raw_input('show and save?') == 'n':
			pl.xlabel('time (s)')
			pl.ylim(0,2400)
			pl.plot(self.cmd_log['tm'],self.cmd_log['cmd'], 'b-s')
			pl.plot(self.nd_log['tm'],self.nd_log['vz'], 'g-+')
			pl.plot(self.nd_log['tm'],self.nd_log['al'], 'r-+')
			pl.grid(True)
			pl.show()
			#print self.nd_log


def main(args):
	if len(args) != 3 or float(args[1]) == 0 or float(args[2]) == 0:
		vzcmda = 0.4; vzdura = 3
	else:
		vzcmda = float(args[1]); vzdura = float(args[2])
	print '\nalti_impulse_test.py\nvzcmd = %(a)f\nvzdur = %(b)f\n'%{'a':vzcmda, 'b':vzdura}
	rospy.init_node('alti_tester', anonymous=True)
	at = alti_tester(vzcmda, vzdura)
	at.test_seq()
	#rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
