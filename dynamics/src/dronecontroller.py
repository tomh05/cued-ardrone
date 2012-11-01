#!/usr/bin/env python

'''
dronecontroller.py
This program is used in testing dynamics of the drone, ie carrying out step response tests and collecting data for analysis.
'''

import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8

from time import time, sleep
import sys
from numpy import pi
import pickle

#from imageprocessor import imageprocessor

class dronecontroller:
	
	########## From measurements ###########
	# width = npx*altd*0.005149 mm         #       
	# height = npx*altd*0.004702 mm        #
	########################################

	def navdataCallback(self, msg):
		self.nd_logger(msg)
		
		
	def __init__(self):
		self.reftm = time()
		self.logstart = self.reftm + 6.0
		self.cmd_log = {'tm':[], 'tw':[]}
		self.nd_log = {'tm':[], 'nd':[], 'ph':[], 'th':[], 'ps':[], 'vx':[], 'vy':[], 'vz':[], 'al':[]}
		self.lasterr = (0.0,0.0)
		self.lastcmd = (0.0,0.0)
		self.seq = 0
		self.lastseq = 0			#used by main_procedure_callback for checking if the navdata is new
		self.mks_log = {'tm':[], 'coords':[], 'mids':[]}
		self.mksseq = 0
		self.lastmksseq = 0
		self.trackmid = 9
		self.misslimit = 12
		self.misstracktimes = self.misslimit
		self.noframelimit = 10
		self.noframetimes = self.noframelimit
		self.cmdinhibflag = False
		self.prinhibflag = False
		
		self.twist = Twist()
		self.cmdpub = rospy.Publisher('cmd_vel', Twist)
		self.landpub = rospy.Publisher('/ardrone/land', Empty)
		self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
		self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
		self.navdatasub = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		self.camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
		
		self.ref = {'al':1100, 'ps':0.0}
		self.error = {'al':[], 'ps':[]}
		self.pnotr = True
	
	def cleartwist(self):
		self.twist.linear.x= 0.0; self.twist.linear.y= 0.0; self.twist.linear.z= 0.0
		self.twist.angular.x= 0.0; self.twist.angular.y= 0.0; self.twist.angular.z= 0.0; 
	
	def nd_logger(self,msg):
		if (time() > self.logstart):
			self.nd_log['tm'].append(time()-self.reftm)
			self.nd_log['nd'].append(msg)
			
			self.nd_log['th'].append(msg.rotY)
			self.nd_log['ph'].append(msg.rotX)
			self.nd_log['ps'].append(msg.rotZ)
			self.nd_log['vx'].append(msg.vx)
			self.nd_log['vy'].append(msg.vy)
			self.nd_log['al'].append(msg.altd)
			
			self.error['al'].append(self.ref['al']-msg.altd)
			self.error['ps'].append(self.ref['ps']-msg.rotZ)
			self.seq = self.seq + 1
	
	def cmd_logger(self,twcmd):
		if (time() > self.logstart):
			self.cmd_log['tm'].append(time()-self.reftm)
			self.cmd_log['tw'].append(twcmd)
	
	def visioninfo_logger(self,coords, mids):
		if (time() > self.logstart):
			self.mks_log['tm'].append(time()-self.reftm)
			self.mks_log['coords'].append(coords)
			self.mks_log['mids'].append(mids)
			self.mksseq = self.mksseq + 1
			#print coords, mids
	
	def main_procedure(self):
		sleep(1)	#nb: sleep 0.3 is min necessary wait before you can publish. perhaps bc ros master takes time to setup publisher.
		self.cleartwist()
		self.cmdpub.publish(self.twist);
		self.camselectclient(1);
		#self.improc = imageprocessor(self)
		self.takeoffpub.publish(Empty()); print 'takeoff'
		while (time() < (self.logstart+2)):
			pass
		print 'start control'
		self.main_timer = rospy.Timer(rospy.Duration(1.0/15.0), self.main_timer_callback)
	
	def main_timer_callback(self,event):
		if (self.nd_log['al'][-1]>1500):
			self.landpub.publish(Empty())
		
		self.twist.linear.z = max(min(0.0013*self.error['al'][-1], 1.0), -1.0)
		self.twist.angular.z = max(min(self.error['ps'][-1]/150, 1.0), -1.0)
		#print self.nd_log['al'][-1]

		#endif height and yaw control are activated
		if (time() > self.logstart + 6 and time() < self.logstart + 7.5):
			print '-0.2'
			self.twist.linear.y=-0.5
		if (time() > self.logstart + 7.5 and time() < self.logstart + 9):
			print '0.2'
			self.twist.linear.y=0.5
		if (time() > self.logstart + 9 and time() < self.logstart + 10):
			print '-0.2'
			self.twist.linear.y=-0.5
		if (time() > self.logstart + 10):
			self.cleartwist()
			print 'land'
			self.landpub.publish(Empty())
			file1 = open('roll_0_5_altctrl.pkl','w')
			pickle.dump([self.nd_log, self.cmd_log],file1)
			file1.close()
			self.main_timer.shutdown()

		self.cmdpub.publish(self.twist)
		self.cmd_logger(self.twist)
		#print 'twist: \n', self.twist
			
			

def main(args):
	rospy.init_node('controller', anonymous=True)
	controller = dronecontroller()
	controller.main_procedure()
	rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
