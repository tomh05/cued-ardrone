#!/usr/bin/env python
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8

from time import time, sleep
import sys
from math import sin, cos, radians, pi
import pickle


class ssmtester:
	
	def __init__(self):
		self.reftm = time()
		self.logstart = self.reftm + 6.0
		self.cmdstart = self.reftm + 8.0
		self.cmd_log = {'tm':[], 'tw':[]}
		self.nd_log = {'tm':[], 'nd':[], 'ph':[], 'th':[], 'ps':[], 'vx':[], 'vy':[], 'vz':[], 'al':[]}
		
		self.ref = {'al':1100, 'ps':0.0}
		self.error = {'al':[], 'ps':[]}
		self.curcoord=(0.0,0.0)
		self.tarcoord=(0.0,0.0)
		self.pnotr = True
		
		self.twist = Twist()
		self.cmdpub = rospy.Publisher('cmd_vel', Twist)
		self.landpub = rospy.Publisher('/ardrone/land', Empty)
		self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
		self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
		self.navdatasub = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		self.camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)

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

	
	def cleartwist(self):
		self.twist.linear.x= 0.0; self.twist.linear.y= 0.0; self.twist.linear.z= 0.0
		self.twist.angular.x= 0.0; self.twist.angular.y= 0.0; self.twist.angular.z= 0.0; 
	
	
	def navdataCallback(self, msg):
		self.nd_logger(msg)
		try:
			dt=(msg.header.stamp - self.nd_log['nd'][-2].header.stamp).to_sec()
			dx=(msg.vx+self.nd_log['vx'][-2])*dt/2.0
			dy=(msg.vy+self.nd_log['vy'][-2])*dt/2.0
			yaw=(msg.rotZ+self.nd_log['ps'][-2])/2.0
			yawr=radians(yaw)
			du=dx*cos(yawr)-dy*sin(yawr)
			dv=dx*sin(yawr)+dy*cos(yawr)
			self.curcoord = (self.curcoord[0]+du, self.curcoord[1]+dv)
			#print yaw
			#print self.curcoord
		except:
			pass
			

	def nd_logger(self,msg):
		if (time() > self.logstart):
			self.nd_log['tm'].append(time()-self.reftm)
			self.nd_log['nd'].append(msg)
			
			self.nd_log['th'].append(msg.rotY)		# forw+ backw-
			self.nd_log['ph'].append(msg.rotX)		# left- right+
			self.nd_log['ps'].append(msg.rotZ)		# ccw+  cw-
			self.nd_log['vx'].append(msg.vx)
			self.nd_log['vy'].append(msg.vy)
			self.nd_log['al'].append(msg.altd)
			
			self.error['al'].append(self.ref['al']-msg.altd)
			self.error['ps'].append(self.ref['ps']-msg.rotZ)
			self.seq = self.seq + 1
		else:
			pass
	

	def cmd_logger(self,twcmd):
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
		while (time() < (self.cmdstart)):
			pass
		
		self.curcoord=(0.0, 0.0)
		self.tarcoord=(0.0, 0.0)
		print '********** t=8 *********** start control ***********'
		self.main_timer = rospy.Timer(rospy.Duration(1.0/15.0), self.main_timer_callback)
	

	def main_timer_callback(self,event):
		if (self.nd_log['al'][-1]>1500):
			self.landpub.publish(Empty())
		
		self.twist.linear.z = max(min(0.0013*self.error['al'][-1], 1.0), -1.0)
		self.twist.angular.z = max(min(self.error['ps'][-1]/150, 1.0), -1.0)
		#print self.nd_log['al'][-1]

		#endif height and yaw control are activated
		self.t1=6.5
		self.ro=0.2
		self.pi=0.2
		if (time() > self.cmdstart + 5 and time() < self.cmdstart + self.t1):
			#print self.ro
			self.twist.linear.x=self.pi

		if (time() > self.cmdstart + self.t1):
			self.cleartwist()
			print 'land', 'ro=', self.ro, 'pi=', self.pi
			self.landpub.publish(Empty())
			print self.curcoord
			file1 = open('ssm-r-0-05','w')
			pickle.dump([self.nd_log, self.cmd_log],file1)
			file1.close()
			self.main_timer.shutdown()

		self.cmdpub.publish(self.twist)
		self.cmd_logger(self.twist)
		#print 'twist: \n', self.twist
			
			
def main(args):
	rospy.init_node('ssmtester', anonymous=True)
	tester = ssmtester()
	tester.main_procedure()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
