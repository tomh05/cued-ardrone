#!/usr/bin/env python

'''
HoverController.py
This program performs high level control of marker hovering: 
sets up transform trees, defines world coordinate, works out current pose and sets desired pose, and sends these to position_controller
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import tf

from time import time, sleep
import sys
from math import sin, cos, radians, pi
import pickle

from PositionController import PositionController

class HoverController:
	
	def __init__(self):
		self.reftm = time()
		self.cmdstart = self.reftm + 7.0
		
		self.twist = Twist()
		self.cmdpub = rospy.Publisher('cmd_vel', Twist)
		self.landpub = rospy.Publisher('/ardrone/land', Empty)
		self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
		self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
		self.camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
		
		self.tl = tf.TransformListener()
		self.gottf = False
		
	
	def cleartwist(self):
		self.twist.linear.x= 0.0; self.twist.linear.y= 0.0; self.twist.linear.z= 0.0
		self.twist.angular.x= 0.0; self.twist.angular.y= 0.0; self.twist.angular.z= 0.0; 
	

	def hover_procedure(self):
		sleep(1)	#nb: sleep 0.3 is min necessary wait before you can publish. perhaps bc ros master takes time to setup publisher.
		#self.pc = PositionController()		
		self.cleartwist()
		self.cmdpub.publish(self.twist);
		self.camselectclient(1);
		
		#self.takeoffpub.publish(Empty()); print 'takeoff'
		while (time() < (self.cmdstart)):
			pass

		self.desiposw=(0.0, 0.6, 1.7)
		print '*********** start control ***********'
		self.hover_timer = rospy.Timer(rospy.Duration(1.0/15.0), self.hover_timer_callback)
	

	def hover_timer_callback(self,event):
		
		if (time() > self.cmdstart + 1.0):	
			try:
				if (self.tl.frameExists('/4x4_100') and self.tl.canTransform('/4x4_100','/ardrone_base_link', rospy.Time(0))):
					(currposw, curroriw) = self.tl.lookupTransform('/4x4_100','/ardrone_base_link', rospy.Time(0))
					self.gottf = True
				else:
					self.gottf = False
					print 'cannot transform'
			except:
				self.gottf = False
				print 'transform exception'
			
			if self.gottf:
				print tf.transformations.euler_from_quaternion(curroriw)
		
			'''
			file1 = open('ssm-r-0-05','w')
			pickle.dump([self.nd_log, self.cmd_log],file1)
			file1.close()
			self.main_timer.shutdown()
			'''

			
def main(args):
	rospy.init_node('hovercontroller', anonymous=True)
	hc = HoverController()
	hc.hover_procedure()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
