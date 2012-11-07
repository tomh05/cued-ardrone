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
		self.cleartwist()
		self.cmdpub.publish(self.twist);
		self.camselectclient(1);
		
		self.pc = PositionController()		
		
		self.takeoffpub.publish(Empty()); print 'takeoff'
		sleep(4)

		self.desiposw=(0.0, 0.0, 0.0)
		print '*********** start control ***********'
		self.hover_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.hover_timer_callback)
		sleep(2)
		self.pc.pc_timer_init()
		sleep(10)
		self.desiposw=(0.7,0.0,0.0)
		self.pc.dpw_handler(self.desiposw)
		sleep(10)
		self.desiposw=(0.0,-0.8,0.0)
		self.pc.dpw_handler(self.desiposw)
		sleep(10)
		self.desiposw=(0.0,0.0,0.0)
		self.pc.dpw_handler(self.desiposw)
		sleep(10)
		self.desiposw=(-1.0,-0.3,0.0)
		self.pc.dpw_handler(self.desiposw)
		sleep(10)
		self.desiposw=(0.0,0.0,0.0)
		self.pc.dpw_handler(self.desiposw)
		sleep(10)
		self.landpub.publish(Empty()); print 'land'
	

	def hover_timer_callback(self,event):
		
		if True:	
			if (self.tl.canTransform('/4x4_100','/ardrone_base_link', rospy.Time(0))):
				tnow=rospy.Time.now()
				tpast=self.tl.getLatestCommonTime('/4x4_100','/ardrone_base_link')
				tdiff=rospy.Time.to_sec(tnow)-rospy.Time.to_sec(tpast)
				if (tdiff<0.1):
					try:
						(pos, ori) = self.tl.lookupTransform('/4x4_100','/ardrone_base_link', rospy.Time(0))
						self.gottf = True
					except:
						self.gottf = False
						#print 'error: transform exception'
				else:
					self.gottf = False
					#print 'error: latest tf common time > 0.1 seconds ago'
			else:
				self.gottf = False
				#print 'error: cantransform = False'
			
			if self.gottf:
				ori2e = tf.transformations.euler_from_quaternion(ori)
				curroriw = (ori2e[0], ori2e[1], ori2e[2]+pi)	#curroriw is in euler form! And +pi to align base_link frame with marker frame
				currposw = pos
				self.pc.pose_handler(currposw, curroriw, self.desiposw)
				#print currposw
				#print curroriw
				
			else:
				pass
				
			'''
			file1 = open('ssm-r-0-05','w')
			pickle.dump([self.nd_log, self.cmd_log],file1)
			file1.close()
			self.main_timer.shutdown()
			'''

			
def main(args):
	rospy.init_node('hovercontroller', anonymous=True)
	hc = HoverController()
	sleep(0.5)
	hc.hover_procedure()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
