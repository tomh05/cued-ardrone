#!/usr/bin/env python

'''
HoverController.py
This program performs high level control of marker hovering: 
sets up transform trees, defines world coordinate, works out current pose and sets desired pose, and sends these to position_controller
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata, ARMarkers
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
		self.posesub = rospy.Subscriber('/ar_pose_marker', ARMarkers, self.posesub_callback)
		
		self.tl = tf.TransformListener()
		self.gottf = False
		self.confmarker = False
		
		self.dpw=(0.0, 0.0, 0.0)
		self.dyw=0.0
		self.cpw=self.dpw
		self.cyw=self.dyw
		
		
	def cleartwist(self):
		self.twist.linear.x= 0.0; self.twist.linear.y= 0.0; self.twist.linear.z= 0.0
		self.twist.angular.x= 0.0; self.twist.angular.y= 0.0; self.twist.angular.z= 0.0; 
	

	def hover_procedure(self):
		sleep(1)	#nb: sleep 0.3 is min necessary wait before you can publish. perhaps bc ros master takes time to setup publisher.
		self.cleartwist()
		self.cmdpub.publish(self.twist);
		self.camselectclient(1);

		self.pc = PositionController(self.dpw, self.dyw, self.cpw, self.cyw)
		self.pc.refalon = True	
		
		self.takeoffpub.publish(Empty()); print 'takeoff'; 
		sleep(4)

		print '*********** start control ***********'
		self.hover_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.hover_timer_callback)
		sleep(2)
		self.pc.pc_timer_init()
		sleep(10)
		self.dpw=(0.7,-0.7,1.6)
		self.pc.dpw_handler(self.dpw)
		self.pc.refal_handler(1600)
		sleep(10)
		self.dpw=(-1.0,0.0,0.8)
		self.pc.dpw_handler(self.dpw)
		self.pc.refal_handler(800)
		sleep(10)
		self.dpw=(0.0,0.0,1.3)
		self.pc.refal_handler(1300)
		self.pc.dpw_handler(self.dpw)
		sleep(10)
		#self.landpub.publish(Empty()); print 'land'
	
	
	def posesub_callback(self, msg):
		# determines whether a confident pose was received and flags accordingly
		if msg.markers == []:
			self.confmarker = False
			#print 'no markers'
		else:
			if msg.markers[0].confidence > 75:
				#print 'found confident marker'
				self.confmarker = True
			else:
				#print 'bad marker'
				self.confmarker = False
				
	
	
	
	def hover_timer_callback(self,event):
		
		if True:	
			if (self.tl.canTransform('/4x4_94','/ardrone_base_link', rospy.Time(0))):
				tnow=rospy.Time.now()
				tpast=self.tl.getLatestCommonTime('/4x4_94','/ardrone_base_link')
				tdiff=rospy.Time.to_sec(tnow)-rospy.Time.to_sec(tpast)
				if (tdiff<0.1):
					try:
						(pos, ori) = self.tl.lookupTransform('/4x4_94','/ardrone_base_link', rospy.Time(0))
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
			
			if self.gottf and self.confmarker == True:
				ori2e = tf.transformations.euler_from_quaternion(ori)
				self.cyw = ori2e[2]+pi	#curroriw is in euler form! And +pi to align base_link frame with marker frame
				self.cpw = pos
				self.pc.cpw_cyw_handler(self.cpw, self.cyw)
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
