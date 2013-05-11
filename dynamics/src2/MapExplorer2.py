#!/usr/bin/env python

'''
MapExplorer.py
Uses PositionController.py and issues orders of destinations to the
drone, in an attempt to map out the world.
'''

import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect, CaptureImageFeatures
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import Path
import cv2

import math
import pickle
import numpy as np
from time import time, sleep
from PositionController2 import PositionController

	
def resolveFromFloat32MultiArray(msg):
	nrows = msg.layout.dim[0].size
	ncols = msg.layout.dim[0].stride
	array = np.reshape(msg.data, [nrows, ncols])
	return array

class MapExplorer2:
		
	def __init__(self):
		self.cmdpub = rospy.Publisher('cmd_vel', Twist)
		rospy.init_node('map_explorer')
		self.landpub = rospy.Publisher('/ardrone/land', Empty)
		self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
		self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
		#~ rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		self.camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
		
		# Set up capture_feature proxy.
		# Initialize lists for storing found template keypoints and template descriptors
		self.capture_image_features = rospy.ServiceProxy('CaptureImageFeatures', CaptureImageFeatures)
		self.seq = 0;
		self.tempskppt = []		# stores keypoints for found templates
		self.tempsdesc = []		# stores descriptors for found templates
		self.kppt = []	# current kp
		self.desc = []	# current desc
		self.alt = -1000
		self.img = np.asarray([])		# current img
		sleep(1)
		
		# make sure cmd_vel topic is clear and drone is reset
		self.resetpub.publish(Empty())
		sleep(1)
		self.resetpub.publish(Empty())
		self.cmdpub.publish(Twist())
		
		self.camselectclient(1); print 'select bottom camera'
		self.dpw=(0,0,1)
		self.dyw=0
		self.cpw=self.dpw
		self.cyw=self.dyw
		self.good_to_capture = False
		self.run()
	
	def run(self):
		# 1. Take off. Instantiate PositionController
		self.takeoffpub.publish(Empty()); print 'takeoff'
		sleep(4)
		pc=PositionController(self.dpw,self.dyw,self.cpw,self.cyw, 1.0/3000, -0.010, -0.0002); print 'construct position controller'
		sleep(1)
		
		# 2. Start position control by starting pc_timer
		pc.pc_timer_init(); print 'start PC timer; enable control system'
		sleep(4)
		
		# 3. Hover and request to take template
		pc.pc_timer_shutdown(); print 'stop pc timer; hover'
		
		resp = self.capture_image_features(self.seq)
		self.handleCapturedFeatures(resp)
		for p in self.kppt:
			cv2.circle(self.img, tuple(p.astype(int)), 5, (255, 0, 0))
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',self.img)
		
		sleep(0.2)
		
		# 4. Restart position control, move left x meters
		cpw=(0,0,1); pc.cpw_handler(cpw)
		dpw=(0,0.4,1); pc.dpw_handler(dpw);
		pc.pc_timer_init(); print 'start PC timer; enable control system'
		sleep(5)
		
		# 5. Hover and request template
		pc.pc_timer_shutdown(); print 'stop pc timer; hover'
		
		self.seq+=1
		resp = self.capture_image_features(self.seq)
		self.handleCapturedFeatures(resp)
		for p in self.kppt:
			cv2.circle(self.img, tuple(p.astype(int)), 5, (0, 0, 255))
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',self.img)
				
		sleep(0.2)
		
		# 6. Match template
		
		
		#~ # reset position and move right x meters
		#~ cpw=(0,0.4,1); pc.cpw_handler(cpw)
		#~ dpw=(0,0,1); pc.dpw_handler(dpw)
		#~ pc.pc_timer_init(); print 'start PC timer; enable control system'
		#~ sleep(5)
		#~ 
		#~ # hover and time to match template
		#~ pc.pc_timer_shutdown(); print 'stop pc timer; hover'
		#~ sleep(1)
		#~ pass
		
		# land
		self.landpub.publish(Empty()); print 'finished - land'
		
		#rospy.spin()
	
	def handleCapturedFeatures(self, resp):
		"""Process feature server's response and update 
		self.kppt, self.desc, self.alt, self.img.
		"""
		if resp.error == 0:
			try:				
				self.kppt = resolveFromFloat32MultiArray(resp.kppt)
				self.desc = resolveFromFloat32MultiArray(resp.desc)
				#~ self.tempskppt.append(self.kppt)
				#~ self.tempsdesc.append(self.desc)
				self.alt = resp.alt
				
				# convert 2d matrix into 3 channel BGR and save in self.img
				img2dmat = resolveFromFloat32MultiArray(resp.img)
				_r = img2dmat.shape[0]; _c = img2dmat.shape[1]
				self.img = np.reshape(img2dmat, [_r, _c/3, 3])
				
			except:
				print 'handleCapturedFeatures exception'
		print resp.alt
	
	
	def matchToKnownTemplates(self):
		"""Uses self.kppt, self.desc, self.alt, and self.templateLib, 
		to determine with of the known templates in self.templateLib 
		matches the current captured frame of features. 
		Calculates transform from world/model frame to current camera 
		frame.
		"""
	
	def extractTemplate(self, kppt):
		"""Selects a subset of the image keypoints to form a template.
		Calculates obj coordinates of the keypoints (in model frame).
		"""
		return np.array(list(x for x in kppt if abs(x[0]-180)<140 and abs(x[1]-320)<250)
		
			
	def navdataCallback(self, msg):
		self.check_capture(msg)

	def check_capture(self,navd):
		"""Check if capture flag should be set to tell imgproc to 
		capture and save image for feature extraction
		"""
		alt_ok = 0
		ang_ok = 0
		vel_ok = 0
		if abs(navd.altd - 1000) < 50:
			alt_ok = 1
		if abs(navd.rotX) < 0.1 and abs(navd.rotY) < 0.1:
			ang_ok = 1
		if abs(navd.vx) < 50 and abs(navd.vy) < 50:
			vel_ok = 1
			
		if alt_ok == 1 and ang_ok ==1 and vel_ok ==1:
			self.good_to_capture = True
		
	def captureImage(self):
		while self.good_to_capture == False:
			pass
		
		self.good_to_capture = False

if __name__=='__main__':
	map_explorer2=MapExplorer2()
	
	
