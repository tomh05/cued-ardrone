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
from tf import transformations
import cv2

import math
import pickle
import numpy as np
from time import time, sleep
from PositionController2 import PositionController
from Template import *

	
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
		self.camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
		#~ rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		
		# Set up capture_feature proxy.
		# Initialize template library for storing templates
		self.capture_image_features = rospy.ServiceProxy('CaptureImageFeatures', CaptureImageFeatures)
		self.seq = 0;
		#~ self.tempskppt = []		# stores keypoints for found templates
		#~ self.tempsdesc = []		# stores descriptors for found templates
		#~ self.kppt = []	# current kp
		#~ self.desc = []	# current desc
		#~ self.alt = -1000
		#~ self.img = np.asarray([])		# current img
		self.templatelib = []
		
		self.loadCameraInfo()
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
		
	
	def loadCameraInfo(self):
		fh = open('CameraInfo.pickle', 'r')
		camdict = pickle.load(fh)
		fh.close()
		self.cameraMatrix = camdict['cameraMatrix']
		self.inverseCameraMatrix = camdict['inverseCameraMatrix']
		self.distCoeffs = camdict['distCoeffs']
		self.P = camdict['P']
		self.fx = self.cameraMatrix[0][0]
		self.u0 = self.cameraMatrix[0][2]
		self.fy = self.cameraMatrix[1][1]
		self.v0 = self.cameraMatrix[1][2]
	
	
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
		kppt, desc, alt, img = self.handleFeatureResponse(resp)
		for p in kppt:
			cv2.circle(img, tuple(p.astype(int)), 5, (255, 0, 0))
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',img)
		self.analyseFeatures(kppt, desc, alt)
		
		
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
		kppt, desc, alt, img = self.handleFeatureResponse(resp)
		for p in kppt:
			cv2.circle(img, tuple(p.astype(int)), 5, (255, 0, 0))
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',img)				
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
	
	def handleFeatureResponse(self, resp):
		"""Handles feature server's response and returns 
		kppt, desc, alt, img in the correct format.
		"""
		if resp.error == 0:		
			kppt = resolveFromFloat32MultiArray(resp.kppt)
			desc = resolveFromFloat32MultiArray(resp.desc)
			#~ self.tempskppt.append(self.kppt)
			#~ self.tempsdesc.append(self.desc)
			alt = resp.alt
			
			# convert 2d matrix into 3 channel BGR and save in self.img
			img2dmat = resolveFromFloat32MultiArray(resp.img)
			_r = img2dmat.shape[0]; _c = img2dmat.shape[1]
			img = np.reshape(img2dmat, [_r, _c/3, 3])
	
			print resp.alt
			return kppt, desc, alt, img
		
		else:
			print 'Error occurred when handling feature server response. Land.'
			self.landpub.publish(Empty())
			return -1, -1, -1, -1
	
	
	def analyseFeatures(self, kppt, desc, alt):
		"""Uses kppt, desc, alt, and self.templatelib.
		> Matches kppt/desc with templatelib, usesransac to estimate pose, 
		selects the most matching template with most inliers.
		> Quote R, T (coord xform from model to camera, points to m frame) 
		from ransac, flatten (make 2d) to Rf, Tf, calculate Q, S (xform 
		from camera to model, points to c frame).
		> Calculates objc, using alt. Calculates objm, using Q, S.
		> Choose a new template, save in self.templatelib.
		"""
		if self.templatelib == []:
			pass
			
		else:
			pass
	
	def matchFeaturesEstimatePose(self, pf, df, pt, dt, objm):
		""" keypoints feature, desc feature, keypoints template, desc template, obj coord in model frame.
		Returns Rf, Tf, inliers.
		"""
		# find match
		pfmatch, ptmatch = self.matchPoints(df, dt, pf, pt)
		# estimate pose
		rvec, tvec, inliers = cv2.solvePnPRansac(objm, pfmatch, \
		self.cameraMatrix, self.distCoeffs)
		# flatten
		rmat = cv2.Rodrigues(rvec)[0]
		rot = transformations.euler_from_matrix(rmat)[2]
		Rf = np.array([[cos(rot), -sin(rot), 0],[sin(rot),cos(rot),0],[0,0,1]])
		print Rf
		Tf = tvec; Tf[2] = 0
		return Rf, Tf, inliers
		
	
	def matchPoints(self, img_desc, img2_desc, i1_kppt, i2_kppt):
		# Match features
		matches = self.dm.match(img_desc, img2_desc)
		matches2 = self.dm.match(img2_desc, img_desc)
		
		# Produce ordered arrays of paired points
		i1_indices = list(x.queryIdx for x in matches)
		i2_indices = list(x.trainIdx for x in matches)
		i2_indices2 = list(x.queryIdx for x in matches2) 
		i1_indices2 = list(x.trainIdx for x in matches2)
		
		# Find pairing that are consistent in both dirs
		comb1 = set(zip(i1_indices, i2_indices))
		comb2 = set(zip(i1_indices2, i2_indices2))
		comb = list(comb1.intersection(comb2))
		comb = zip(*list(comb))
		i1_indices = comb[0]
		i2_indices = comb[1]
		
		# Collect matched points from indices
		kppt1_array = np.array(list(x for x in i1_kppt))
		kppt2_array = np.array(list(x for x in i2_kppt))
		i1_pts = kppt1_array[i1_indices,:]
		i2_pts = kppt2_array[i2_indices,:]
		i1_pts = i1_pts.astype('float32')
		i2_pts = i2_pts.astype('float32')		
		return i1_pts, i2_pts
	
	
	def calcObjcCoord(self, kppt, Zc):
		"""Calculate Nx3 object coordinates for template points
		"""
		#~ Zc = 1000 		# assume 1 meter hover height
		Xc = Zc/self.fx*(kppt[:,0] - self.u0)
		Yc = Zc/self.fy*(kppt[:,1] - self.v0)
		kpcoord = np.append(Xc.reshape([len(Xc),1]), Yc.reshape([len(Yc),1]), axis=1)
		kpcoord = np.append(kpcoord, np.zeros([len(Xc), 1]), axis=1)
		kpcoord = kpcoord.astype('float32')
		return kpcoord
	
	
	def extractTemplate(self, kppt):
		"""Selects a subset of the image keypoints to form a template.
		Calculates obj coordinates of the keypoints (in model frame).
		"""
		return np.array(list(x for x in kppt if abs(x[0]-180)<140 and abs(x[1]-320)<250))
	
	
	#~ def navdataCallback(self, msg):
		#~ self.check_capture(msg)
#~ 
	#~ def check_capture(self,navd):
		#~ """Check if capture flag should be set to tell imgproc to 
		#~ capture and save image for feature extraction
		#~ """
		#~ alt_ok = 0
		#~ ang_ok = 0
		#~ vel_ok = 0
		#~ if abs(navd.altd - 1000) < 50:
			#~ alt_ok = 1
		#~ if abs(navd.rotX) < 0.1 and abs(navd.rotY) < 0.1:
			#~ ang_ok = 1
		#~ if abs(navd.vx) < 50 and abs(navd.vy) < 50:
			#~ vel_ok = 1
			#~ 
		#~ if alt_ok == 1 and ang_ok ==1 and vel_ok ==1:
			#~ self.good_to_capture = True
		#~ 
	#~ def captureImage(self):
		#~ while self.good_to_capture == False:
			#~ pass
		#~ 
		#~ self.good_to_capture = False



if __name__=='__main__':
	map_explorer2=MapExplorer2()
	
	
