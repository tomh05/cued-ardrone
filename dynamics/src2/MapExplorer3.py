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
from math import cos, sin
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
		self.img = np.asarray([])		# current img
		self.imglib = []
		self.templatelib = []
		
		self.dm = cv2.DescriptorMatcher_create('BruteForce')
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
		# --------------------------------------------------------------
		
		
		# 2. Start position control by starting pc_timer
		pc.pc_timer_init(); print '\n\n\nstart PC timer; enable control system'
		sleep(4)
		
		# 3. Hover and request to take template
		pc.pc_timer_shutdown(); print 'stop pc timer; hover'
		
		resp = self.capture_image_features(self.seq)
		kppt, desc, alt, img = self.handleFeatureResponse(resp)
		self.img = img.copy()
		
		for p in kppt:	# show captured features in file
			cv2.circle(img, tuple(p.astype(int)), 5, (255, 0, 0))
		cv2.circle(img, (320,180), 8, (0, 0, 255), thickness = 3)
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',img)
		
		self.analyseFeatures(kppt, desc, alt)	# main step for calculating pose and new template
		sleep(0.2)
		# --------------------------------------------------------------
		
		
		# 4. Restart position control, move left x meters
		cpw=(0,0,1); pc.cpw_handler(cpw)
		dpw=(0,0.4,1); pc.dpw_handler(dpw);
		pc.pc_timer_init(); print '\n\n\nstart PC timer; enable control system'
		sleep(5)
		
		# 5. Hover and request template
		pc.pc_timer_shutdown(); print 'stop pc timer; hover'
		
		self.seq+=1
		resp = self.capture_image_features(self.seq)
		kppt, desc, alt, img = self.handleFeatureResponse(resp)
		self.img = img.copy()
		
		for p in kppt:		# show captured features in file
			cv2.circle(img, tuple(p.astype(int)), 5, (255, 0, 0))
		cv2.circle(img, (320,180), 8, (0, 0, 255), thickness = 3)
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',img)
		
		self.analyseFeatures(kppt, desc, alt)	# main step for calculating pose and new template
		sleep(0.2)
		# --------------------------------------------------------------
		
				
		# 4. Restart position control, move left x meters
		cpw=(0,0,1); pc.cpw_handler(cpw)
		dpw=(0,0.4,1); pc.dpw_handler(dpw);
		pc.pc_timer_init(); print '\n\n\nstart PC timer; enable control system'
		sleep(5)

		# 5. Hover and request template
		pc.pc_timer_shutdown(); print 'stop pc timer; hover'
		
		self.seq+=1
		resp = self.capture_image_features(self.seq)
		kppt, desc, alt, img = self.handleFeatureResponse(resp)
		self.img = img.copy()
		
		for p in kppt:		# show captured features in file
			cv2.circle(img, tuple(p.astype(int)), 5, (255, 0, 0))
		cv2.circle(img, (320,180), 8, (0, 0, 255), thickness = 3)
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',img)
		
		self.analyseFeatures(kppt, desc, alt)	# main step for calculating pose and new template
		sleep(0.2)
		# --------------------------------------------------------------
		
				
		# 4. Restart position control, move left x meters
		cpw=(0,0,1); pc.cpw_handler(cpw)
		dpw=(0,0.4,1); pc.dpw_handler(dpw);
		pc.pc_timer_init(); print '\n\n\nstart PC timer; enable control system'
		sleep(5)

		# 5. Hover and request template
		pc.pc_timer_shutdown(); print 'stop pc timer; hover'
		
		self.seq+=1
		resp = self.capture_image_features(self.seq)
		kppt, desc, alt, img = self.handleFeatureResponse(resp)
		self.img = img.copy()
		
		for p in kppt:		# show captured features in file
			cv2.circle(img, tuple(p.astype(int)), 5, (255, 0, 0))
		cv2.circle(img, (320,180), 8, (0, 0, 255), thickness = 3)
		cv2.imwrite('showfeatures'+str(self.seq)+'.png',img)
		
		self.analyseFeatures(kppt, desc, alt)	# main step for calculating pose and new template
		sleep(0.2)
		
		
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
		
		p1 = self.templatelib[0].S
		p2 = self.templatelib[3].S
		distance = np.linalg.norm(p1-p2)
		print '\n\ndistance = ', distance
		
		#rospy.spin()
	
	def handleFeatureResponse(self, resp):
		"""Handles feature server's response and returns 
		kppt, desc, alt, img in the correct format.
		"""
		if resp.error == 0:		
			kppt = resolveFromFloat32MultiArray(resp.kppt)
			desc = resolveFromFloat32MultiArray(resp.desc)
			desc = desc.astype('float32')
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
		
		Returns 0 if normal exit. Returns -1 if error.
		"""
		if self.templatelib == []:
			print '\n\n\n current templatelib length: 0'
			print '-'*50
			
			# analyse features and collect elements for new template
			Q = np.eye(3)
			S = np.zeros([3,1])
			ntkppt, ntindices = self.extractTemplate(kppt, [])
			objc = self.calcObjcCoord(ntkppt, alt)
			objm = objc
			ntdesc = desc[ntindices]
			mid = 0
			tm = time()
			
			# construct new template and save
			t = Template(tm,ntkppt,ntdesc,objc,objm,Q,S,mid)
			self.templatelib.append(t)
			
			#~ print ntkppt, ntdesc
			print kppt.shape, desc.shape, 'kppt shape, desc shape'
			print ntkppt.shape, ntdesc.shape, objc.shape, objm.shape, 'new template: ntkppt, ntdesc, objc, objm shape'
			
			img = self.img.copy()
			for p in ntkppt:
				cv2.circle(img, tuple(p.astype(int)), 5, (0, 0, 0))
			cv2.circle(img, (320,180), 8, (0, 0, 255), thickness = 3)
			cv2.imwrite('template'+str(self.seq)+'.png',img)
			self.imglib.append(img)
			
			return 0
			
		else:
			# match features with all known templates; receive Rf, Tf, inliers for all templates.
			allpt = [x.kppt for x in self.templatelib]
			alldt = [x.desc for x in self.templatelib]
			allobjm = [x.objm for x in self.templatelib]
			nl = len(self.templatelib)
			print '\n\n\n current templatelib length: ', nl
			
			res = map(self.matchFeaturesEstimatePose,[kppt]*nl, [desc]*nl, allpt, alldt, allobjm)
			unzipped = zip(*res)
			allRf = unzipped[0]
			allTf = unzipped[1]
			allinliers = unzipped[2]
			
			# find the template with most inliers matched with current image features
			all_n_inliers = map(len, allinliers)	# number of matched inliers for all templates
			max_n_inliers = max(all_n_inliers)
			tmatch_index = all_n_inliers.index(max_n_inliers)
			print all_n_inliers, 'all numbers of inliers'
			print '-'*50
			print tmatch_index, max_n_inliers, 'tmatch_index, max number of inliers'
			print '-'*50
			
			threshold = 30
			if max_n_inliers < threshold:
				print 'Max number of matched features with current templates < ', threshold, '. Land.'
				self.landpub.publish(Empty())
				return -1
			
			#~ tmatch = self.templatelib[tmatch_index] 		# best template with most inliers
			Rf = allRf[tmatch_index]
			Tf = allTf[tmatch_index]
			inliers = allinliers[tmatch_index]
			
			# calculate Q, S
			Q = np.linalg.inv(Rf)
			S = -np.dot(Q,Tf)
			
			
			#~ print allRf, 'allRf'
			#~ print '-'*50
			#~ print allTf, 'allTf'
			#~ print '-'*50
			#~ print allinliers, 'allinliers'
			#~ print '-'*50
			print 'matched Rf:\n',Rf, '\nTf:\n', Tf, '\nQ:\n', Q, '\nS:\n', S
			print '-'*50
			#~ print tmatch, 'matched template'
			
			
			# extract a new template using kppt and inliers, get ntkppt, ntindices
			ntkppt, ntindices = self.extractTemplate(kppt, inliers)
			
			# calculate objc for ntkppt
			objc = self.calcObjcCoord(ntkppt, alt)
			
			# use Q, S to calculate objm from objc. Xm = Q.Xc + S
			Xc = np.transpose(objc)
			Q_Xc = Q.dot(Xc)
			nXc = objc.shape[0]		# number of objc points
			tileS = np.tile(S, (1, nXc))
			Xm = Q_Xc + tileS
			print Q_Xc.shape, nXc, 'Q_Xc, nXc'
			objm = np.transpose(Xm)
			objm = objm.astype('float32')
			
			# pick out the ntdesc's
			ntdesc = desc[ntindices]
			
			# construct new template and save
			mid = 0
			tm = time()
			t = Template(tm,ntkppt,ntdesc,objc,objm,Q,S,mid)
			self.templatelib.append(t)
			
			
			img = self.img.copy()
			for p in ntkppt:
				cv2.circle(img, tuple(p.astype(int)), 5, (0, 0, 0))
			cv2.circle(img, (320,180), 8, (0, 0, 255), thickness = 3)
			cv2.imwrite('template'+str(self.seq)+'.png',img)
			self.imglib.append(img)	
			
			return 0
			
	
	def matchFeaturesEstimatePose(self, pf, df, pt, dt, objm):
		""" keypoints feature, desc feature, keypoints template, desc template, obj coord in model frame.
		Returns Rf, Tf, inliers.
		"""
		# find match
		pfmatch, ptmatch, pfmatch_i, ptmatch_i = self.matchPoints(df, dt, pf, pt)
		# estimate pose
		objm_match = objm[ptmatch_i,:]
		rvec, tvec, inliers = cv2.solvePnPRansac(objm_match, pfmatch, \
		self.cameraMatrix, self.distCoeffs)
		# flatten
		rmat = cv2.Rodrigues(rvec)[0]
		rot = transformations.euler_from_matrix(rmat)[2]
		Rf = np.array([[cos(rot), -sin(rot), 0],[sin(rot),cos(rot),0],[0,0,1]])
		#~ print Rf
		Tf = tvec; Tf[2] = 0
		#~ print '\n', rvec, 'rvec\n', tvec, 'tvec\n'
		if inliers == None:
			inliers = np.array([])
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
		return i1_pts, i2_pts, i1_indices, i2_indices
	
	
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
	
	
	def extractTemplate(self, kppt, inliers):
		"""Selects a subset of the image keypoints to form a template.
		Uses inliers of current match to select a new template in image.
		returns new template's keypoints and indices in old kppt.
		"""
		ntkppt = []		# new template keypoints
		ntindex = []	# new template indices (of kppt)
		
		#~ for i in range(kppt.shape[0]):
			#~ if abs(kppt[i][0]-180)<140 and abs(kppt[i][1]-320)<250)):
				#~ ntkppt.append(kppt[i])
				#~ ntindex.append(i)
				
		c = zip(*[(i, x) for i, x in enumerate(list(kppt)) \
		 if abs(x[0]-320)<270 and abs(x[1]-180)<150])
		try:
			ntkppt = np.array(c[1])
			ntindex = list(c[0])
		except:
			print 'Error occurred when extracting new template - no features for template found. Land.'
			self.landpub.publish(Empty())
		return ntkppt, ntindex
	
	
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
	
	
