#!/usr/bin/env python

import roslib; roslib.load_manifest('dynamics')

from dynamics.srv import *
import rospy
from dynamics.msg import Navdata
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
import cv2
import cv
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
import sensor_msgs.msg
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import Path
import math
import pickle
from time import time, sleep

class ImageCapturer:
	
	def __init__(self):
		self.loadCameraInfo()
		rospy.init_node('capture_image_features_server')
		rospy.Subscriber('/ardrone/bottom/image_raw',Image,self.imgproc)
		rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		
		self.bridge = CvBridge()
		self.fd = cv2.FeatureDetector_create('SIFT')
		self.de = cv2.DescriptorExtractor_create('SIFT')
		self.dm = cv2.DescriptorMatcher_create('BruteForce')
		
		self.capture_count = 0
		self.capture_ok = False
		self.capture_request = False
		self.timeout_value = 10		# wait 10 seconds for capture (need both capture_ok and capture_request on)
		
		self.kppt = Float32MultiArray()
		self.desc = Float32MultiArray()
		self.alt = -1000
		self.t_latest_capture = time()-10
		
		s = rospy.Service('CaptureImageFeatures', CaptureImageFeatures, self.handleCaptureImageFeatures)
		print "capture_image_features_server.py: Server ready."
		rospy.spin()
		
	def handleCaptureImageFeatures(self, req):
		print "capture_image_features_server.py: Capture image features request Received."
		self.capture_request = True
		treq = time()
		while time()-self.t_latest_capture > 0.4:
			# print 'capture_image_features_server.py: waiting for feature captures'
			# if no updated image captures for 1 second, return with error
			if time()-treq > self.timeout_value:
				print 'Timeout. ', self.timeout_value, ' seconds'
				self.capture_request = False
				return CaptureImageFeaturesResponse(1,Float32MultiArray(),Float32MultiArray(),-1000)
		
		# while loop exits with met condition, capture successful
		print 'time to capture = ', time()-treq
		self.capture_request = False
		kppt = self.kppt
		desc = self.desc
		alt = self.alt
		return CaptureImageFeaturesResponse(0, kppt, desc, alt)
	
	def navdataCallback(self, msg):
		self.check_capture(msg)
		#print 'navdata callback'
		
	def check_capture(self,navd):
		"""Check if capture_ok flag should be set to tell imgproc to 
		capture and save image for feature extraction"""
		alt_ok = 0
		ang_ok = 0
		vel_ok = 0
		if abs(navd.altd - 1000) < 100:
			alt_ok = 1
		if abs(navd.rotX) < 0.4 and abs(navd.rotY) < 0.4:
			ang_ok = 1
		if abs(navd.vx) < 50 and abs(navd.vy) < 50:
			vel_ok = 1
			
		if alt_ok == 1 and ang_ok ==1 and vel_ok ==1:
			self.alt = navd.altd
			self.capture_ok = True
		else:
			#~ if self.capture_request == True:
				#~ print 'alt, ang, vel: \n', alt_ok, ang_ok, vel_ok
			self.capture_ok = False
		
	def imgproc(self, d):
		"""Converts the ROS published image to a cv2 numpy array
		and passes to FeatureTracker"""
		self.time_now = d.header.stamp
		if self.capture_ok == True and self.capture_request == True:
			self.capture_request = False
			self.capture_count += 1
			# ROS to cv image to cv2 numpy array image
			bridge = CvBridge()
			cvimg = bridge.imgmsg_to_cv(d,"bgr8")
			img = np.asarray(cvimg)
			print 'captured image. capture_count = ', self.capture_count
			
			# capture image, extract features (keypoints and descriptors), undistort keypoint points (kppt)
			img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			img_pts = self.fd.detect(img)
			img_kp, img_desc = self.de.compute(img, img_pts)
			img_kppt = np.array(list(x.pt for x in img_kp))
			img_kppt = self.myUndistortPoints(img_kppt)
			
			# mould kppt and desc into Float32MultiArray messages
			self.kppt = self.toFloat32MultiArray(img_kppt)
			self.desc = self.toFloat32MultiArray(img_desc)
			
			self.t_latest_capture = time()
			
			#capture_image_featuresResponse(np.zeros(2),np.ones(2))
	
	def myUndistortPoints(self, pts):
		# Undistort matched points
		n = pts.size/2
		undist_pts = cv2.undistortPoints(pts.reshape([n,1,2]), self.cameraMatrix, self.distCoeffs)
		undist_pts = undist_pts.reshape([n,2])
		# 'Un-normalize' points
		undist_pts[:,0] = undist_pts[:,0]*self.fx+self.u0
		undist_pts[:,1] = undist_pts[:,1]*self.fy+self.v0
		return undist_pts
		
	def toFloat32MultiArray(self, inputarray):
		msg = Float32MultiArray()
		inputarray_dim = np.shape(inputarray)
		msg.layout.dim.append(MultiArrayDimension('nrows, ncols', inputarray_dim[0], inputarray_dim[1]))
		inputarray_aslist=list(np.reshape(inputarray,np.size(inputarray)))
		msg.data = inputarray_aslist
		return msg

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
		print 'capture_image_features_server.py: Camera info loaded.'

if __name__ == "__main__":
	ImageCapturer()
