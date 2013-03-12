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
from std_msgs.msg import Float32MultiArray
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
		
		self.capture_count = 0
		self.capture_ok = False
		self.capture_request = False
		
		s = rospy.Service('capture_image_features', capture_image_features, self.handle_capture_image_features)
		print "capture_image_features_server.py: Server ready."
		rospy.spin()
		
	def handle_capture_image_features(self, req):
		print "Handling capture image features request."
		kppt = Float32MultiArray(); kppt.data = np.zeros(2);
		desc = Float32MultiArray(); desc.data = np.ones(2);
		return capture_image_featuresResponse(kppt, desc)

		self.capture_request = True
		
	def navdataCallback(self, msg):
		self.check_capture(msg)
		
	def check_capture(self,navd):
		"""Check if capture flag should be set to tell imgproc to 
		capture and save image for feature extraction"""
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
			print 'capture_ok. alt, rotx, roty = ', navd.altd, navd.rotX, navd.rotY
			self.capture_ok = True
		else:
			self.capture_ok = False
		
	def imgproc(self, d):
		"""Converts the ROS published image to a cv2 numpy array
		and passes to FeatureTracker"""
		self.time_now = d.header.stamp
		if self.capture_ok == True and self.capture_request == True:
			self.capture_request = False
			self.capture_count += 1
			# ROS to cv image
			bridge = CvBridge()
			cvimg = bridge.imgmsg_to_cv(d,"bgr8")
			# cv to cv2 numpy array image
			npimg = np.asarray(cvimg)
			print 'captured image. capture_count = ', self.capture_count
			#capture_image_featuresResponse(np.zeros(2),np.ones(2))
		

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
