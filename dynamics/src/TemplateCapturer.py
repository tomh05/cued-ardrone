#!/usr/bin/env python

'''
templateCapturer.py
Test frame to frame image capturing, feature matching and template creating.
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
import cv2 
import cv
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
import sensor_msgs.msg
import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
import math
from time import time, sleep

from PositionController import PositionController

class TemplateCapturer:
	def __init__(self):
		self.frameskip = 0
		self.calibrated = False
		self.fd = cv2.FeatureDetector_create('SIFT')
		self.de = cv2.DescriptorExtractor_create('SIFT')
		self.dm = cv2.DescriptorMatcher_create('BruteForce')
		self.capture_count = 0
		self.capture_flag = 0
		
		self.img_pub = rospy.Publisher("/template_track/img",Image)
		self.corner_pub = rospy.Publisher("/template_track/corners", Float32MultiArray)
		
		self.published_no = 0
		
		self.connect()
	    
	def connect(self):
		rospy.init_node('template_capturer')
		self.tf = tf.TransformListener()
		rospy.Subscriber('/ardrone/bottom/image_raw',Image,self.imgproc)
		rospy.Subscriber('/ardrone/bottom/camera_info',sensor_msgs.msg.CameraInfo, self.setCameraInfo)
		rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		rospy.spin()
	
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
			print 'capture_flag up. alt, rotx, roty = ', navd.altd, navd.rotX, navd.rotY
			print 'capture_count = ', self.capture_count
			self.capture_flag = 1
		
	def imgproc(self, d):
		"""Converts the ROS published image to a cv2 numpy array
		and passes to FeatureTracker"""
		self.time_now = d.header.stamp
		if self.capture_flag == 1:
			self.capture_flag = 0
			self.capture_count += 1
			# ROS to cv image
			bridge = CvBridge()
			cvimg = bridge.imgmsg_to_cv(d,"bgr8")
			# cv to cv2 numpy array image
			npimg = np.asarray(cvimg)
			cv2.imwrite('goodcapture'+str(self.capture_count)+'.png',npimg)
			# Pass to FeatureTracker
			# self.manager(npimg)
			print 'captured image. capture_count = ', self.capture_count
		
	def setCameraInfo(self, ci):
		"""Converts the ROS published camera info into numpy arrays and 
		stores in FeatureTracker"""
		if not self.calibrated:
			if len(ci.D) == 0:
				return
			self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
			self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
			self.distCoeffs = np.array([ci.D], dtype=np.float32)
			self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
			self.calibrated = True    
			print "Calibration Initialised"

	


if __name__=='__main__':
	template_capturer = TemplateCapturer()
