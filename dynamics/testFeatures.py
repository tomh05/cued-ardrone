#!/usr/bin/env python

import roslib; roslib.load_manifest('dynamics')
import rospy

import cv2 
import cv
import numpy as np
from sensor_msgs.msg import Image
import matplotlib.pyplot as plot
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
import math
from time import time, sleep
import os
import pickle
from PositionController import PositionController


def stackImagesVertically(top_image, bottom_image):
	if len(top_image.shape)==2:
		# Get image dimensions
		h1, w1 = top_image.shape[:2]
		h2, w2 = bottom_image.shape[:2]
		# Create an empty array that is bounding box size of image stack
		stacked = np.zeros((h1+h2, max(w1,w2)), np.uint8)
		# Drop in the top_image
		stacked[:h1, :w1] = top_image
		# Drop in the bottom_image
		stacked[h1:h1+h2, :w2] = bottom_image
		return stacked
	else:
		# Get image dimensions
		h1, w1, d1 = top_image.shape[:3]
		h2, w2, d2 = bottom_image.shape[:3]
		# Create an empty array that is bounding box size of image stack
		stacked = np.zeros((h1+h2, max(w1,w2), max(d1,d2)), np.uint8)
		# Drop in the top_image
		stacked[:h1, :w1, :d1] = top_image
		# Drop in the bottom_image
		stacked[h1:h1+h2, :w2, :d2] = bottom_image
		return stacked


class Tester:
	def __init__(self):
		self.fd = cv2.FeatureDetector_create('SIFT')
		self.de = cv2.DescriptorExtractor_create('SIFT')
		self.dm = cv2.DescriptorMatcher_create('BruteForce')
		
		self.loadCameraInfo()
		self.analyzeImages()
		
	def loadCameraInfo(self):
		fh = open('CameraInfo.pickle', 'r')
		camdict = pickle.load(fh)
		fh.close()
		self.cameraMatrix = camdict['cameraMatrix']
		self.inverseCameraMatrix = camdict['inverseCameraMatrix']
		self.distCoeffs = camdict['distCoeffs']
		self.P = camdict['P']
		
		
	def analyzeImages(self):
		# Image1
		directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
		path = directory + '/dynamics/goodcapture1.png'
		img = cv2.imread(path)
		colorimg = np.copy(img)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		img_pts = self.fd.detect(img)
		img_kp, img_desc = self.de.compute(img, img_pts)
		#~ print len(img_pts)
		#~ print len(img_kp)
		#~ print img_desc.shape
		#~ for p in img_kp:
			#~ cv2.circle(colorimg, (int(p.pt[0]),int(p.pt[1])) , int(p.size), (0, 0, 255))
		#cv2.imshow('img', img)
		#cv2.waitKey()
		
		# Image2
		path = directory + '/dynamics/goodcapture16.png'
		img2 = cv2.imread(path)
		colorimg2 = img2.copy()
		img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
		img2_pts = self.fd.detect(img2)
		img2_kp, img2_desc = self.de.compute(img2, img2_pts)
		#~ print len(img2_pts)
		#~ print len(img2_kp)
		#~ print img2_desc.shape
		#~ for p in img2_kp:
			#~ cv2.circle(colorimg2, (int(p.pt[0]),int(p.pt[1])) , int(p.size), (0, 0, 255))
		#cv2.imshow('img2', img2)
		#cv2.waitKey()
		
		
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
		kp1_array = np.array(list(x.pt for x in img_kp))
		kp2_array = np.array(list(x.pt for x in img2_kp))
		i1_pts = kp1_array[i1_indices,:]
		i2_pts = kp2_array[i2_indices,:]
		#~ print i1_pts.shape
		n = len(i2_pts)
		
		# Undistort matched points
		undistorted = cv2.undistort(colorimg, self.cameraMatrix, self.distCoeffs)
		i1_undist_pts = cv2.undistortPoints(i1_pts.reshape([n,1,2]), self.cameraMatrix, self.distCoeffs)
		i1_undist_pts = i1_undist_pts.reshape([n,2])
		i1_undist_pts[:,0] = i1_undist_pts[:,0]*self.cameraMatrix[0][0]+self.cameraMatrix[0][2]
		i1_undist_pts[:,1] = i1_undist_pts[:,1]*self.cameraMatrix[1][1]+self.cameraMatrix[1][2]
		print i1_undist_pts
		for p in i1_undist_pts:
			cv2.circle(undistorted, (int(p[0]),int(p[1])) , 5, (255, 0, 0))
			
		# Make a template
		t1_center = np.median(i1_undist_pts, axis=0)
		t1_center = t1_center.astype('int')
		
		cv2.circle(undistorted, tuple(t1_center), 10, (0,0,0))
		cv2.imshow('img',undistorted)
		cv2.waitKey(0)
		
		
		#~ undistorted = cv2.undistort(img, self.cameraMatrix, self.distCoeffs)
		#~ stacked = img - undistorted
		#~ cv2.imshow('undistort', stacked)
		#~ cv2.waitKey(0)
		
		#~ # Draw matches
		#~ stacked_img = stackImagesVertically(colorimg, colorimg2)
		#~ for i in range(len(i1_pts)):
			#~ cv2.line(stacked_img, (int(i1_pts[i,0]), int(i1_pts[i,1])), (int(i2_pts[i,0]), int(i2_pts[i,1])+360), (255,255,255))
		#~ cv2.imshow('img',stacked_img)
		#~ cv2.waitKey(0)
			

if __name__=='__main__':
	tester1 = Tester()
