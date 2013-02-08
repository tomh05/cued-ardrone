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

from PositionController import PositionController


def stackImagesVertically(top_image, bottom_image):
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


class Tester:
	def __init__(self):
		self.fd = cv2.FeatureDetector_create('SIFT')
		self.de = cv2.DescriptorExtractor_create('SIFT')
		self.dm = cv2.DescriptorMatcher_create('BruteForce')
		
		self.analyzeImages()
		
	def analyzeImages(self):
		
		
		# Image1
		directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
		path = directory + '/dynamics/goodcapture1.png'
		img = cv2.imread(path)
		#imgcopy = np.copy(img)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		img_pts = self.fd.detect(img)
		img_kp, img_desc = self.de.compute(img, img_pts)
		print len(img_pts)
		print len(img_kp)
		print img_desc.shape
		#cv2.imshow('img', img)
		#cv2.waitKey(1000)
		sleep(0.5)		
		#kpdetail=[[p.pt, p.size] for p in img_kp]
		#print kpdetail
		for p in img_kp:
			cv2.circle(img, (int(p.pt[0]),int(p.pt[1])) , int(p.size), (255, 0, 0))
		#cv2.imshow('img', img)
		#cv2.waitKey()
		
		# Image2
		path = directory + '/dynamics/goodcapture16.png'
		img2 = cv2.imread(path)
		img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
		img2_pts = self.fd.detect(img2)
		img2_kp, img2_desc = self.de.compute(img2, img2_pts)
		print len(img2_pts)
		print len(img2_kp)
		print img2_desc.shape
		#cv2.imshow('img2', img2)
		#cv2.waitKey(1000)
		sleep(0.5)		
		#kpdetail=[[p.pt, p.size] for p in img_kp]
		#print kpdetail
		for p in img2_kp:
			cv2.circle(img2, (int(p.pt[0]),int(p.pt[1])) , int(p.size), (255, 0, 0))
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
		
		# Order pairs
		kp1_array = np.array(list(x.pt for x in img_kp))
		kp2_array = np.array(list(x.pt for x in img2_kp))
		i1_pts = kp1_array[i1_indices,:]
		i2_pts = kp2_array[i2_indices,:]
		
		print len(i1_pts)
		print len(i2_pts)
		
		# Draw matches
		stacked_img = stackImagesVertically(img, img2)
		for i in range(len(i1_pts)):
			cv2.line(stacked_img, (int(i1_pts[i,0]), int(i1_pts[i,1])), (int(i2_pts[i,0]), int(i2_pts[i,1])+360), (255,255,255))
		cv2.imshow('img',stacked_img)
		cv2.waitKey(0)
			

if __name__=='__main__':
	tester1 = Tester()
