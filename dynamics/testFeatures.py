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
		print path
		img = cv2.imread(path)
		print img.shape
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		print img.shape
		img_pts = self.fd.detect(img)
		img_kp, img_desc = self.de.compute(img, img_pts)
		print len(img_pts)
		print len(img_kp)
		print img_desc.shape
		cv2.imshow('img', img)
		cv2.waitKey(1000)
		sleep(0.5)		
		#kpdetail=[[p.pt, p.size] for p in img_kp]
		#print kpdetail
		for p in img_kp:
			cv2.circle(img, (int(p.pt[0]),int(p.pt[1])) , int(p.size), (255, 0, 0))
		cv2.imshow('img', img)
		cv2.waitKey()
		
		





if __name__=='__main__':
	tester1 = Tester()
