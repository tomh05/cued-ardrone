#!/usr/bin/env python

'''
MapExplorer.py
Uses PositionController.py and issues orders of destinations to the
drone, in an attempt to map out the world.
'''

import roslib; roslib.load_manifest('dynamics')
import rospy
from std_msgs.msg import Empty, UInt8
from tf import transformations
import tf

import math
from math import cos, sin, pi
import pickle
import numpy as np
from time import time, sleep

import copy

from visualization_msgs.msg import Marker, MarkerArray

class MarkerVisualizer:
	def __init__(self):
		self.pub=rospy.Publisher('/templatemarkers', MarkerArray)
		rospy.init_node('viz')
		self.msub=rospy.Subscriber('addmarker', Marker, self.addMarker)
		self.clearsub=rospy.Subscriber('clearmarkers',Empty, self.clearMarkers)
		self.r=rospy.Rate(10)
		self.ma=MarkerArray()
		
		self.br = tf.TransformBroadcaster()
		self.w2dr = (0,0,1)
				
		self.run()
		
	def clearMarkers(self, msg):
		self.ma=MarkerArray()
			
	def addMarker(self, m):
		self.ma.markers.append(m)

	def run(self):
		while not rospy.is_shutdown():
			self.br.sendTransform(self.w2dr, tf.transformations.quaternion_from_euler(0, 0, 0),\
			 rospy.Time.now(), 'ardrone_base_link', "world")
			self.br.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(pi, 0, -pi/2),\
			 rospy.Time.now(), 'MEorigin', "world")
			for m in self.ma.markers:
				m.header.seq+=1
				m.header.stamp=rospy.Time.now()			
			self.pub.publish(self.ma)
			self.r.sleep()
		
	
if __name__=='__main__':
	MarkerVisualizer()
