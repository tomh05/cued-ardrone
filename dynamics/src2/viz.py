#!/usr/bin/env python

'''
MapExplorer.py
Uses PositionController.py and issues orders of destinations to the
drone, in an attempt to map out the world.
'''

import roslib; roslib.load_manifest('dynamics')
import rospy
from tf import transformations

import math
from math import cos, sin
import pickle
import numpy as np
from time import time, sleep

import copy

from visualization_msgs.msg import Marker, MarkerArray

def run():
	pub=rospy.Publisher('/templatemarkers', MarkerArray)
	rospy.init_node('viz')
	r=rospy.Rate(10)
	
	m=Marker()
	m.header.frame_id='/world'
	
	m.ns='basic'
	m.id=0
	m.type=Marker.CUBE
	
	m.action=Marker.ADD
	
	m.pose.position.x=0
	m.pose.position.y=0
	m.pose.position.z=0
	m.pose.orientation.x=0.0
	m.pose.orientation.y=0.0
	m.pose.orientation.z=0.0
	m.pose.orientation.w=1.0
	
	m.scale.x=50
	m.scale.y=50
	m.scale.z=1.0
	
	m.color.r=0.0
	m.color.g=1.0
	m.color.b=0.0
	m.color.a=1.0
	
	m.lifetime=rospy.Duration(1.0)
	
	ma=MarkerArray()
	ma.markers.append(m)
	
	#~ print ma
	
	tstop=time()+5
	while (time()<tstop):
		for m in ma.markers:
			m.header.seq+=1
			m.header.stamp=rospy.Time.now()
		pub.publish(ma)
		r.sleep()
	
	tstop=time()+5
	m2=copy.deepcopy(m)
	m2.pose.position.x=500
	m2.id=1
	ma.markers.append(m2)
	while (time()<tstop):
		for m in ma.markers:
			m.header.seq+=1
			m.header.stamp=rospy.Time.now()
		pub.publish(ma)
		r.sleep()	
	
	rospy.spin()
		
	
if __name__=='__main__':
	run()
