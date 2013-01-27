#!/usr/bin/env python

'''
HoverController.py
This program performs high level control of marker hovering: 
sets up transform trees, defines world coordinate, works out current pose and sets desired pose, and sends these to position_controller
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

import tf
import numpy
from math import pi, cos, sin
from time import sleep
from dynamics.msg import ARMarkers

def posesub_callback(msg):
	print '\n'
	print msg.header.seq
	if msg.markers == []:
		print 'no markers'
	else:
		print msg.markers[0].confidence
	



if __name__ == '__main__':
	rospy.init_node('posemsgtest')
	posesub = rospy.Subscriber('/ar_pose_marker', ARMarkers, posesub_callback)
	rospy.spin()
