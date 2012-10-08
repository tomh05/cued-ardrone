#!/usr/bin/env python

import roslib; roslib.load_manifest('simpledrone')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
import numpy


def navdata(d): 
	#print d.rotX
	pass

def imgproc(d):
	bridge = CvBridge()
	cvimg = bridge.imgmsg_to_cv(d,"bgr8")
	npimg = numpy.asarray(cvimg)
	print "got image"
	cv2.imshow("Output",npimg)
	cv2.waitKey(1)

def connect():
	rospy.init_node('dronecontroller')
	rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,navdata)
	rospy.Subscriber('/ardrone/front/image_raw',Image,imgproc)


def run():
	connect()
	cv2.namedWindow("Output")
	rospy.spin()

if __name__ == '__main__':
	run()
