#!/usr/bin/env python
import roslib
roslib.load_manifest('dynamics')

import rospy
import cv
import Image as PImage
from std_msgs.msg import String, UInt8, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from time import time, sleep
import sys, os
# Where is this file?
this_dir = os.path.abspath(os.path.dirname(__file__))
# Insert a path to load modules from relative to this file
sys.path.insert(0, os.path.abspath(os.path.join(this_dir, '..')))
# Load the aruco module.
from ardrone.aruco import detect_markers

class imageprocessor:
	cbct = 0	#callback count
	dtct = 0	#detect count
	last_frame_epoch = time()	#used for calculating frame rate
	
	def __init__(self, suCtrler):
		self.image_pub = rospy.Publisher("/ardrone/image_marked",Image)
		self.camselectpub = rospy.Publisher("/ardrone/camselect",UInt8)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.image_raw_callback)
		self.suCtrler = suCtrler
		
	def image_raw_callback(self,data):
		try:
			cv_oim = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e
	
		oim_size = (640, 320)
		im_size = (640, 320)
		cv_im_midp = (im_size[0]/2,im_size[1]/2)
		
		#convert cv to PIL
		oim = PImage.fromstring('RGB',oim_size,cv_oim.tostring())
		#crop PIL
		im = oim.crop((0,0,im_size[0],im_size[1]))
		#convert cropped PIL to cv for showing
		cv_im = cv.CreateImageHeader(im_size, cv.IPL_DEPTH_8U, 3)
		cv.SetData(cv_im, im.tostring())
		
		self.cbct = self.cbct + 1
		
		markers = detect_markers(im)
		coords = []; mids=[]
		for m in markers:
			centroid = (m.centroid_x(), m.centroid_y())
			coord = (im_size[1]/2-centroid[1], im_size[0]/2-centroid[0])	#drone coord and cv coord are different!
			coords.append(coord)
			mids.append(m.id())
			cv.Line(cv_im,cv_im_midp,centroid,cv.Scalar(0,0,255))
		self.suCtrler.visioninfo_logger(coords, mids)
		
		if len(markers) != 0:
			self.dtct = self.dtct + 1
		if self.cbct == 20:
			epoch = time()
			#print 'marker detect rate: '+str(5*self.dtct)+'%   '+'frame rate: '+str(20.0/(epoch-self.last_frame_epoch))
			self.cbct = 0; self.dtct = 0; self.last_frame_epoch = epoch
		cv.ShowImage("Image Window",cv_im)
		cv.WaitKey(2)
		
		try:
			self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_im, "bgr8"))
		except CvBridgeError, e:
			print e


def main(args):
	rospy.init_node('image_converter', anonymous=True)
	ic = image_conv_processor()
	sleep(0.5)
	ic.camselectpub.publish(1)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
