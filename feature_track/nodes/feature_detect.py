#!/usr/bin/env python

import roslib; roslib.load_manifest('feature_track')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
import cv
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg
from std_msgs.msg import Empty
import std_msgs.msg
from rospy.numpy_msg import numpy_msg
#from feature_track.msg import StampedMatchesWithImages

class FeatureInstance():
    def __init___ (self, target):
        
        

        

        
        
        
        
    
class FeatureDetector():
    def __init__(self):
        rospy.init_node('Feature_Detector')
        self.fd = cv2.FeatureDetector_create('SIFT')
        self.de = cv2.DescriptorExtractor_create('SIFT')
        self.dm = cv2.DescriptorMatcher_create('BruteForce') #'BruteForce-Hamming' for binary
        self.loaded = False
        self.loading = False
        self.match_pub = rospy.Publisher('/feature_detect/feature', StampedFeaturesWithImage)
        self.desc1 = None
        self.kp1 = None
        self.header1 = None
        self.pts1 = None
        self.img1 = None
        self.subscription = rospy.Subscriber(target, Image, self.image_proc)
        
        
        
    def feature_detect(self, img):
        """Takes a cv2 image and features are extracted then published"""
        
        # Convert to monochrome
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        self.pts2 = self.pts1
        self.kp2 = self.kp1
        self.img2 = self.img1
        self.header2 = self.header1
        
        # Extract features and descriptors
        self.pts1 = self.fd.detect(grey_now)
        self.kp1, self.desc1 = self.de.compute(grey_now, self.pts1)
        
    def img_proc(self, d):
        """Converts the ROS published image to a cv2 numpy array
        and passes to FeatureTracker"""        
        
        if self.loading == True:
            self.loading = False
            self.img1 = d
            self.header1 = d.header            
            # ROS to cv image
            bridge = CvBridge()
            cvimg = bridge.imgmsg_to_cv(d,"bgr8")
            # cv to cv2 numpy array image
            npimg = np.asarray(cvimg)
            # Pass to FeatureTracker
            self.feature_detect(npimg)
        



def run():
    # Initialise tracker
    f = FeatureDetector()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()

