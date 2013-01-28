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
from custom_msgs.msg import StampedFrames
from custom_msgs.msg import StampedMatchesWithImage
from copy import deepcopy

class FeatureInstance():
    def __init__ (self, target):
        self.loaded = False
        self.loading = False
        self.desc1 = None
        self.desc2 = None
        self.subscription = rospy.Subscriber(target, Image, self.image_proc)
        
        self.fd = cv2.FeatureDetector_create('SIFT')
        self.de = cv2.DescriptorExtractor_create('SIFT')
        
    def image_proc(self, d):
        """Converts the ROS published image to a cv2 numpy array
        and passes to FeatureTracker"""        
        
        
        if self.loading == True:
            print "Loading frame..."
            self.loading = False   
            if self.desc1 != None:         
                self.img2 = deepcopy(self.img1)
                self.header2 = deepcopy(self.header1)
            self.img1 = d
            self.header1 = d.header            
            # ROS to cv image
            bridge = CvBridge()
            cvimg = bridge.imgmsg_to_cv(d,"bgr8")
            # cv to cv2 numpy array image
            npimg = np.asarray(cvimg)
            # Pass to FeatureTracker
            self.feature_detect(npimg)
        
    def feature_detect(self, img):
        """Takes a cv2 numpy array image features are extracted.
        Multiple view geometry is then used to calculate the rotation and
        translation of the camera between frames and the position of observed
        points"""
        
        # Convert to monochrome
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if self.desc1 != None:            
            self.pts2 = self.pts1
            self.kp2 = self.kp1
            self.desc2 = self.desc1
        
        # Extract features and descriptors
        self.pts1 = self.fd.detect(grey_now)
        self.kp1, self.desc1 = self.de.compute(grey_now, self.pts1)
        
        
        
        
    
class FeatureHandler():
    def __init__(self):
        rospy.init_node('Feature_Matcher')
        rospy.Subscriber('/feature_matcher/load', std_msgs.msg.String, self.load_features)
        rospy.Subscriber('/feature_matcher/process', std_msgs.msg.String, self.match_images)
        self.match_pub = rospy.Publisher('/feature_matcher/matches', StampedFrames)
        self.lookup = { 'null': -1}
        self.instances = []
        self.dm = cv2.DescriptorMatcher_create('BruteForce') #'BruteForce-Hamming' for binary 
        
    def load_features(self, target):
        target = str(target).split(' ')[1]
        if not target in self.lookup:
            self.lookup[target] = len(self.instances)
            self.instances.append(FeatureInstance(target))
        self.instances[self.lookup[target]].loading = True
        
    def match_images(self, targets):
        targets = str(targets).split(' ')[1]
        targets = targets.split(',')
        if (not targets[0] in self.lookup) or (not targets[1] in self.lookup):
            print "Frames not initialised"
            return
        if targets[0] == targets[1]:
            a = self.instances[self.lookup[targets[0]]]
            if (a.desc2 == None):
                print "Frames not loaded"
                return
            a_img = a.img1
            b_img = a.img2
            header = a.header1
            matches_pts1, matches_pts2 = self.match_points(a.kp1, a.kp2, a.desc1, a.desc2)
            
        else:
            a = self.instances[self.lookup[targets[0]]]
            b = self.instances[self.lookup[targets[1]]]
            if (a.desc1 == None or b.desc1 == None):
                print "Frames not loaded"
                return
            a_img = a.img1
            b_img = b.img1
            if (a.header1.stamp > b.header1.stamp):
                header = a.header1
            else:
                header = b.header1
            matches_pts1, matches_pts2 = self.match_points(a.kp1, b.kp1, a.desc1, b.desc1)
            
        # Publish matches and images
        stamped_frames = StampedFrames()
        
        smwi = StampedMatchesWithImage()
        smwi.header = a_img.header
        smwi.pts = matches_pts1.reshape(-1,).tolist()
        smwi.image = a_img
        
        stamped_frames.frame1 = deepcopy(smwi)
        
        smwi.header = b_img.header
        smwi.pts = matches_pts2.reshape(-1,).tolist()
        smwi.image = b_img
        
        stamped_frames.frame2 = smwi
        
        stamped_frames.header = header
        stamped_frames.header.frame_id = targets[0]+","+targets[1]
        
        self.match_pub.publish(stamped_frames)
        
    
    def match_points(self, kp1, kp2, desc1, desc2):
        """Matches the two points. There appears to be no way to specify match
        paramaters from python, so crossCheck is implemented manually"""
        # Match features        
        matches = self.dm.match(desc1, desc2)
        matches2 = self.dm.match(desc2, desc1)

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
        kp1_array = np.array(list(x.pt for x in kp1))
        kp2_array = np.array(list(x.pt for x in kp2))
        i1_pts = kp1_array[i1_indices,:]
        i2_pts = kp2_array[i2_indices,:]
        
        return i1_pts, i2_pts 



def run():
    # Initialise tracker
    f = FeatureHandler()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
