#!/usr/bin/env python

#============================== Feature Matcher ===============================
# 
# This script matches the latest features with the previous features.
#
# There is no particular reason to confine this to sequential pairs beyond 
# practicle CPU limitations. 
#
#==============================================================================
# 
# The two major cpu commitments in the script are the matching and calc F
#   Match times are consistent and generally make up the majority of proc time
#   F times are variable (due to RANSAC) and occasionally much higher
#
# SIFT (BruteForce) should be capable of 50 fps
# ORB (BruteForce-Hamming) should be capable of 35 fps (higher num of features)
#
#==============================================================================

import roslib; roslib.load_manifest('feature_track')
import rospy
import numpy as np
import cv2
import std_msgs.msg
from custom_msgs.msg import StampedFeaturesMatches
from custom_msgs.msg import StampedFeaturesWithImage
from sensor_msgs.msg import CameraInfo
import time

class FeatureBuffer:
    def __init__(self):
        self.points = None
        self.desc = None
        self.header = None

class FeatureMatcher:
    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Set up descriptor matcher
        self.matcher_type = 'None'
        # Initialise list of feature frames
        self.feature_buffer = []
        
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        rospy.Subscriber('/feature_extractor/features',StampedFeaturesWithImage,self.on_got_features)
        self.match_pub = rospy.Publisher('/feature_matcher/matches',StampedFeaturesMatches)
        
    def on_got_features(self, sfwi):
        self.time_prev = time.time()
        # Get np formatted descriptors
        new_kp, new_desc = self.get_keypoints_and_descriptors(sfwi)
        
        if len(self.feature_buffer) > 0:
            pts1, pts2, desc1, desc2 = self.match_points(self.feature_buffer[-1].points, new_kp, self.feature_buffer[-1].desc, new_desc)
            if len(pts1) > 16:
                #print len(pts1), " Desc matches"
                # Undistort points using calibration data    
                pts1_un = cv2.undistortPoints(np.array([pts1]), self.cameraMatrix, self.distCoeffs, P=self.cameraMatrix)[0]
                pts2_un = cv2.undistortPoints(np.array([pts2]), self.cameraMatrix, self.distCoeffs, P=self.cameraMatrix)[0]
                # Extract fundamental
                F, pts1_F, pts2_F, desc1_F, desc2_F = self.extract_fundamental(pts1_un, pts2_un, desc1, desc2)
                if len(pts1_F) > 4:
                    #print len(pts1_corr), " F matches"
                    self.publish_matches(pts1_F, pts2_F, desc1_F, desc2_F, self.feature_buffer[-1].header, sfwi.header)
        
        fb = FeatureBuffer()
        fb.points = new_kp
        fb.desc = new_desc
        fb.header = sfwi.header
        self.feature_buffer.append(fb)
        print "Features Matched ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) "
    
    def publish_matches(self, pts1, pts2, desc1, desc2, header1, header2):
        sfm = StampedFeaturesMatches()
        sfm.header1 = header1
        sfm.header2 = header2
        sfm.points1 = pts1.reshape(-1,).tolist()
        sfm.points2 = pts2.reshape(-1,).tolist()
        sfm.descriptors1 = desc1.reshape(-1,).tolist()
        sfm.descriptors1_stride = desc1.shape[1]
        sfm.descriptors2 = desc2.reshape(-1,).tolist()
        sfm.descriptors2_stride = desc2.shape[1]
        sfm.descriptors_matcher = self.matcher_type
        self.match_pub.publish(sfm)
    
    def get_keypoints_and_descriptors(self, sfwi):
        # Need to re-numpy the array kp & descriptors
        kp = np.reshape(np.array(sfwi.points), (-1, 2))
        if (sfwi.descriptors_matcher == 'BruteForce'):
            desc = np.reshape(np.array(sfwi.descriptors, np.float32), (-1, sfwi.descriptors_stride))   
        elif (sfwi.descriptors_matcher == 'BruteForce-Hamming'):
            desc = np.reshape(np.array(sfwi.descriptors, np.uint8), (-1, sfwi.descriptors_stride))
        # Switch matcher if necessary
        if (self.matcher_type != sfwi.descriptors_matcher):
            self.matcher_type = sfwi.descriptors_matcher
            self.dm = cv2.DescriptorMatcher_create(self.matcher_type)
            self.feature_buffer = []
            print "Auto-switching to ", self.matcher_type
        return kp, desc
    
    def match_points(self, kp1, kp2, desc1, desc2):        
        time_prev = time.time()
        """Matches the two points. There appears to be no way to specify match
        paramaters from python, so crossCheck is implemented manually"""
        """NOTE: Whilst cross check doubles matching time, it significantly 
        reduces the time wasted due false matches later in the pipeline.
        Fundamental matching takes ~4x as long without cross checking"""
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
        
        # Find descriptors that were matched
        desc1 = np.array(list(desc1[i] for i in i1_indices))
        desc2 = np.array(list(desc2[i] for i in i2_indices))
        
        # Order pairs
        i1_pts = kp1[i1_indices,:]
        i2_pts = kp2[i2_indices,:]
        
        return i1_pts, i2_pts, desc1, desc2
        
    def extract_fundamental(self, i1_pts_undistorted, i2_pts_undistorted, desc1, desc2):
        """Extract fundamental matrix and then remove outliers"""
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 5, param2 = 0.99)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_corr = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        mask_prepped = np.resize(mask.T, (desc1.shape[1],desc2.shape[0])).T
        desc1_corr = np.reshape(desc1[mask_prepped==1], (-1, desc1.shape[1]))
        desc2_corr = np.reshape(desc2[mask_prepped==1], (-1, desc2.shape[1]))
        
        return F, i1_pts_corr, i2_pts_corr, desc1_corr, desc2_corr
        
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
        self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
        self.distCoeffs = np.array([ci.D], dtype=np.float32)
        self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
        print "                    Calibration Initialised\r\n"


def run():
    rospy.init_node('Feature_Matcher')
    # Initialise controller
    
    # Get parameters
    
    # Print startup info
    print "\r\n"
    print "======================= Feature Matcher ==========================="
    print " Auto-detecting matcher type from feature type"
    print "==================================================================="
    print "\r\n"
    
    fm = FeatureMatcher()
    
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
