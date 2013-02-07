#!/usr/bin/env python

#================================= Scan =======================================
# 
# This script carries out a single 'scan' proceedure.
#
#==============================================================================
#
# Triangulation in arbitrary flight was found to be infeasible (at least as 
# implemented). 
#
# This attempts to take advantage to stable well defined drone aspects
#
# The drone is capable of determining absolute elevation using the ultrasound
# and absolute orientation using the magnetometer
# 
# Additionally the drone is capable of two smooth movements:
#   It can ascend and descent levelly
#   It can yaw levelly
# Lateral motion tilts the drone and is less predictable
#
# The two absolute measurements and reliable motions happend to coincide and 
# are exploited by this
#
#==============================================================================
#
# A 'scan' is as follows:
#   The drone stops and holds position to remove intertia
# 1)The drone records an image (frame1)
#   The drone ascends to an increased elevation
#   The drone records an image (frame2)
#   Matches are triangulated from frame1 to frame2 using known elevation change
#   The drone yaws 90deg
# 2)The drone records an image (frame1)
#   The drone descends to the original elevation
#   The drone records an image (frame2)
#   Matches are triangulated from frame1 to frame2 using known elevation change
#   The drone yaws 90deg
# 3)Stages 1 and 2 are repeated, leaving the drone in its original position
#
#   Good triangulation should have been carried out in all cardinal directions
#
#==============================================================================

import roslib; roslib.load_manifest('scan')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
import cv
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg as gm
import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D
import tf
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
from std_msgs.msg import Header
from nav_msgs.msg import Path
import math
import time
import threading
import os
from copy import deepcopy

    
def stackImagesVertically(top_image, bottom_image):
    """Takes two cv2 numpy array images top_image, bottom_image
       Returns a cv2 numpy array image of the two stacked vertically
       Image dimensions need not match, output it left aligned"""
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
    
def trunc(f, n=10):
    '''Truncates/pads a float f to a string of n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return str(f)[:slen]
    
def np_to_str(n):
    """Tidily converts a numpy array to a string"""
    s = ""
    for r in n:
        s+=trunc(r)+", "
    return s

def enum(**enums):
    return type('Enum', (), enums)

class ScanController:
    #States = enum(FLIGHT = 1, 
    #                  LOW_0 = 2, ASCEND_0 = 3, HIGH_0 = 4, ROT_0 = 5,
    #                  HIGH_90 = 6, DESCEND_90 = 7, LOW_90 = 8, ROT_90 = 9,
    #                  LOW_180 = 10, ASCEND_180 = 11, HIGH_180 = 12, ROT_180 = 13,
    #                  HIGH_270 = 14, DESCEND_270 = 15, LOW_270 = 16, ROT_270 = 17,
    #                  DONE = 18)
    
    def __init__(self):
        #self.state = States.FLIGHT
        self.image = None
        
        # Initialise tracker
        self.feature_tracker = FeatureTracker()
        
        # Initialise ROS node
        self.connect()
        
        self.frame_1_is_loaded = False
        self.frame_2_is_loaded = False
        
        self.auto_scan_timer = None
    

        
    def crude_auto_scan(self, event):
        """ Carries out a single scan
        
        At present is somewhat brute force controlled and WILL lock the thread
        whilst running
        """
        
        print "Pre\r\n"
        
        
        twist = gm.Twist()
        twist_pub = rospy.Publisher('cmd_vel',gm.Twist)

        #if self.state == States.DONE:
        #    return
        
        print "Flight\r\n"
        
        #if self.state == States.FLIGHT:
        
        """
        # Remove any inertia
        """
        twist = gm.Twist()
        twist_pub.publish(twist)
        rospy.sleep(1.0)
        #self.state = States.LOW_0
        
        """
        # Process the first frame
        """
        print "Frame1\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(1)
        self.feature_tracker.load_tf(1)
        
        """
        # Ascend 0.3m
        """
        deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
        position1, quaternion1 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)   
        
        twist = gm.Twist()
        twist.linear.z  = +0.25
        twist_pub.publish(twist)
        done = False
        
        while(not done):
            deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
            self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
            position2, quaternion2 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
            print "Height Change : ", position2[2]-position1[2]
            if (position2[2]-position1[2]> 0.3):
                done = True
                twist = gm.Twist()
                twist_pub.publish(twist)
                
        rospy.sleep(0.5)
        #self.state = States.HIGH_0
        
        """
        # Process the second frame
        """
        print "Frame2\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(2)
        self.feature_tracker.load_tf(2)
        
        self.feature_tracker.process_frames()
        
        """
        # Rotate 90 deg
        """
        print "Rotating\r\n"       
        
        
        deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
        position1, quaternion1 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
        quaternion1 = tf.transformations.quaternion_inverse(quaternion1)
        
        twist = gm.Twist()
        twist.angular.z  = +0.1
        twist_pub.publish(twist)
        done = False
        
        while(not done):
            deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
            self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
            position2, quaternion2 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
            quaternion2 = tf.transformations.quaternion_inverse(quaternion2)
            relative_quat = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(quaternion1), quaternion2)
            angles = tf.transformations.euler_from_quaternion(relative_quat)
            if (angles[2] > np.pi/2.):
                done = True
                twist = gm.Twist()
                twist_pub.publish(twist)
                
        rospy.sleep(0.5)
        
        """
        # Process the first frame
        """
        print "Frame1\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(1)
        self.feature_tracker.load_tf(1)
        
        deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
        position1, quaternion1 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)     
        
        """
        # Descend 0.3m
        """
        twist = gm.Twist()
        twist.linear.z  = -0.25
        twist_pub.publish(twist)
        done = False
        
        while(not done):
            deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
            self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
            position2, quaternion2 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
            if (position2[2]-position1[2]< -0.3):
                done = True
                twist = gm.Twist()
                twist_pub.publish(twist)
                
        rospy.sleep(0.5)
        #self.state = States.HIGH_0
        
        """
        # Process the second frame
        """
        print "Frame2\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(2)
        self.feature_tracker.load_tf(2)
        
        self.feature_tracker.process_frames()
        
    def get_frame_1(self, empty):
        print "Loading Frame 1\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(1)
        self.feature_tracker.load_tf(1)
        self.frame_1_is_loaded = True
    
    def get_frame_2(self, empty):
        print "Loading Frame 2\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(2)
        self.feature_tracker.load_tf(2)
        self.frame_2_is_loaded = True
        
        if self.frame_1_is_loaded:
            print "Processing Frames\r\n"
            self.feature_tracker.process_frames()
            # Shift frame2 to frame1 incase we want sequential use
            self.feature_tracker.load_through()
            
    def start_scanning(self, empty):
        print "Beginning auto-scanning"
        self.get_frame_1(Empty)
        self.auto_scan_timer = rospy.Timer(rospy.Duration(0.1), self.auto_scan)
        
    def stop_scanning(self, empty):
        if self.auto_scan_timer != None:
            self.auto_scan_timer.shutdown()
            
    def auto_scan(self, event):        
        # Get latest world co-ords of drone
        #now = rospy.Time.now()
        #self.feature_tracker.tf.waitForTransform( "world", "ardrone_base_frontcam", self.feature_tracker.time_now, rospy.Duration(1.))
        pw, q = self.feature_tracker.tf.lookupTransform( "world", "ardrone_base_frontcam", rospy.Time(0))
        # Difference in position in world co-ords
        diff = self.feature_tracker.position_w1 - np.array(pw)
        # Magnitude of difference
        mag = np.sqrt(diff[0]*diff[0] + diff[1]*diff[1]+diff[2]*diff[2])
        #print "diff: ", diff
        if (abs(diff[2]) > 0.2):
            print "Triggering"
            self.auto_scan_timer.shutdown()
            self.get_frame_2(Empty)
            self.start_scanning(Empty)
        
    
    def connect(self):     
        
        self.feature_tracker.tf = tf.TransformListener()
        self.tf = self.feature_tracker.tf
        rospy.Subscriber('/ardrone/front/image_raw',Image,self.feature_tracker.imgproc)
        rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, self.feature_tracker.setCameraInfo)
        rospy.Subscriber('/xboxcontroller/button_a',Empty,self.get_frame_1)
        rospy.Subscriber('/xboxcontroller/button_b',Empty,self.get_frame_2)
        rospy.Subscriber('/xboxcontroller/button_x',Empty,self.start_scanning)
        rospy.Subscriber('/xboxcontroller/button_y',Empty,self.stop_scanning)
        rospy.Subscriber('/xboxcontroller/button_back',Empty,self.crude_auto_scan)

    

class FeatureTracker:
    def __init__(self):
        self.roll = 0.
        self.quaternion = tf.transformations.quaternion_from_euler(0.,0.,0., axes='sxyz')
        self.grey_previous = None
        self.calibrated = False
        self.fd = cv2.FeatureDetector_create('SIFT')
        self.de = cv2.DescriptorExtractor_create('SIFT')
        self.dm = cv2.DescriptorMatcher_create('BruteForce') #'BruteForce-Hamming' for binary
        self.desc1 = None
        self.kp1 = None
        cv2.namedWindow("track")
        self.cloud_pub = rospy.Publisher('/scan/absolute_cloud', PointCloud)
        self.cloud_pub2 = rospy.Publisher('/scan/relative_cloud', PointCloud)
        self.pose_pub = rospy.Publisher('/scan/pose', gm.PoseStamped)
        self.prev_position = None
        self.prev_quat = None
        self.prev_prev_position = None
        self.mag_dist = None
        self.speech_limiter = 0
        self.corners = None
        self.time1 = None
        
        self.image = None
        
        self.descriptor_buffer = None
        self.location_buffer = None
        
    def compute_F_error(self, F, x1_32, x2_32):
        errs = []
        for i, p in enumerate(zip(x1_32.T, x2_32.T)):
            errs.append(np.r_[p[1], 1].T.dot(F).dot(np.r_[p[0], 1]))
        return np.mean(errs)
        
    def find_and_match_points(self, grey_now):
        
        # Detect points
        pts2 = self.fd.detect(grey_now)
        self.pts2 = pts2

        # Describe points        
        kp1, desc1 = self.kp1, self.desc1
        kp2, desc2 = self.de.compute(grey_now, pts2)
        self.kp2, self.desc2 = kp2, desc2            
        
        # Bottom out if failed to get features
        if desc1 == None or desc2 == None or len(desc1) == 0 or len(desc2) == 0:
            self.grey_previous = grey_now
            self.kp1, self.desc1 = kp2, desc2
            print "No Features Found"
            return False, None, None
        
        # Match points
        i1_pts, i2_pts = self.match_points(kp1, kp2, desc1, desc2)
        
        # Bottom out if failed to get enough matches
        if len(i1_pts) < 8:
            self.grey_previous = grey_now
            self.kp1, self.desc1 = kp2, desc2
            print "Insufficient matches"
            return False, None, None
        
        return True, i1_pts, i2_pts
        
    def match_points(self, kp1, kp2, desc1, desc2):
        """Matches the two points. There appears to be no wat to specify match
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
        
        # Find descriptors that were matched
        self.desc = np.array(list(desc1[i] for i in i1_indices))
        
        return i1_pts, i2_pts
        
    def match_known_points(self, kp1, desc1):
        """Matches the specified point with already triangulated points"""
        desc2 = self.descriptor_buffer
        # Match features        
        matches = self.dm.match(desc1, desc2)
        #print "matches:\r\n", matches
        matches2 = self.dm.match(desc2, desc1)
        #print "matches2:\r\n", matches2
        
        #print "desc1:\r\n", desc1
        #print "desc2:\r\n", desc2

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
        #print i1_indices
        #print i2_indices
        
        # Order pairs
        kp1_array = np.array(list(x.pt for x in kp1))
        i2_pts_3D = np.array(list(self.location_buffer.T[i] for i in i2_indices))
        i1_pts = kp1_array[i1_indices,:]
        #print "loc buffer:\r\n", self.location_buffer
        
        # Find descriptors that were matched
        desca = np.array(list(desc1[i] for i in i1_indices))
        descb = np.array(list(desc2[i] for i in i2_indices))
        #print "desca:\r\n", desca
        #print "descb:\r\n", descb
        
        return i1_pts, i2_pts_3D
        
    def make_homo(self, pts):
        pts = np.append(pts,np.array([np.ones(pts.shape[0])]).T, 1)
        return pts
        
        
    def extract_fundamental(self, i1_pts_undistorted, i2_pts_undistorted, i1_pts_draw, i2_pts_draw):
        """
        Extract fundamental matrix and then remove outliers
        FM_RANSAC should be good with lowish outliers
        FM_LMEDS may be more robust in some cases
        """
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 3, param2 = 0.99)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_corr = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        #i1_pts_corr = np.array([i1_pts_masked])
        #i2_pts_corr = np.array([i2_pts_masked])
        
        i1_pts_draw_corr = np.reshape(i1_pts_draw[mask_prepped==1], (-1, 2))
        i2_pts_draw_corr = np.reshape(i2_pts_draw[mask_prepped==1], (-1, 2))
        #i1_pts_draw_corr = np.array([i1_pts_masked])
        #i2_pts_draw_corr = np.array([i2_pts_masked])
        
        
        # Filter descriptors
        # This slightly perverse rearrangement effectively horizontally stacks
        # the appropriate number of 1D column masks to match the desc matrix
        #
        # np.resize replicates elements in memory order to fill elements added
        # by resizing so the mask is transposed to ensure the mask is correctly
        # stacked, then transposed back into the desired shape
        mask_prepped = np.resize(mask.T, (self.desc.shape[1],self.desc.shape[0])).T
        self.desc_corr = np.reshape(self.desc[mask_prepped==1], (-1, self.desc.shape[1]))
        
        
        '''
        """============================================================
        # Filter points that fit F using cv2.correctMatches
        # This unhelpfully overwrites np.nan over rejected entried
        # np.nan == np.nan returns false so have to use np.isnan(.)
        # NB: This check appears redundant as F is calculated to match
        ============================================================"""
                
        i1_pts_corr, i2_pts_corr = cv2.correctMatches(F, i1_pts_undistorted, i2_pts_undistorted)
        mask_nan = np.isnan(i1_pts_corr[0])
        i1_pts_corr = np.reshape(i1_pts_corr[0][mask_nan == False], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_corr[0][mask_nan == False], (-1, 2))
        i1_pts_draw_corr = np.reshape(i1_pts_draw_corr[0][mask_nan == False], (-1, 2))
        i2_pts_draw_corr = np.reshape(i2_pts_draw_corr[0][mask_nan == False], (-1, 2))
        '''
        
        return F, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr
        
    def extract_projections(self, F, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr):
        """
        Uses known camera calibration to extract E
        Produces 4 possible P2 via linear algebra
        Isolates best P2 by projecting data points
        Filters out conflicting points
        """
        
        # Camera Matrices to extract essential matrix and then normalise
        E = self.cameraMatrix.transpose().dot(F.dot(self.cameraMatrix))
        E /= E[2,2]
        #print "E", E
        
        W = np.array([[0, -1, 0],[1, 0, 0], [0, 0, 1]])
        Z = np.array([[0, 1, 0],[-1, 0, 0], [0, 0, 0]])
                    
        # SVD of E
        U,SIGMA,V = np.linalg.svd(E)
        # Contruct Diagonal
        SIGMA = np.diag(SIGMA)
        # Force third eig to zero
        SIGMA[2,2] = 0
        if SIGMA[0,0] < 0.7*SIGMA[1,1] or SIGMA[1,1] < 0.7*SIGMA[0,0]:
            print "WARNING: Disparate singular values"
            
        """====================================================================
        # Technically E should be reformed and the SVD taken again
        # Otherwise there is no point in constraining SIGMA as only U and V
        # Are used from this point
        # In most cases this has no perceivable effect on U and V
        ===================================================================="""    
        E = U.dot(SIGMA.dot(V))
        # SVD of E
        U,SIGMA,V = np.linalg.svd(E)
        if np.linalg.det(U.dot(V))<0:
            V = -V 
        
        
        # Use camera1 as origin viewpoint
        P1 = np.append(np.identity(3), [[0], [0], [0]], 1)
        
        """============================================================
        # Compute the four possible P2 projection matrices
        # Note in particular the matrix multiplication order
        # This caught me out for a long while
        # Also the V returned by np's svd is V'
        ============================================================"""
        projections = []
        # UWV'|u3
        projections.append(np.append(U.dot(W.dot(V)),np.array([U[:,2]]).transpose(),1))
        # UWV'|-u3
        projections.append(np.append(U.dot(W.dot(V)),np.array([-U[:,2]]).transpose(),1))
        # UW'V'|u3
        projections.append(np.append(U.dot(W.transpose().dot(V)),np.array([U[:,2]]).transpose(),1))
        # UW'V'|-u3
        projections.append(np.append(U.dot(W.transpose().dot(V)),np.array([-U[:,2]]).transpose(),1))
        
        """============================================================
        # Determine projection with most valid points
        # Produce boolean mask for best case and filter pts
        ===============================================================
        # A comparison between manually triangulated and cv2 tri found
        # different results. It turns out cv2 output un-normalised homo
        # co-ords (i.e. non-unity w)
        ============================================================"""  
        
        # First must normalise co-ords
        i1_pts_corr_norm = self.make_homo(i1_pts_corr)
        i2_pts_corr_norm = self.make_homo(i2_pts_corr)
                              
        ind = 0
        maxfit = 0
        secfit = 0
        for i, P2 in enumerate(projections):
            points4D = self.unfiltered_triangulate_points(i1_pts_corr.transpose(), i2_pts_corr.transpose(), P1, P2)
            d1 = np.dot(self.cameraMatrix.dot(P1),points4D)[2]
            d2 = np.dot(self.cameraMatrix.dot(P2),points4D)[2]
            PI = sum((d1>0) & (d2>0))
            print "Support for P2 ", i, " : ", PI
            if PI > maxfit:
                secfit = maxfit
                maxfit = PI
                ind = i
                infront = (d1>0) & (d2>0)
        #if (maxfit < 4*secfit): maxfit ~= secfit is not actually a problem where translation is small, so cannot simply filter by it
        if maxfit < 4: # 8 Chosen as at least 8 points are needed to compute an effective fundamental matrix -> if we cannot satisfy at least 8 points we have serious issues
            print "==================================================="
            print "P1 not extracted"
            print "==================================================="
            return False, None, None, None, None, None, None            
        
        #print "P2"
        #print projections[ind]
        
        # Filter points
        infront = np.array([infront]).transpose()
        mask_prepped = np.resize(infront.T, (self.desc_corr.shape[1],self.desc_corr.shape[0])).T
        infront = np.append(infront, infront, 1)
        i1_pts_final = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_final = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        i1_pts_draw_final = np.reshape(i1_pts_draw_corr[infront==True], (-1, 2))
        i2_pts_draw_final = np.reshape(i2_pts_draw_corr[infront==True], (-1, 2))
        
        # Filter descriptors
        self.desc_final = np.reshape(self.desc_corr[mask_prepped==True], (-1, self.desc_corr.shape[1]))
        
        return True, P1, projections[ind], i1_pts_final, i2_pts_final, i1_pts_draw_final, i2_pts_draw_final
    
    
    def filter_correct_matches(self, i1_pts_undistorted, i2_pts_undistorted): # not used
        """
        Filter points that fit F using cv2.correctMatches
        This unhelpfully overwrites np.nan over rejected entried
        np.nan == np.nan returns false so have to use np.isnan(.)
        NB: This check appears redundant as F is calculated to match
        """        
        i1_pts_corr, i2_pts_corr = cv2.correctMatches(F, i1_pts_undistorted, i2_pts_undistorted)
        mask_nan = np.isnan(i1_pts_corr[0])
        i1_pts_corr = np.reshape(i1_pts_corr[0][mask_nan == False], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_corr[0][mask_nan == False], (-1, 2))
        return i1_pts_corr, i2_pts_corr
 
    def rotation_to_euler(self, R):
        """Takes a 3x3 rotation matrix and return success, euler-angles.
        Angles are constrained to be the smallest possible for given R
        Returns success=false on gimbal lock"""
        """Based on 'Computing Euler angles from a rotation matrix' - Gregory
        G. Slabaugh"""
        
        # Check for gimbal lock (i.e. cos(theta) = 0)
        if ((R[2,0] == 1) or (R[2,0] == -1)):
            print "Gimbal lock. Rotation un-resolvable"
            return False, None
        
        theta = np.array([[-np.arcsin(R[2,0])],
                         [np.pi+np.arcsin(R[2,0])]])
                         
        psi = np.arctan2(R[2,1]/np.cos(theta),  R[2,2]/np.cos(theta))
        
        phi = np.arctan2(R[1,0]/np.cos(theta),  R[0,0]/np.cos(theta))
        
        angles = np.hstack((psi, theta, phi))
        
        if angles[0].T.dot(angles[0]) > angles[1].T.dot(angles[1]):
            #print "angles : ", angles[1]
            return True, angles[1]
            
        else:
            #print "angles : ", angles[0]
            return True, angles[0]
            
    def coord_image_to_drone_axis(self, angles):
        drone_angles = angles.copy()
        drone_angles[0] = angles[2]
        drone_angles[1] = -angles[0]
        drone_angles[2] = -angles[1]
        return drone_angles
        
    def coord_drone_to_image_axis(self, angles): #verified
        image_angles = angles.copy()
        image_angles[0] = -angles[1]
        image_angles[1] = -angles[2]
        image_angles[2] = angles[0]
        return image_angles
        
        
    def publish_cloud(self, points, timestamp):
        """ Builds and publishes absolute and relative point clouds"""
        
        """====================================================================
        # Absolute Point Cloud
        ===================================================================="""
        cloud = PointCloud()
        cloud.header.stamp = timestamp
        cloud.header.frame_id = "/world"
        
        
        #print "Pre-shift points:\r\n ", points
        sub = np.add(points.T, self.position_i2).T
        sub = tf.transformations.quaternion_matrix(self.quat_i_to_w2)[:3,:3].dot(sub)
        #print "Post-shift points:\r\n ", sub
        
        
        if (self.descriptor_buffer == None):
            self.descriptor_buffer = self.desc_triangulated
        else:
            self.descriptor_buffer = np.vstack((self.descriptor_buffer, self.desc_triangulated))
        # Descriptor buffer has 1 row per descriptor
        
        if (self.location_buffer == None):
            self.location_buffer = sub
        else:
            self.location_buffer = np.hstack((self.location_buffer, sub))
        # Location buffer has 1 column per 3D location
        # Each row in desc buffer corresponds to a loc column (ordered)
        
        
        # Reshape for easy clouding
        sub = zip(*np.vstack((sub[0], sub[1], sub[2])))

        # Build absolute cloud
        for i, p in enumerate(sub):
            cloud.points.append(gm.Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        self.cloud_pub.publish(cloud)
        
        """====================================================================
        # Relative Point Cloud
        ===================================================================="""        
        cloud = PointCloud()
        cloud.header.stamp = timestamp
        cloud.header.frame_id = "/ardrone_base_frontcam"
        # Reshape for easy clouding
        sub = zip(*np.vstack((points[0], points[1], points[2])))
        
        # Build relative cloud
        for i, p in enumerate(sub):
            cloud.points.append(gm.Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        self.cloud_pub2.publish(cloud) 
        
        print "Cloud  of ", len(cloud.points), "Published"
        
    def world_to_pixel_distorted(self, pts, R, t, K=None, k=None):
        """Takes 3D world co-ord and reverse projects using K and distCoeffs"""
        
        if k == None:
            k = self.distCoeffs
        #print "k : \r\n", k
        k1 = k[0][0] # Radial coeff 1
        k2 = k[0][1] # Radial coeff 2
        p1 = k[0][2] # Tangential coeff 1
        p2 = k[0][3] # Tangential coeff 2
        
            
        if K == None:
            K = self.cameraMatrix
        fx = K[0,0]
        fy = K[1,1]
        cx = K[0,2]                    
        cy = K[1,2]
        
        #print "k : \r\n", len(k)
        #print "pts : \r\n", pts
        
        # Set up projection from R and t                   
        P =  np.diag([1.,1.,1.,1.])
        Pinner = np.hstack((R, t))
        P[:3, :4] = Pinner
        
        # Resolve to world Frame
        sub = P.dot(pts)[:3]
        
        # First order
        x_dash = sub[0]/sub[2]
        #print "x_dash : \r\n", x_dash
        y_dash = sub[1]/sub[2]
        
        # Precalc terms (Note: This is significantly simpler than if we had all 8 distCoeffs)
        r_2 = x_dash*x_dash + y_dash*y_dash
        r_4 = r_2*r_2
        terms = (1+k1*r_2+k2*r_4)
        
        # Second Order
        x_dash2 = x_dash*terms+2*p1*x_dash*y_dash+p2*(r_2+2*x_dash*x_dash)
        y_dash2 = y_dash*terms+2*p2*x_dash*y_dash+p1*(r_2+2*y_dash*y_dash)
        
        # To pixel
        u = fx*x_dash2+cx
        v = fy*y_dash2+cy
        
        return np.array([u,v]).T

    def load_frame(self, frame_no):
        """Loads a frame and extracts features"""
        
        if (frame_no < 1 or frame_no > 2):
            print "Invalid frame number : ", frame_no
            return
        
        
        frame = cv2.cvtColor(self.image_buffer, cv2.COLOR_BGR2GRAY)
        # There appears to be no way to specify the bound on no of 
        # features detected. May be able increase no by running four
        # times; once on each quadrant either by masking (supported by
        # cv2 as an additional parameter) or by actually splitting the
        # image
        pts = self.fd.detect(frame)
        kp, desc = self.de.compute(frame, pts)
        
        if frame_no == 1:
            self.grey_previous = frame
            self.pts1 = pts
            self.kp1 = kp
            self.desc1 = desc
            self.time1 = self.time_buffer
            self.position_from_cloud()
        else:
            self.grey_now = frame
            self.pts2 = pts
            self.kp2 = kp
            self.desc2 = desc
            self.time2 = self.time_buffer
            
    def load_tf(self, frame_no):
        """Loads pose for specified frame"""
        
        if (frame_no < 1 or frame_no > 2):
            print "Invalid frame number : ", frame_no
            return
        
        rospy.sleep(0.1)
        
        """====================================================================
        # World co-ordinate handling (note -p1 != pi)
        ===================================================================="""
        self.tf.waitForTransform("world", "ardrone_base_frontcam", self.time_buffer, rospy.Duration(16))
        # Get tf lookup in reverse frame, this ensures translation is in world axis
        position_w, q1 = self.tf.lookupTransform( "world", "ardrone_base_frontcam",self.time_buffer)
        position_w = np.array((position_w))
        # Flip quat to origin-to-drone-image
        quaternion_i_to_w = q1
        quaternion = tf.transformations.quaternion_inverse(q1)
        
        """====================================================================
        # Image co-ordinate handling
        ===================================================================="""
        
        self.tf.waitForTransform("ardrone_base_frontcam", "world", self.time_buffer, rospy.Duration(16))
        position_i, qi = self.tf.lookupTransform("ardrone_base_frontcam", "world", self.time_buffer)
        position_i = -np.array((position_i))
        quaternion_w_to_i = qi
        qi = tf.transformations.quaternion_inverse(qi)
        
        
        if frame_no == 1:
            self.position_w1 = position_w
            self.quat_w_to_i1 = quaternion_w_to_i
            self.position_i1 = position_i
            self.quat_i_to_w1 = quaternion_i_to_w
            self.quaternion1 = quaternion
            self.image_quaternion1 = qi
        else:
            self.position_w2 = position_w
            self.quat_w_to_i2 = quaternion_w_to_i
            self.position_i2 = position_i
            self.quat_i_to_w2 = quaternion_i_to_w
            self.quaternion2 = quaternion
            self.image_quaternion2 = qi
            
    def load_through(self):
        """Shifts the stored frame2 data to frame1. Would typically be used in
        sequential handing"""
        # Shift image
        self.grey_previous = self.grey_now
        self.pts1 = self.pts2
        self.kp1 = self.kp2
        self.desc1 = self.desc2
        self.time1 = self.time2
        # Shift tf
        self.position_w2 = self.position_w2
        self.quat_w_to_i1 = self.quat_w_to_i2
        self.position_i1 = self.position_i2
        self.quat_i_to_w1 = self.quat_i_to_w2
        self.quaternion1 = self.quaternion2
        self.image_quaternion1 = self.image_quaternion2

    def get_change_in_tf(self):       
        
        # Rotate frame2 position into frame1 image co-ordinates
        R = tf.transformations.quaternion_matrix(self.quat_w_to_i2)[:3, :3]
        position_i1_i2 = R.dot(self.position_w1)
        
        # Difference in position in image (frame1) co-ordinates
        trans = np.array(([(position_i1_i2[0] - self.position_i2[0])],
                          [(position_i1_i2[1] - self.position_i2[1])],
                          [(position_i1_i2[2] - self.position_i2[2])]))
        self.image_coord_trans = np.array([trans[0], trans[1], trans[2]])
        
        
        # Get relative quaternion
        # qmid = qafter.qbefore-1
        self.relative_quat = tf.transformations.quaternion_multiply(self.quat_i_to_w1, tf.transformations.quaternion_inverse(self.quat_i_to_w2))
        self.relative_quat2 = np.array([tf.transformations.euler_from_quaternion(self.relative_quat)]).T
        self.relative_quat[0] = -self.relative_quat2[1]
        self.relative_quat[1] = -self.relative_quat2[2]
        self.relative_quat[2] = self.relative_quat2[0]
        self.relative_quat = tf.transformations.quaternion_from_euler(self.relative_quat[0],self.relative_quat[1],self.relative_quat[2])
   
    def position_from_cloud(self):
        """Attempts to locate world position by matching observed image points to 
        previously triangulated points"""
        
        if self.descriptor_buffer == None:
            print "No previously localised points\r\n"
            return
        
        i1_pts_spec, i2_pts_spec = self.match_known_points(self.kp1, self.desc1)
        print "Cross matches: ", len(i1_pts_spec)
        
        #print i1_pts_spec
        #print i2_pts_spec
        
        """================================================================
        # Calculate pose and translation to matched template
        ================================================================"""
        
        if len(i1_pts_spec) < 5:
            "Not enough matches to localise"
            return
        
        print i2_pts_spec
        # Flip triangulated 3D points into image axis order but world origin
        #i2_pts_image = np.array([-i2_pts_spec.T[1],-i2_pts_spec.T[2], i2_pts_spec.T[0]], dtype=np.float32).T
        #print i2_pts_image
        
        R, t, inliers = cv2.solvePnPRansac(np.array(i2_pts_spec, dtype=np.float32), np.array(i1_pts_spec, dtype=np.float32), self.cameraMatrix, self.distCoeffs)
        
        
        if inliers == None:
            print "===================="
            print "Template not found"
            print "===================="
            return
        
        
        print "No. of inliers: ", len(inliers)    
        
        
        R, J = cv2.Rodrigues(R)        
        Rhomo = np.diag((1., 1., 1., 1.))
        Rhomo[:3, :3] = R
        
        t_position = R.dot(t)
        print "t: ", t_position
        
        success, angles = self.rotation_to_euler(R)
        angles*=180/np.pi
        
        poseStamped = gm.PoseStamped()
        header = Header()
        pose = gm.Pose()
        point = gm.Point()
        orientation = gm.Quaternion() 
        
        # Publish stamped pose
        # Note we need to flip back into world axis
        point.x = t_position[0]
        point.y = t_position[1]
        point.z = t_position[2]
        pose.position = point
        quat = tf.transformations.quaternion_inverse(tf.transformations.quaternion_from_matrix(Rhomo))
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]
        pose.orientation = orientation
        header.stamp = self.time1
        header.frame_id = '/world'
        poseStamped.pose = pose
        poseStamped.header = header
        
        self.pose_pub.publish(poseStamped)
        
        
        '''
        x_homo = self.make_homo(i1_pts_spec)
        X_homo = self.make_homo(i2_pts_spec)
        print x_homo
        print X_homo
        
        # Note using a list of np arrays defeats the point of using np
        X = X_homo[0:5]
        #X = []
        #X.append(X_homo[0])
        #X.append(X_homo[1])
        #X.append(X_homo[2])
        #X.append(X_homo[3])
        #X.append(X_homo[4])
        print "X:\r\n", X
        
        x = self.inverseCameraMatrix.dot(x_homo[0:5].T)
        #x = []
        #x.append(x_homo[0:1].T)
        #x.append(x_homo[1:2].T)
        #x.append(x_homo[2:3].T)
        #x.append(x_homo[3:4].T)
        #x.append(x_homo[4:5].T)
        print "x:\r\n", x
        print x[:,0:1]
        
        
        
        # Set up point lists
        ###NOT COMPLETE
        
        # Compose simultaneous equations
        A = np.zeros((15,14))
        # X is a list of 1x4 homogeneous 3D co-ords
        # x is a list of 3x1 homogenous 2D co-ords pre-multipled by K^-1
        for i in range(5):
            print i
            A[3*i      , 0  :   4] = X[i]
            A[3*i+1    , 4  :   8] = X[i]
            A[3*i+2    , 8  :   9] = 1.
            A[3*i:3*i+3, i+9:i+10] = -x[:,i:i+1]
        
        print "A:\r\n", A
        
        
        # Solve
        U,S,V = np.linalg.svd(A)
        print "S:\r\n", S
        print "V(last):\r\n", V[-1]
        
        # Reconstruct R
        r1 = V[-1,0:3]
        r2 = V[-1,4:7]
        print "r1: ", r1
        print "r2: ", r2
        r3 = np.cross(r1,r2)
        R = np.vstack((np.vstack((r1, r2)), r3))
        
        # Reconstruct t
        t = np.array([[V[-1, 3]], [V[-1, 7]], [V[-1, 8]]])
        
        
        print "======================="
        print "Localised with point cloud"
        print "======================"
        print "World to image rotation (needs axis swap): \r\n", R
        t = -t
        print "World co-ordinates = \r\n", t
        print "Dead reckoned co-ordinates = \r\n", self.position_w1
        
        print "Function incomplete"
        return
        '''
            
    def process_frames(self):
        
        
        """Takes a cv2 numpy array image and compared to a previously
        buffered image. Features are extracted from each frame, 
        undistorted and matched. The matches are then triangulated"""
        
        self.debug_text = []
        self.upper_debug_text = []
        
        grey_previous = self.grey_previous
        grey_now = self.grey_now
        
        """====================================================================
        Rotation and translation from navdata
        ===================================================================="""
        self.get_change_in_tf()
        
        
        """====================================================================
        Find matched points in both images
        ===================================================================="""
        print "No of features 1: ", len(self.kp1)
        print "No of features 2: ", len(self.kp2)
        i1_pts, i2_pts = self.match_points(self.kp1, self.kp2, self.desc1, self.desc2)
        i1_pts_draw = i1_pts
        i2_pts_draw = i2_pts
        if i1_pts == None or len(i1_pts) < 8:
            return
        
        #Check for cross-matching with previously triangulated points
        if self.descriptor_buffer != None:
            i1_pts_spec, i2_pts_spec = self.match_known_points(self.kp1, self.desc)
            print "Frame-frame matches already triangulated: ", len(i1_pts_spec)
    
        print len(i1_pts), " matched points"

        """====================================================================
        Undistort points using known camera calibration
        ==================================================================="""
        if self.calibrated:
            # Undistort points using calibration data    
            i1_pts_undistorted = cv2.undistortPoints(np.array([i1_pts]), self.cameraMatrix, self.distCoeffs, P=self.cameraMatrix)[0]
            i2_pts_undistorted = cv2.undistortPoints(np.array([i2_pts]), self.cameraMatrix, self.distCoeffs, P=self.cameraMatrix)[0]
        else:
            print "WARNING: No calibration info. Cannot Continue"
            return       

        
        """============================================================
        Extract F and filter outliers
        ============================================================"""
        F, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr= self.extract_fundamental(i1_pts_undistorted, i2_pts_undistorted, i1_pts_draw, i2_pts_draw)
        if (i1_pts_corr == None or len(i1_pts_corr) < 1):
            print "No inliers consistent with F"
            return        
            
        print len(i1_pts_corr), " F fitted points"
            
        """============================================================
        # Extract P1 and P2 via E
        ============================================================"""
        success, P1, P2, i1_pts_final, i2_pts_final, i1_pts_draw_final, i2_pts_draw_final = self.extract_projections(F, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr)
        if not success: # Bottom out on fail
            return
        
        self.image_P1 = P1
        self.image_P2 = P2
        
        R = P2[:,:3]
        self.image_based_R2 = R
        #Rhomo = np.diag((1., 1., 1., 1.))
        #Rhomo[:3, :3] = R
        #self.debug_text.append("iR: "+str(np.array([tf.transformations.euler_from_matrix(R)])*180./np.pi))
        t = P2[:,3:4]
        self.image_based_t = t
        
        self.prev_i1_pts_final = i1_pts_final
        self.prev_i2_pts_final = i2_pts_final
        
        self.tf_triangulate_points(i1_pts_final, i2_pts_final, F)
        #self.tf_triangulate_points(i1_pts_corr, i2_pts_corr, F)
        
        print len(i1_pts_final), " E fitted points"
        
        
        """====================================================================
        # Plot fully tracked points
        # Only that fit with the calculated geometry are plotted
        # Note: This plots undistorted points on the distorted image
        ===================================================================="""
        img2 = stackImagesVertically(grey_previous, grey_now)
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        imh = grey_previous.shape[0]
        county = 0
        l = 120
        
        # Draw lines linking fully tracked points
        for p1, p2 in zip(i1_pts_draw_final, i2_pts_draw_final):
            county += 1
            cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
        
        # Write debug text
        for i, l in enumerate(self.debug_text):
            cv2.putText(img2, str(l), (25,img2.shape[0]-25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        for i, l in enumerate(self.upper_debug_text):
            cv2.putText(img2, str(l), (25,25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        
        # Reproject triangulated points
        for p in self.reprojected_frame1:
            cv2.circle(img2, (int(p[0]),int(p[1])), 3, (255, 0, 0), 1)
            #cv2.circle(img2, (int(p[0]),int(p[1])), 3, (255, 63, 63), 1)
        for p in self.reprojected_frame2:
            cv2.circle(img2, (int(p[0]),int(p[1]+imh)), 3, (0, 255, 0), 1)
            #cv2.circle(img2, (int(p[0]),int(p[1])), 3, (255, 63, 63), 1)
        
        # Draw
        cv2.imshow("track", img2)
        cv2.imwrite("temp.jpg", img2)
        print "=============\r\nDrawn\r\n============="
        
        # Render Windows
        cv2.waitKey(10)

    def tf_triangulate_points(self, pts1, pts2, F):
        """ Triangulates 3D points from set of matches co-ords using relative
        camera position determined from tf"""
        
        print pts1
        print pts2
        
        
        # ---------------------------------------------------------------------
        # Working in image co-ordinates throughout
        # ---------------------------------------------------------------------
        
        # These are arrays of the triangulated points reprojected back to the
        # image plane
        self.reprojected_frame1 = []
        self.reprojected_frame2 = []
        
        
        # Bottom out on first frame
        if self.relative_quat == None:
            return
    
        
        # Note: that the translations and rotations are reversed: 
        # While the camera has moved by [R|t], the shift seen from the points
        # is the inverse
        
        # Get t from tf data
        t = -self.image_coord_trans
        t_mag = np.sqrt(t.T.dot(t))
        self.debug_text.append("trans: "+str(t))
        
        # Get rotation matrix
        R = tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(self.relative_quat))[:3, :3]     
        angles = np.array(tf.transformations.euler_from_quaternion(self.relative_quat))
        angles = angles*180./np.pi
        self.debug_text.append("rot: "+np_to_str(angles))  
        
        # Bottom out if motion is insufficient
        mag = np.sqrt(t[0]*t[0]+t[1]*t[1])
        if mag < 0.2:
            print "Motion bad for triangulation :\r\n", str(t)
            return None  
        
        
        # Compare image translation with dead reckoned        
        ti = np.array([self.image_P2[:, 3]]).T
        # Find angle between vectors ( |a||b|cos(theta) = a . b )
        theta = (180./np.pi)*math.acos(t.T.dot(ti)/t_mag)
        print "theta ", theta
        
        # Compose image projection matrix
        Ri = self.image_P1[:3, :3]
        PP1 = np.hstack((Ri, t_mag*ti))        
        print "Image P2:\r\n", PP1
        
        # Compose dead reckoned projection matrix
        P_cam1_to_cam2 = np.hstack((R, t))        
        print "Dead P:\r\n", P_cam1_to_cam2
        
        
        """
        Triangulate using pixel co-ord and K[R t]
        """
        # Factor in camera calibration
        PP2 = np.hstack((self.cameraMatrix, np.array([[0.],[0.],[0,]])))
        PP1 = self.cameraMatrix.dot(P_cam1_to_cam2)
        # Print pre-triangulated count
        pre_triangulated = len(pts1)+0.
        print "Pre-triangulated: ", pre_triangulated
        self.debug_text.append(pre_triangulated)
        # Triangulate
        points3D_image = self.triangulate_points(pts1.transpose(), pts2.transpose(), PP1, PP2)
        if points3D_image == None or len(points3D_image) == 0:
            return            
        points3D_image = points3D_image[:3]        
        
        # Filter points that are behind the camera
        infront = points3D_image[2] > 0
        infront = np.array([infront]).transpose()
        points3D_image = np.reshape(points3D_image[np.hstack((infront, infront, infront)).T==True], (3, -1))
        # Filter descriptors
        self.desc_triangulated = np.reshape(self.desc_accepted[np.resize(infront.T, (self.desc_accepted.shape[1], self.desc_accepted.shape[0])).T==True], (-1, self.desc_accepted.shape[1]))
        if points3D_image == None or len(points3D_image) == 0:
            "No points infront of camera"
            return
        
        # Output number of forward triangulated points
        forward_triangulated = len(points3D_image[0])+0.
        self.debug_text.append(forward_triangulated)
        print "Forward triangulated: ", forward_triangulated    
        
        sq = np.square(points3D_image)
        sq = np.sqrt(sq[0]+sq[1]+sq[2])
        sq_sum = np.sum(sq)
        avg = sq_sum/points3D_image.shape[1]
        if avg > 5.:
            "Points too far"
            return
        
        reasonable = sq < 4.
        reasonable = np.array([reasonable]).transpose()
        points3D_image= np.reshape(points3D_image[np.hstack((reasonable, reasonable, reasonable)).T==True], (3, -1))
        self.desc_triangulated = np.reshape(self.desc_triangulated[np.resize(reasonable.T, (self.desc_triangulated.shape[1], self.desc_triangulated.shape[0])).T==True], (-1, self.desc_triangulated.shape[1]))
        if points3D_image == None or len(points3D_image) == 0:
            return
        
        
        # Filter points that are too far in front
        infront = points3D_image[2] < 5.        
        infront = np.array([infront]).transpose()
        points3D_image= np.reshape(points3D_image[np.hstack((infront, infront, infront)).T==True], (3, -1))
        # Filter descriptors
        self.desc_triangulated = np.reshape(self.desc_triangulated[np.resize(infront.T, (self.desc_triangulated.shape[1], self.desc_triangulated.shape[0])).T==True], (-1, self.desc_triangulated.shape[1]))
        if points3D_image == None or len(points3D_image) == 0:
            return
        
        
        # Triangulated points reprojected back to the image plane
        self.reprojected_frame1 = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, R, t)        
        self.reprojected_frame2 = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, np.diag((1.,1.,1.)), np.array([[0.,0.,0.]]).T)
        
        # Publish Cloud
        # Note: img2 camera is taken to be the origin, so time2 is used
        if forward_triangulated != 0 and (float(forward_triangulated)/pre_triangulated > 0.0):
            print "Publishing Point cloud---------------------------------------------------------"
            self.publish_cloud(points3D_image, self.time2)
        else:
            print "Poor triangulation"
            return
            
    def find_fundamental_from_proj(self, P1, P2, f0 = 640.):
        F = np.diag((0.,0.,0.))
        W = np.diag((0.,0.,0., 0.))
        
        Pk = np.diag((1., 1., f0))
        P1 = Pk.dot(P1)
        P2 = Pk.dot(P2)
        
        W[0] = P1[1]
        W[1] = P1[2]
        W[2] = P2[1]
        W[3] = P2[2]
        F[0,0] = np.linalg.det(W)
        
        W[0] = P1[1]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[2]
        F[0,1] = -np.linalg.det(W)
        
        W[0] = P1[1]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[1]
        F[0,2] = np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[2]
        W[2] = P2[1]
        W[3] = P2[2]
        F[1,0] = -np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[2]
        F[1,1] = np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[1]
        F[1,2] = -np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[1]
        W[2] = P2[1]
        W[3] = P2[2]
        F[2,0] = np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[1]
        W[2] = P2[0]
        W[3] = P2[2]
        F[2,1] = -np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[1]
        W[2] = P2[0]
        W[3] = P2[1]
        F[2,2] = np.linalg.det(W)
        
        
        # Normalise
        norm = np.sqrt(F[0,0]*F[0,0]+F[0,1]*F[0,1]+F[0,2]*F[0,2]+F[1,0]*F[1,0]+F[1,1]*F[1,1]+F[1,2]*F[1,2]+F[2,0]*F[2,0]+F[2,1]*F[2,1]+F[2,2]*F[2,2])
        F = F/norm
        
        
        return F
       
    def optimal_correction_triangulate_point(self, xvector1, xvector2, F, f0 = 640.):
        """This carries out the Optimal Correction Method outlined by Kanatami 
        et al in 'Triangulation from Two Views Revisited: Hartley-Sturm vs. 
        Optimal Correction'. This particular iterative method is significantly
        faster than Hartley-Sturm's and is stable in all cases.
        
        This method shifts the pixel locations the minimal distance such that 
        they both satisfy the epipolar constraint. This has the implicit effect
        of causing the triangulation to intersect exactly
        
        x1 and x2 are pixel co-ord homogenous matched points
        F is the fundamental
        f0 is a scale factor for numerical stability and ~ image width (pixels)
        """
        
        # NOTE: The x1, x2 vectors are different to those in the paper.
        # The paper uses x_vector = [ x/f0 ]   This uses:  [ x ]
        #                           [ y/f0 ]               [ y ]
        #                           [ 1    ]               [ 1 ]
        # This simply means some scalings f0 scaling are skipped
        
        
        # Flatten fundamental
        u = np.reshape(F, (1,-1))[0]
        
        
        # Precalc terms that do not change in iteration
        f0f0 = f0 * f0
        f10 = u[2]*f0
        f11 = u[5]*f0
        f20 = u[6]*f0
        f21 = u[7]*f0        
        uu = np.array([u[0]*u[0], u[1]*u[1], u[2]*u[2], u[3]*u[3], u[4]*u[4], u[5]*u[5], u[6]*u[6], u[7]*u[7]])
        u0u1 = u[0]*u[1]
        u0u2 = u[0]*u[2]
        u0u3 = u[0]*u[3]
        u0u6 = u[0]*u[6]
        u1u2 = u[1]*u[2]
        u1u4 = u[1]*u[4]
        u1u7 = u[1]*u[7]
        u3u4 = u[3]*u[4]
        u3u5 = u[3]*u[5]
        u3u6 = u[3]*u[6]
        u4u5 = u[4]*u[5]
        u4u7 = u[4]*u[7]
        
        # P is a vector of elementwise products of Xihat and u
        # Xihat is never individually calculated
        P8 = f0f0*u[8]
        
        # Q is the sum of constant diagonal terms in V0[Xi]
        Qconst = f0f0*(uu[2] + uu[5] + uu[6] + uu[7])
        
        
        
        # Convergence target.
        # ie. minimal error reduction considered to justify another iteration
        # Should be multiplied by f0f0 wrt actual desired limit
        convergence = 1.0e-8
        
        # Initialise values
        # E0 is initialised to ~ infinity
        # Should be multiplied by f0f0 wrt actual nominal E0
        E0 = 1.0e+10
        xhat1 = xvector1[0]
        yhat1 = xvector1[1]
        xhat2 = xvector2[0]
        yhat2 = xvector2[1]
        # Original x values are saved in this form to make the code cleaner
        ox1 = xvector1[0]
        oy1 = xvector1[1]
        ox2 = xvector2[0]
        oy2 = xvector2[1]
        xtwiddle1 = 0.
        ytwiddle1 = 0.
        xtwiddle2 = 0.
        ytwiddle2 = 0.
        
        done = False
        iterations = 0
        while(not done):
            iterations = iterations + 1
            # Compute P -> a vector of elementwise products of Xihat and u
            P = np.array([(xhat1*xhat2 + xhat2*xtwiddle1 + xhat1*xtwiddle2)*u[0],
                          (xhat1*yhat2 + yhat2*xtwiddle1 + xhat1*ytwiddle2)*u[1],
                          f0*(xhat1+xtwiddle1)*u[2],
                          (yhat1*xhat2 + xhat2*ytwiddle1 + yhat1*xtwiddle2)*u[3],
                          (yhat1*yhat2 + yhat2*ytwiddle1 + yhat1*ytwiddle2)*u[4],
                          f0*(yhat1+ytwiddle1)*u[5],
                          f0*(xhat2+xtwiddle2)*u[6],
                          f0*(yhat2+ytwiddle2)*u[7],
                          P8])
                             
            # Pre-calc intermediate terms
            xx1 = xhat1*xhat1
            xx2 = xhat2*xhat2
            yy1 = yhat1*yhat1
            yy2 = yhat2*yhat2
            xy1 = xhat1*yhat1
            xy2 = xhat2*yhat2
            f0x1 = f0*xhat1
            f0x2 = f0*xhat2
            f0y1 = f0*yhat1
            f0y2 = f0*yhat2
            
            # Calculate results of diagonal terms of (u, V0[Xi]u)
            diag = np.array([(xx1+xx2)*uu[0], (xx1+yy2)*uu[1], (yy1+xx2)*uu[3], (yy1+yy2)*uu[4], Qconst])
            # Calculate results of unique off-diagonal terms of (u, V0[Xi]u)
            # Matrix is symmetric so each value actually occurs twice
            cross = np.array([xy2*u0u1, f0x2*u0u2, xy1*u0u3, f0x1*u0u6,
                              f0y2*u1u2, xy1*u1u4, f0x1*u1u7,
                              xy2*u3u4, f0x2*u3u5, f0y1*u3u6,
                              f0y2*u4u5, f0y1*u4u7])
            
            # Evaluate (u, Xihat)
            uxi = np.sum(P)
            # Evaluate (u, V0[Xi]u)
            # Non-diagonal terms each occur twice
            uxiu = np.sum(diag) + 2.0*np.sum(cross)
            #      (u, Xihat)
            # C = ------------
            #     (u, V0[Xi]u)
            C = uxi/uxiu
            
            # Update twiddle values
            #
            # [ xtwiddle1 ] = C[ u0 u1 u2 ][ xhat2 ]
            # [ ytwiddle1 ]    [ u3 u4 u5 ][ yhat2 ]
            #                              [ f0    ]
            #
            # [ xtwiddle2 ] = C[ u0 u3 u6 ][ xhat1 ]
            # [ ytwiddle2 ]    [ u2 u5 u8 ][ yhat1 ]
            #                              [ f0    ]            
            xtwiddle1 = C*(u[0]*xhat2 + u[1]*yhat2+f10)
            ytwiddle1 = C*(u[3]*xhat2 + u[4]*yhat2+f11)
            xtwiddle2 = C*(u[0]*xhat1 + u[3]*yhat1+f20)
            ytwiddle2 = C*(u[1]*xhat1 + u[4]*yhat1+f21)
            
            # Evaluate reprojection error
            # This should technically be divided by f0f0 but this is avoided as
            # E is f0f0E0typical and convergence is f0f0convergencetypical
            E = xtwiddle1*xtwiddle1 + ytwiddle1*ytwiddle1 + xtwiddle2*xtwiddle2 + ytwiddle2*ytwiddle2
            
            # Check if iteration reduced error
            if abs(E-E0) > convergence:
                # Update error bound
                E0 = deepcopy(E)
                # Update difference terms
                xhat1 = ox1 - xtwiddle1
                yhat1 = oy1 - ytwiddle1
                xhat2 = ox2 - xtwiddle2
                yhat2 = oy2 - ytwiddle2
            else:
                # No benefit to further iteration
                done = True
                
        new_x1 = np.array((xhat1, yhat1, 1.0))
        new_x2 = np.array((xhat2, yhat2, 1.0))
        
        return np.hstack((new_x1, new_x2, E))
    
    def triangulate_point(self, x1,x2,P1,P2): 
        """ Point pair triangulation from
        least squares solution. """
        # Compose matrix representing simultaneous equations
        M = np.zeros((4,4))
        M[0] = x1[0]*P1[2]-P1[0]
        M[1] = x1[1]*P1[2]-P1[1]
        M[2] = x2[0]*P2[2]-P2[0]
        M[3] = x2[1]*P2[2]-P2[1]
        # Compute SVD
        U,S,V = np.linalg.svd(M)
        
        # numpy SVD is ordered from largest to smallest (for S)
        # so least squares solution will always lie in the last column of V
        # BUT since numpy SVD returns V transpose not V, it is the last row
        X = V[-1,:]
        
        
        # Get projected pixel co-ords
        projected_pixels_homo = P1.dot(V[-1,:])
        projected_pixels = (projected_pixels_homo/projected_pixels_homo[2])[:2]
        # Get dif between proj and image
        error = projected_pixels-x1[:2]
        error_mag1 = np.sqrt(error[0]*error[0]+ error[1]*error[1])
        
        # Same for frame 2
        projected_pixels_homo = P2.dot(V[-1,:])
        projected_pixels = (projected_pixels_homo/projected_pixels_homo[2])[:2]
        error = projected_pixels-x2[:2]
        error_mag2 = np.sqrt(error[0]*error[0]+ error[1]*error[1])
        
        # max is more useful than average as we want a good fit to both
        error_max = max(error_mag1, error_mag2)        
        
        return np.hstack((X / X[3], error_max))
    
        
    
    def unfiltered_triangulate_points(self, x1,x2,P1,P2):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""
        
        F = self.find_fundamental_from_proj(P1, P2)
        n = x1.shape[1]
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        # Correct points
        corr = np.array([self.optimal_correction_triangulate_point(x1[:,i],x2[:,i], F) for i in range(n)])
        corr1 = corr[:, :3].T
        corr2 = corr[:, 3:6].T
        # Triangulate for each pair
        Combi = np.array([self.triangulate_point(corr1[:,i],corr2[:,i],P1,P2) for i in range(n)]) # Looping here is probably unavoidable
        # Extract 4D points
        X = Combi[:,:4]
        return X.T
    
    def triangulate_points(self, x1,x2,P1,P2, max_error = 10., max_squared_error = 100.):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""
        
        F = self.find_fundamental_from_proj(P1, P2)
        
        n = x1.shape[1]
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        # Correct points
        corr = np.array([self.optimal_correction_triangulate_point(x1[:,i],x2[:,i], F) for i in range(n)])
        corr1 = corr[:, :3].T
        corr2 = corr[:, 3:6].T
        errors = corr[:, 6:7]
        
        shift = x1 - corr1
        shift = shift*shift
        shift1 = shift[0]+shift[1]
        
        shift = x2 - corr2
        shift = shift*shift
        shift2 = shift[0]+shift[1]
        
        accepted = np.logical_and(shift1<max_squared_error, shift2<max_squared_error)
        
        # Filter points with too much F implied shift
        accepted = np.array([accepted]).T
        mask_prepped = np.resize(accepted.T, (self.desc_final.shape[1],self.desc_final.shape[0])).T
        accepted = np.hstack((accepted, accepted, accepted)).T
        x1 = np.reshape(x1[accepted==True], (3, -1))
        x2 = np.reshape(x2[accepted==True], (3, -1))
        # Filter descriptors
        self.desc_shifted = np.reshape(self.desc_final[mask_prepped==True], (-1, self.desc_final.shape[1]))
        
        n = len(x1.T)        
        if x1 == None or n <= 1:
            print "No points correctable"
            return None
            
        self.corr_triangulated = n+0.
        print "Corr-triangulated: ", self.corr_triangulated
        self.debug_text.append(self.corr_triangulated)
        
        
        # Triangulate for each pair
        Combi = np.array([self.triangulate_point(x1[:,i],x2[:,i],P1,P2) for i in range(n)]) # Looping here is probably unavoidable
        # Extract 4D points
        X = Combi[:,:4]        
        # Create mask
        accepted = Combi[:, 4] < max_error
        
        # Filter points with too low accuracy (reprojected to actual image)
        accepted = np.array([accepted]).T
        mask_prepped = np.resize(accepted.T, (self.desc_shifted.shape[1],self.desc_shifted.shape[0])).T
        accepted = np.hstack((accepted, accepted, accepted)).T
        X = np.reshape(X.T[accepted==True], (3, -1))
        # Filter descriptors
        self.desc_accepted = np.reshape(self.desc_shifted[mask_prepped==True], (-1, self.desc_shifted.shape[1]))
        
        if X == None or len(X) == 0:
            print "No points triangulatable"
            return None
        
        self.triangulated = len(X[0])+0.
        print "Triangulated: ", self.triangulated
        self.debug_text.append(self.triangulated)
        
        print X
        
        return X
        
        
    def imgproc(self, d):
        """Converts the ROS published image to a cv2 numpy array"""
        
        self.time_now = d.header.stamp
        
        # ROS to cv image
        bridge = CvBridge()
        cvimg = bridge.imgmsg_to_cv(d,"bgr8")
        # cv to cv2 numpy array image
        npimg = np.asarray(cvimg)
        # Pass to FeatureTracker
        self.image = npimg
        
    def image_flip(self):
        """Temporarily fixes the images"""
        self.image_buffer = self.image.copy()
        self.time_buffer = deepcopy(self.time_now)
        
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        if not self.calibrated:
            if len(ci.D) == 0:
                return
            self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
            self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
            self.distCoeffs = np.array([ci.D], dtype=np.float32)
            self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
            self.calibrated = True    
            print "Calibration Initialised"


def run():
    rospy.init_node('Scan_Controller')
    # Initialise controller
    s = ScanController()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
