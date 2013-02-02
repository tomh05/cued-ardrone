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
        print "diff: ", diff
        if (mag > 0.2):
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
        self.prev_position = None
        self.prev_quat = None
        self.prev_prev_position = None
        self.mag_dist = None
        self.speech_limiter = 0
        self.corners = None
        self.time_prev = None
        
        self.image = None
        
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
        
        return i1_pts, i2_pts
        
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
        i1_pts_masked = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        i1_pts_undistorted = np.array([i1_pts_masked])
        i2_pts_undistorted = np.array([i2_pts_masked])
        
        i1_pts_masked = np.reshape(i1_pts_draw[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_draw[mask_prepped==1], (-1, 2))
        i1_pts_draw_corr = np.array([i1_pts_masked])
        i2_pts_draw_corr = np.array([i2_pts_masked])
        
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
        i1_pts_corr_norm = self.inverseCameraMatrix.dot(self.make_homo(i1_pts_corr).transpose()).transpose()[:,:2]
        i2_pts_corr_norm = self.inverseCameraMatrix.dot(self.make_homo(i2_pts_corr).transpose()).transpose()[:,:2]
                              
        ind = 0
        maxfit = 0
        secfit = 0
        for i, P2 in enumerate(projections):
            # infront accepts only both dimensions
            # WARNING: cv2.tri produces unnormalised homo coords
            points4D = cv2.triangulatePoints(P1, P2, i1_pts_corr_norm.transpose(), i2_pts_corr_norm.transpose())
            # normalise homogenous coords
            points4D /= points4D[3]
            #points4D = self.triangulate_points(i1_pts_corr.transpose(), i2_pts_corr.transpose(), P1, P2)
            d1 = np.dot(P1,points4D)[2]
            d2 = np.dot(P2,points4D)[2]
            PI = sum((d1>0) & (d2>0))
            #print "Support for P2 ", i, " : ", PI
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
        infront = np.append(infront, infront, 1)
        i1_pts_final = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_final = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        i1_pts_draw_final = np.reshape(i1_pts_draw_corr[infront==True], (-1, 2))
        i2_pts_draw_final = np.reshape(i2_pts_draw_corr[infront==True], (-1, 2))
        
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
        sub = np.add(points.T, self.position_i1).T
        sub = tf.transformations.quaternion_matrix(self.quat_i_to_w1)[:3,:3].dot(sub)
        #print "Post-shift points:\r\n ", sub
        
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
            self.time_prev = self.time_now
        else:
            self.grey_now = frame
            self.pts2 = pts
            self.kp2 = kp
            self.desc2 = desc
            
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
        
        self.tf.waitForTransform("ardrone_base_frontcam", "world", self.time_now, rospy.Duration(16))
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
        self.time_prev = self.time_now
        # Shift tf
        self.position_w2 = self.position_w2
        self.quat_w_to_i1 = self.quat_w_to_i2
        self.position_i1 = self.position_i2
        self.quat_i_to_w1 = self.quat_i_to_w2
        self.quaternion1 = self.quaternion2
        self.image_quaternion1 = self.image_quaternion2

    def get_change_in_tf(self):       
        
        # Rotate frame2 position into frame1 image co-ordinates
        R = tf.transformations.quaternion_matrix(self.quat_w_to_i1)[:3, :3]
        position_i2_i1 = R.dot(self.position_w2)
        
        # Difference in position in image (frame1) co-ordinates
        trans = np.array(([(position_i2_i1[0] - self.position_i1[0])],
                          [(position_i2_i1[1] - self.position_i1[1])],
                          [(position_i2_i1[2] - self.position_i1[2])]))
        self.image_coord_trans = np.array([trans[0], trans[1], trans[2]])
        
        
        # Get relative quaternion
        # qmid = qafter.qbefore-1
        self.relative_quat = tf.transformations.quaternion_multiply(self.quat_i_to_w2, tf.transformations.quaternion_inverse(self.quat_i_to_w1))
        self.relative_quat2 = np.array([tf.transformations.euler_from_quaternion(self.relative_quat)]).T
        self.relative_quat[0] = -self.relative_quat2[1]
        self.relative_quat[1] = -self.relative_quat2[2]
        self.relative_quat[2] = self.relative_quat2[0]
        self.relative_quat = tf.transformations.quaternion_from_euler(self.relative_quat[0],self.relative_quat[1],self.relative_quat[2])
   
        
            
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
        success, i1_pts, i2_pts = self.find_and_match_points(self.grey_now)
        i1_pts_draw = i1_pts
        i2_pts_draw = i2_pts
        if not success:
            return
    
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
        self.debug_text.append("trans: "+str(t))
        
        
        # Get rotation matrix
        R = tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(self.relative_quat))[:3, :3]     
        angles = np.array(tf.transformations.euler_from_quaternion(self.relative_quat))
        angles = angles*180./np.pi
        self.debug_text.append("rot: "+np_to_str(angles))  
        
        # Bottom out if motion is insufficient
        if abs(t[0]) < 0.1 and abs(t[1]) < 0.1:
            print "Motion bad for triangulation :\r\n", str(t)
            return None        
        
        # Compose projection matrix
        P_cam1_to_cam2 = np.hstack((R, t))
        
        
        """
        Triangulate using pixel co-ord and K[R t]
        """
        # Factor in camera calibration
        PP1 = np.hstack((self.cameraMatrix, np.array([[0.],[0.],[0,]])))
        PP2 = self.cameraMatrix.dot(P_cam1_to_cam2)
        points3D_image, accepted = self.triangulate_points(pts1.transpose(), pts2.transpose(), PP1, PP2, F, max_error = 20.)
        if points3D_image == None or len(points3D_image) == 0:
            return
        points3D_image = points3D_image[:3]
        
        pre_triangulated = len(points3D_image[0])+0.
        print "Triangulated: ", pre_triangulated
        self.debug_text.append(pre_triangulated)
        
        # Filter points with too low accuracy (reprojected to actual image)
        accepted = np.array([accepted]).T
        accepted = np.hstack((accepted, accepted, accepted)).T
        points3D_image = np.reshape(points3D_image[accepted==True], (3, -1))
        if points3D_image == None or len(points3D_image) == 0:
            return
        
        # Output number of triangulated points
        triangulated = len(points3D_image[0])+0.
        print "Accepted Triangulated: ", triangulated
        self.debug_text.append(triangulated)
        print points3D_image
        
        # Filter points that are behind the camera
        infront = points3D_image[2] > 0        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image= np.reshape(points3D_image[infront==True], (3, -1))
        if points3D_image == None or len(points3D_image) == 0:
            return
        
        
        #sq = np.square(points3D_image)
        #sq = np.sqrt(sq[0]+sq[1]+sq[2])
        #sq_sum = np.sum(sq)
        #avg = sq_sum/points3D_image.shape[1]
        #if avg > 5.:
        #    "Points too far"
        
        #reasonable = sq < 4.
        #reasonable = np.array([reasonable]).transpose()
        #reasonable = np.hstack((reasonable, reasonable, reasonable)).transpose()        
        #points3D_image= np.reshape(points3D_image[reasonable==True], (3, -1))
        #if points3D_image == None or len(points3D_image) == 0:
        #    return
        
        '''
        # Filter points that are too far in front
        infront = points3D_image[2] < 4.        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image= np.reshape(points3D_image[infront==True], (3, -1))
        '''
        
        # Triangulated points reprojected back to the image plane
        self.reprojected_frame1 = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, np.diag((1.,1.,1.)), np.array([[0.,0.,0.]]).T)        
        self.reprojected_frame2 = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, R, t)
        print "repro: \r\n", self.reprojected_frame1
        
        # Output number of forward triangulated points
        forward_triangulated = len(points3D_image[0])+0.
        self.debug_text.append(forward_triangulated)
        print "Forward triangulated: ", forward_triangulated
        
        # Publish Cloud
        # Note: img2 camera is taken to be the origin, so time_now is used
        if triangulated != 0 and (float(forward_triangulated)/pre_triangulated > 0.0):
            print "Publishing Point cloud"
            self.publish_cloud(points3D_image, self.time_prev)
        else:
            print "Poor triangulation"
            return
            
    def find_fundamental_from_proj(self, P1, P2, f0 = 600.):
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
       
    def optimal_correction_triangulate_point(self, x1, x2, F, f0 = 600.):
        # Flatten fundamental
        u = np.reshape(F, (1,-1))[0]
        
        
        
        # Precalc constant terms
        f0f0 = f0 * f0
        uu = np.array([u[0]*u[0], u[1]*u[1], u[2]*u[2], u[3]*u[3], u[4]*u[4], u[5]*u[5], u[6]*u[6], u[7]*u[7]])
        
        Q00 = f0f0*(uu[2] + uu[5] + uu[6] + uu[7])
        
        # Accuracy target
        E0 = 1.0e+10#*f0f0
        convergence = 1.0e-6#*f0f0
        
        
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
        
        P9 = f0f0*u[8]
        
        f10 = u[2]*f0
        f11 = u[5]*f0
        f20 = u[6]*f0
        f21 = u[7]*f0
        
        # Initialise values
        xhat1 = x1[0]
        yhat1 = x1[1]
        xhat2 = x2[0]
        yhat2 = x2[1]
        ox1 = x1[0]
        oy1 = x1[1]
        ox2 = x2[0]
        oy2 = x2[1]
        xtwiddle1 = 0.
        ytwiddle1 = 0.
        xtwiddle2 = 0.
        ytwiddle2 = 0.
        
        done = False
        iterations = 0
        while(not done):
            iterations = iterations + 1
            # Compute P
            P = np.array([(xhat1*xhat2 + xhat2*xtwiddle1 + xhat1*xtwiddle2)*u[0],
                          (xhat1*yhat2 + yhat2*xtwiddle1 + xhat1*ytwiddle2)*u[1],
                          f0*(xhat1+xtwiddle1)*u[2],
                          (yhat1*xhat2 + xhat2*ytwiddle1 + yhat1*xtwiddle2)*u[3],
                          (yhat1*yhat2 + yhat2*ytwiddle1 + yhat1*ytwiddle2)*u[4],
                          f0*(yhat1+ytwiddle1)*u[5],
                          f0*(xhat2+xtwiddle2)*u[6],
                          f0*(yhat2+ytwiddle2)*u[7],
                          P9])
                             
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
            
            
            diag = np.array([Q00, (xx1+xx2)*uu[0], (xx1+yy2)*uu[1], f0f0*uu[2], (yy1+xx2)*uu[3], (yy1+yy2)*uu[4], f0f0*uu[5], f0f0*uu[6], f0f0*uu[7]])
            #diag = np.array([Q00, (xx1+xx2)*uu[0], (xx1+yy2)*uu[1], (yy1+xx2)*uu[3], (yy1+yy2)*uu[4]])
            cross = np.array([xy2*u0u1, f0x2*u0u2, xy1*u0u3, f0x1*u0u6,
                              f0y2*u1u2, xy1*u1u4, f0x1*u1u7,
                              xy2*u3u4, f0x2*u3u5, f0y1*u3u6,
                              f0y2*u4u5, f0y1*u4u7])
            
            uxi = np.sum(P)
            uxiu = np.sum(diag) + 2.0*np.sum(cross)
            C = uxi/uxiu
            
            xtwiddle1 = C*(u[0]*xhat2 + u[1]*yhat2+f10)
            ytwiddle1 = C*(u[3]*xhat2 + u[4]*yhat2+f11)
            xtwiddle2 = C*(u[0]*xhat1 + u[3]*yhat1+f20)
            ytwiddle2 = C*(u[1]*xhat1 + u[4]*yhat1+f21)
            
            E = xtwiddle1*xtwiddle1 + ytwiddle1*ytwiddle1 + xtwiddle2*xtwiddle2 + ytwiddle2*ytwiddle2
            
            if abs(E-E0) > convergence:
                E0 = deepcopy(E)
                xhat1 = deepcopy(ox1 - xtwiddle1)
                yhat1 = deepcopy(oy1 - ytwiddle1)
                xhat2 = deepcopy(ox2 - xtwiddle2)
                yhat2 = deepcopy(oy2 - ytwiddle2)
            else:
                done = True
                
        new_x1 = np.array((xhat1, yhat1, 1.0))
        new_x2 = np.array((xhat2, yhat2, 1.0))
        
        return np.hstack((new_x1, new_x2))
        
    def triangulate_point(self, x1,x2,P1,P2): 
        """ Point pair triangulation from
        least squares solution. """
        # Compose matrix representing simultaneous equations
        M = np.zeros((6,6))
        M[:3,:4] = P1
        M[3:,:4] = P2
        M[:3,4] = -x1
        M[3:,5] = -x2
        # Compute SVD
        U,S,V = np.linalg.svd(M)
        # numpy SVD is ordered from largest to smallest (for S)
        # so least squares solution will always lie in the last column of V
        # BUT since numpy SVD returns V transpose not V, it is the last row
        X = V[-1,:4]
        
        print "s1, s2 : ", V[-1, 4], ", ", V[-1, 5]
        
        # Get projected pixel co-ords
        projected_pixels_homo = P1.dot(V[-1,:4])
        projected_pixels = (projected_pixels_homo/projected_pixels_homo[2])[:2]
        # Get dif between proj and image
        error = projected_pixels-x1[:2]
        error_mag1 = np.sqrt(error[0]*error[0]+ error[1]*error[1])
        
        # Same for frame 2
        projected_pixels_homo = P2.dot(V[-1,:4])
        projected_pixels = (projected_pixels_homo/projected_pixels_homo[2])[:2]
        error = projected_pixels-x2[:2]
        error_mag2 = np.sqrt(error[0]*error[0]+ error[1]*error[1])
        
        # max is more useful than average as we want a good fit to both
        error_max = max(error_mag1, error_mag2)        
        
        return np.hstack((X / X[3], error_max))
        
    def triangulate_points(self, x1,x2,P1,P2, F, max_error = 10., max_squared_error = 64.):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""
        
        #F = self.find_fundamental_from_proj(P1, P2)
        #print "Dead reckon F:\r\n", F
        
        #print "P1:\r\n", P1
        #print "P2:\r\n", P2
        
        #P2 = self.cameraMatrix.dot(self.image_P2)
        #P1 = self.cameraMatrix.dot(self.image_P1)
        #R1 = np.linalg.inv(self.image_P1[:3, :3])
        #t1 = np.array([self.image_P1[:, 3]]).T
        #t1[0] = (t1[0]/np.sign(t1[0]))*np.sign(P1[0,3])
        #t1[1] = (t1[1]/np.sign(t1[1]))*np.sign(P1[1,3])
        #t1[2] = (t1[2]/np.sign(t1[2]))*np.sign(P1[2,3])
        #P1 = self.cameraMatrix.dot(np.hstack((R1, t1)))
        #print "P1:\r\n", P1
        #print "P2:\r\n", P2
        #F = self.find_fundamental_from_proj(P1, P2)
        #print "Image F:\r\n", F
        
        n = x1.shape[1]
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        #print "x1:\r\n", x1
        #print "x2:\r\n", x2
        #print "P1:\r\n", P1
        #print "P2:\r\n", P2
        # Correct points
        #corr = np.array([self.optimal_correction_triangulate_point(x1[:,i],x2[:,i], F) for i in range(n)])
        #corr1 = corr[:, :3].T
        #corr2 = corr[:, 3:6].T
        #print "Corr:\r\n", corr
        #print "Corr1:\r\n", corr1
        #print "Corr2:\r\n", corr2
        
        
        #shift = x1 - corr1
        #shift = shift*shift
        #shift1 = shift[0]+shift[1]
        
        #shift = x2 - corr2
        #shift = shift*shift
        #shift2 = shift[0]+shift[1]
        
        #print "shift: \r\n", shift
        #accepted = np.logical_and(shift1<max_squared_error, shift2<max_squared_error)
        
        # Filter points with too much F implied shift
        #accepted = np.array([accepted]).T
        #accepted = np.hstack((accepted, accepted, accepted)).T
        #x1 = np.reshape(x1[accepted==True], (3, -1))
        #x2 = np.reshape(x2[accepted==True], (3, -1))
        #print "x1: \r\n", x1
        
        #n = len(x1.T)
        
        #if x1 == None or n <= 1:
        #    return None, None
            
        
        
        # Triangulate for each pair
        Combi = np.array([self.triangulate_point(x1[:,i],x2[:,i],P1,P2) for i in range(n)]) # Looping here is probably unavoidable
        # Extract 4D points
        X = Combi[:,:4]        
        # Create mask
        accepted = Combi[:, 4] < max_error
        return X.T, accepted
        
        
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
