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
        
        

        
    def manual_scan(self, event):
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
            if (position1[2]-position2[2]> 0.3):
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
            if (position1[2]-position2[2]> 0.3):
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
    
    
    def connect(self):
        
        rospy.Subscriber('/xboxcontroller/button_y',Empty,self.manual_scan)
        self.feature_tracker.tf = tf.TransformListener()
        self.tf = self.feature_tracker.tf
        rospy.Subscriber('/ardrone/front/image_raw',Image,self.feature_tracker.imgproc)
        rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, self.feature_tracker.setCameraInfo)

    

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
        self.cloud_pub = rospy.Publisher('pointCloud', PointCloud)
        self.cloud_pub2 = rospy.Publisher('pointCloud_no_floor', PointCloud)
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
        
        
    def extract_fundamental(self, i1_pts_undistorted, i2_pts_undistorted):
        """
        Extract fundamental matrix and then remove outliers
        FM_RANSAC should be good with lowish outliers
        FM_LMEDS may be more robust in some cases
        """
        temp_time = time.time()
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 1, param2 = 0.99)
        #print "F time : ", time.time()-temp_time
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_masked = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        i1_pts_undistorted = np.array([i1_pts_masked])
        i2_pts_undistorted = np.array([i2_pts_masked])
        
        i1_pts_masked = np.reshape(self.i1_pts_draw[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(self.i2_pts_draw[mask_prepped==1], (-1, 2))
        self.i1_pts_draw = np.array([i1_pts_masked])
        self.i2_pts_draw = np.array([i2_pts_masked])
        
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
        self.i1_pts_draw = np.reshape(self.i1_pts_draw[0][mask_nan == False], (-1, 2))
        self.i2_pts_draw = np.reshape(self.i2_pts_draw[0][mask_nan == False], (-1, 2))
        
        return F, i1_pts_corr, i2_pts_corr
        
    def extract_projections(self, F, i1_pts_corr, i2_pts_corr):
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
            print "P2 not extracted"
            print "==================================================="
            return False, None, None, None, None            
        
        #print "P2"
        #print projections[ind]
        
        # Filter points
        infront = np.array([infront]).transpose()
        infront = np.append(infront, infront, 1)
        i1_pts_corr = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        self.i1_pts_draw = np.reshape(self.i1_pts_draw[infront==True], (-1, 2))
        self.i2_pts_draw = np.reshape(self.i2_pts_draw[infront==True], (-1, 2))
        
        return True, P1, projections[ind], i1_pts_corr, i2_pts_corr
    
    
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
        cloud = PointCloud()
        cloud.header.stamp = timestamp
        cloud.header.frame_id = "/ardrone_base_frontcam" # Should be front camera really
        cloud2 = PointCloud()
        cloud2.header.stamp = timestamp
        cloud2.header.frame_id = "/ardrone_base_frontcam" # Should be front camera really
        
        
        for i, p in enumerate(points): # Ideally done without a loop
            cloud.points.append(gm.Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
            if p[2] > 0: # Only points above drone image axis (crude removing of floor)
                cloud2.points.append(gm.Point32())
                cloud2.points[len(cloud2.points)-1].x = p[0]
                cloud2.points[len(cloud2.points)-1].y = p[1]
                cloud2.points[len(cloud2.points)-1].z = p[2]
        self.cloud_pub.publish(cloud)
        self.cloud_pub2.publish(cloud2)
        
    publish_cloud2(self, points, timestamp):
        cloud2 = PointCloud2()
        cloud2.header = timestamp
        cloud2.width = len(points[0])
        cloud2.height = 1
        cloud2.fields.resize(3)
        cloud2.fields[0].name = 'x'
        cloud2.fields[1].name = 'y'
        cloud2.fields[2].name = 'z'
        offset = 0
        d = 0
        # All offsets are *4, as all field data types are float32
        for (size_t d = 0; d < output.fields.size (); ++d, offset += 4):
            cloud2.fields[d].offset = offset
            cloud2.fields[d].datatype = 7 # FLOAT32 datatype
            cloud2.fields[d].count  = 1
            offset = offset + 4
            d = d + 1
        
         
        
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
        self.tf.waitForTransform("world", "ardrone_base_frontcam", self.time_buffer, rospy.Duration(16))
        
        # Get tf lookup in reverse frame, this ensures translation is in world axis
        p1, q1 = self.tf.lookupTransform( "world", "ardrone_base_frontcam",self.time_buffer)
        position = p1
        # Flip quat to origin-to-drone-image
        quaternion = tf.transformations.quaternion_inverse(q1)
        
        if frame_no == 1:
            self.position1 = position
            self.quaternion1 = quaternion
        else:
            self.position2 = position
            self.quaternion2 = quaternion

    def get_change_in_tf(self):       
        
        
        # Difference in position in world, drone axis
        trans = np.array(([(self.position2[0] - self.position1[0])],
                          [(self.position2[1] - self.position1[1])],
                          [(self.position2[2] - self.position1[2])]))
        
        R = tf.transformations.quaternion_matrix(self.quaternion2)[:3, :3]
        trans = R.dot(trans)
        # Re-order axis into image co-ords
        self.image_coord_trans = np.array([trans[0], trans[1], trans[2]])
        
        # Get relative quaternion
        # qmid = qbefore-1.qafter
        self.relative_quat = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(self.quaternion1), self.quaternion2)
        
   
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
        self.i1_pts_draw = i1_pts
        self.i2_pts_draw = i2_pts
        if not success:
            return
    
        
        """====================================================================
        Undistort points using known camera calibration
        ==================================================================="""
        if self.calibrated:
            # Undistort points using calibration data    
            i1_pts_undistorted = cv2.undistortPoints(np.array([i1_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)[0]
            i2_pts_undistorted = cv2.undistortPoints(np.array([i2_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)[0]
        else:
            print "WARNING: No calibration info. Cannot Continue"
            return       

        
        """============================================================
        Extract F and filter outliers
        ============================================================"""
        E, i1_pts_corr, i2_pts_corr= self.extract_fundamental(i1_pts_undistorted, i2_pts_undistorted)
        if (i1_pts_corr == None or len(i1_pts_corr) < 1):
            print "No inliers consistent with F"
            return        
            
            
        """============================================================
        # Extract P1 and P2 via E
        ============================================================"""
        success, P1, P2, i1_pts_final, i2_pts_final = self.extract_projections(E, i1_pts_corr, i2_pts_corr)
        if not success: # Bottom out on fail
            return
        
        R = P2[:,:3]
        self.image_based_R2 = R
        t = P2[:,3:4]
        self.image_based_t = t
        
        self.prev_i1_pts_final = i1_pts_final
        self.prev_i2_pts_final = i2_pts_final
        
        self.tf_triangulate_points(i1_pts_final, i2_pts_final)
        
        
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
        for p1, p2 in zip(self.i1_pts_draw, self.i2_pts_draw):
            county += 1
            cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
        
        # Write debug text
        for i, l in enumerate(self.debug_text):
            cv2.putText(img2, str(l), (25,img2.shape[0]-25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        for i, l in enumerate(self.upper_debug_text):
            cv2.putText(img2, str(l), (25,25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        
        # Reproject triangulated points
        for p in self.reprojected:
            cv2.circle(img2, (int(p[0]),int(p[1])+imh), 3, (220, 20, 60), 1)
            #cv2.circle(img2, (int(p[0]),int(p[1])), 3, (255, 63, 63), 1)
        for p in self.reprojected2:
            cv2.circle(img2, (int(p[0]),int(p[1])), 3, (255, 182, 193), 1)
            #cv2.circle(img2, (int(p[0]),int(p[1])+imh), 3, (255, 63, 63), 1)
        
        # Draw
        cv2.imshow("track", img2)
        cv2.imwrite("temp.jpg", img2)
        print "=============\r\nDrawn\r\n============="
        
        # Render Windows
        cv2.waitKey(10)

    def homography_to_pose(self, H):
        """input homography[9] - 3x3 Matrix
        // please note that homography should be computed
        // using centered object/reference points coordinates
        // for example coords from [0, 0], [320, 0], [320, 240], [0, 240]
        // should be converted to [-160, -120], [160, -120], [160, 120], [-160, 120]"""
        print "H : \r\n", H
        print "K : \r\n", self.cameraMatrix
        print "K-1 : \r\n", self.inverseCameraMatrix
        
        '''
        invH = np.array([[self.inverseCameraMatrix[0,0]*H[0,0]+self.inverseCameraMatrix[1,0]*H[0,1]+self.inverseCameraMatrix[2,0]*H[0,2]],
                        [self.inverseCameraMatrix[0,1]*H[0,0]+self.inverseCameraMatrix[1,1]*H[0,1]+self.inverseCameraMatrix[2,1]*H[0,2]],
                        [self.inverseCameraMatrix[0,2]*H[0,0]+self.inverseCameraMatrix[1,2]*H[0,1]+self.inverseCameraMatrix[2,2]*H[0,2]]])
        print "invH man : ", invH
        
 
                
        
        inverseCameraMatrix=self.inverseCameraMatrix.copy()
        inverseCameraMatrix[2,0] = self.inverseCameraMatrix[0,2]
        inverseCameraMatrix[2,1] = self.inverseCameraMatrix[1,2]
        inverseCameraMatrix[0,2] = 0
        inverseCameraMatrix[1,2] = 0        
        
        print "K-1 re : \r\n", inverseCameraMatrix
        '''
        
        print "H vect : \r\n", H[:, :1]
        invH = self.inverseCameraMatrix.dot(H[:, :1])
        print "invH vec : \r\n", invH 
        invH2 = self.inverseCameraMatrix.dot(H)
        print "invh2 : \r\n", invH2
        
        # Get 1/mag
        lam = 1/np.sqrt(invH.transpose().dot(invH))
        print "lam : ", lam
        
        # Scale 
        inverseCameraMatrix = self.inverseCameraMatrix.copy()*lam
        print "Scaled : \r\n", inverseCameraMatrix

        # Extract R. Force 3rd column to be orthonormal
        R = inverseCameraMatrix.dot(H)
        print "R before : \r\n", R
        R[:, 2:3] = np.cross(R[:,:1].T,R[:,1:2].T).T
        print "R : \r\n", R
        
        # Calc t (Scaled Kinv dot 3rd col of H)
        t = inverseCameraMatrix.dot(H[:,2:3])
        print "t : ", t
        
        # Transform T into next orthogonal matrix (Frobenius sense)
        # Remember again that V is transpose compared to convention (R = USV)
        U,S,V = np.linalg.svd(R)
        
        R = U.dot(V)
        print "R ortho : \r\n", R
    
    def tf_triangulate_points(self, pts1, pts2):
        """ Triangulates 3D points from set of matches co-ords using relative
        camera position determined from tf"""
        """ Continuous triangulation during normal flight appears infeasible.
        Inertia means IMU data, whilst representative on the whole, is not
        accurate at a given instance so only useful for steady maintained
        motion. 
        
        Furthermore flight will not in general produce motion that is
        ideal for triangulation. Carrying out the SVD solution to triangulation
        will give different results depending on the premultiplication, which
        shows we are not getting enough information.
        """
        
        # ---------------------------------------------------------------------
        # Working in image co-ordinates throughout
        # ---------------------------------------------------------------------
        
        # These are arrays of the triangulated points reprojected back to the
        # image plane
        self.reprojected = []
        self.reprojected2 = []
        
        
        # Bottom out on first frame
        if self.relative_quat == None:
            return
    
        
        # Get t from tf data or predefined data 
        # For some reason t must be inverted compared to expected, it does not
        # work to simply swap projection matrices PP1 and PP2
        t = -self.image_coord_trans  
        self.debug_text.append("trans: "+str(t))
        
        
        # Get rotation matrix
        R = tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(self.relative_quat))[:3, :3]
        
        self.debug_text.append("rot: "+np_to_str(tf.transformations.euler_from_matrix(R)))  
        
    
        # Bottom out if motion if insufficient
        if abs(t[1]) < 0.1 and abs(t[2]) < 0.1:
            print "Motion bad for triangulation"
            return
        
        
        # Compose projection matrix
        P_cam1_to_cam2 = np.hstack((R, t))
        
        
        """
        Triangulate using pixel co-ord and K[R t]
        """
        # Factor in camera calibration
        PP1 = np.hstack((self.cameraMatrix, np.array([[0.],[0.],[0,]])))
        PP2 = self.cameraMatrix.dot(P_cam1_to_cam2)
        #points3D_image1 = self.triangulate_points(pts1.transpose(), pts2.transpose(), PP1, PP2)[:3]
        points4D = cv2.triangulatePoints(PP1, PP2, pts1.transpose(), pts2.transpose())
        # Normalise homogeneous co-ords
        points3D_image2 = (points4D/points4D[3])[:3]
        
        
        """
        Triangulating using Kinv premultiplied pixel co-ord and [R t]
        """
        # Pre-multiply co-ords
        pts1_norm = self.inverseCameraMatrix.dot(self.make_homo(pts1).transpose()).transpose()[:,:2]
        pts2_norm = self.inverseCameraMatrix.dot(self.make_homo(pts2).transpose()).transpose()[:,:2]       
        PP1 = np.hstack((np.array([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]), np.array([[0.],[0.],[0,]])))
        PP2 = P_cam1_to_cam2
        #points3D_image3 = self.triangulate_points(pts1_norm.transpose(), pts2_norm.transpose(), PP1, PP2)[:3]
        points4D = cv2.triangulatePoints(PP1, PP2, pts1_norm.transpose(), pts2_norm.transpose())
        points3D_image4 = (points4D/points4D[3])[:3]
        
        
        """
        Filter out points that have a significant discrepancy between
        triangulation methods. The motion has provided insufficient to get a
        useful triangulation
        
        With set data and externally recorded motion a translation of 0.2m
        produced no losses to this filtering. Resultant triangulation is good.
        In flight however most points are removed by this filter suggesting
        a poor basis for triangulation.
        """
        numeric_accuracy = abs(points3D_image4 - points3D_image2)>1. 
        # True means inaccurate, ie. more than 1m discrepancy depending on calc method
        numeric_accuracy = numeric_accuracy[0] | numeric_accuracy[1] | numeric_accuracy[2]
        numeric_accuracy = np.array([numeric_accuracy]).transpose()
        numeric_accuracy = np.hstack((numeric_accuracy, numeric_accuracy, numeric_accuracy)).transpose()        
        points3D_image = np.reshape(points3D_image2[numeric_accuracy==False], (3, -1))
        
        points3D_image = points3D_image2
        
        # Output number of triangulated points
        triangulated = len(points3D_image[0])+0.
        self.debug_text.append(triangulated)
        
        # Filter points that are behind the camera
        infront = points3D_image[2] > 0        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image= np.reshape(points3D_image[infront==True], (3, -1))
        
        # Triangulated points reprojected back to the image plane
        self.reprojected2 = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, np.diag((1,1,1)), np.array([[0],[0],[0]]))        
        self.reprojected = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, R, t)
        
        
        '''
        # Filter points
        infront = points3D_image1[2] > 0        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image1= np.reshape(points3D_image1[infront==True], (3, -1))
        
        self.reprojected2 = self.world_to_pixel_distorted(self.make_homo(points3D_image1.T).T, np.diag((1,1,1)), np.array([[0],[0],[0]]))        
        self.reprojected = self.world_to_pixel_distorted(self.make_homo(points3D_image1.T).T, R, t)
        
        # Filter points
        infront = points3D_image2[2] > 0        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image2= np.reshape(points3D_image2[infront==True], (3, -1))
        
        self.reprojected4 = self.world_to_pixel_distorted(self.make_homo(points3D_image2.T).T, np.diag((1,1,1)), np.array([[0],[0],[0]]))        
        self.reprojected3 = self.world_to_pixel_distorted(self.make_homo(points3D_image2.T).T, R, t)
        
        # Filter points
        infront = points3D_image3[2] > 0        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image3= np.reshape(points3D_image3[infront==True], (3, -1))
        
        self.reprojected6 = self.world_to_pixel_distorted(self.make_homo(points3D_image3.T).T, np.diag((1,1,1)), np.array([[0],[0],[0]]))        
        self.reprojected5 = self.world_to_pixel_distorted(self.make_homo(points3D_image3.T).T, R, t)
        
        # Filter points
        infront = points3D_image4[2] > 0        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image4= np.reshape(points3D_image4[infront==True], (3, -1))
        
        self.reprojected8 = self.world_to_pixel_distorted(self.make_homo(points3D_image4.T).T, np.diag((1,1,1)), np.array([[0],[0],[0]]))        
        self.reprojected7 = self.world_to_pixel_distorted(self.make_homo(points3D_image4.T).T, R, t)
        
        points3D_image = points3D_image1
        '''
        
        # Output number of forward triangulated points
        forward_triangulated = len(points3D_image[0])+0.
        self.debug_text.append(forward_triangulated)
        
        
        # Publish Cloud
        # Note: img1 camera is taken to be the origin, so time_prev NOT
        # time_now is used.        
        #if (forward_triangulated/triangulated) > 0.5:
        print "Publishing Point cloud"
        points3D= zip(*np.vstack((points3D_image[0], points3D_image[1], points3D_image[2])))
        self.publish_cloud(points3D, self.time_prev)
        
        
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
    rospy.Subscriber('/xboxcontroller/button_back', Empty, s.manual_scan)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
