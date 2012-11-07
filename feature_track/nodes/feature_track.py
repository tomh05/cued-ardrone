#!/usr/bin/env python

#=====================Notes====================================================
# 
# It is unclear whether the translation matrix needs rescaling by camera proj
#
# It may be worth reading camera cal .yamls from disk if missing
#
# The various checks and filters carried out here should allow us to provide
# a quantified measure of confidence in the result
#
# Only need to store the keypoints and descriptors (kp# and desc#)
#==============================================================================

import roslib; roslib.load_manifest('feature_track')
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
from std_msgs.msg import Empty
from nav_msgs.msg import Path
import math
import time
import threading
import os
    
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
    
class FeatureTracker:
    def __init__(self):
        self.roll = 0.
        self.quaternion = tf.transformations.quaternion_from_euler(0.,0.,0., axes='sxyz')
        self.grey_previous = None
        self.frameskip = 0
        self.calibrated = False
        self.fd = cv2.FeatureDetector_create('SIFT')
        self.de = cv2.DescriptorExtractor_create('SIFT')
        self.dm = cv2.DescriptorMatcher_create('BruteForce')
        self.desc1 = None
        self.kp1 = None
        cv2.namedWindow("track")
        self.cloud_pub = rospy.Publisher('pointCloud', PointCloud)
        self.preload_template('/home/alex/cued-ardrone/feature_track/templates/boxTemplate.png')
        self.tf = tf.TransformListener()
        self.prev_position = None
        self.mag_dist = None
        self.speech_limiter = 0
        self.corners = None
                
    def preload_template(self, path):
        """Template features and descriptors need only be extracted once, so
        they are pre-calced here"""
        template = cv2.imread(path) # This should really use the ROS_PACKAGE_PATH
        self.grey_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template_pts = self.fd.detect(self.grey_template)
        self.template_kp, self.template_desc = self.de.compute(self.grey_template, template_pts)
        
    def compute_fundamental(self, x1,x2): # Not used
        n = x1.shape[1]
        # build matrix for equations
        A = np.zeros((n,9))
        for i in range(n):
            A[i] = [x1[0,i]*x2[0,i], x1[0,i]*x2[1,i], x1[0,i]*1.,
            x1[1,i]*x2[0,i], x1[1,i]*x2[1,i], x1[1,i]*1.,
            1.*x2[0,i], 1.*x2[1,i], 1. ]
        # compute linear least square solution
        U,S,V = np.linalg.svd(A)
        F = V[-1].reshape(3,3)
        # constrain F
        # make rank 2 by zeroing out last singular value
        U,S,V = np.linalg.svd(F)
        S[2] = 0
        F = np.dot(U,np.dot(np.diag(S),V))
        return F
        
    def compute_F_error(self, F, x1_32, x2_32):
        errs = []
        for i, p in enumerate(zip(x1_32.T, x2_32.T)):
            errs.append(np.r_[p[1], 1].T.dot(F).dot(np.r_[p[0], 1]))
        return np.mean(errs)
        
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
        return X / X[3]
        
    def triangulate_points(self, x1,x2,P1,P2):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""
        n = x1.shape[1]
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        # Triangulate for each pair
        X = [ self.triangulate_point(x1[:,i],x2[:,i],P1,P2) for i in range(n)] # Looping here is probably unavoidable
        return np.array(X).T

    def quaternion_multiply(self, quat1, quat2): # Not used
        v1 = np.array([quat1[0],quat1[1],quat1[2]])
        v2 = np.array([quat2[0],quat2[1],quat2[2]])
        cp = np.cross(v1, v2)
        x = quat1[0]+quat2[0]+cp[0]
        y = quat1[1]+quat2[1]+cp[1]
        z = quat1[2]+quat2[2]+cp[2]
        w = quat1[3]*quat2[3]-np.dot(v1, v2)
        return np.array([x,y,z,w])
        
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
        print "F time : ", time.time()-temp_time
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
        
    def extract_homography(self, i1_pts_undistorted, i2_pts_undistorted):
        """
        Extract homography and then remove outliers
        """
        
        H, mask = cv2.findHomography(i1_pts_undistorted, i2_pts_undistorted, cv2.RANSAC)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_masked = np.reshape(i1_pts_undistorted[0][mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_undistorted[0][mask_prepped==1], (-1, 2))
        i1_pts_undistorted = np.array([i1_pts_masked])
        i2_pts_undistorted = np.array([i2_pts_masked])              
        
        return H, i1_pts_undistorted[0], i2_pts_undistorted[0]
        
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
        print "E", E
        
        W = np.array([[0, -1, 0],[1, 0, 0], [0, 0, 1]])
        Z = np.array([[0, 1, 0],[-1, 0, 0], [0, 0, 0]])
                    
        # SVD of E
        U,SIGMA,V = np.linalg.svd(E)
        if np.linalg.det(U.dot(V))<0:
            V = -V
        # Contruct Diagonal
        SIGMA = np.diag(SIGMA)
        # Force third eig to zero
        SIGMA[2,2] = 0
        if SIGMA[0,0] < 0.7*SIGMA[1,1] or SIGMA[1,1] < 0.7*SIGMA[0,0]:
            print "WARNING: Disparate singular values"
        
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
            print "Support for P2 ", i, " : ", PI
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
        
        print "P2"
        print projections[ind]
        
        # Filter points
        infront = np.array([infront]).transpose()
        infront = np.append(infront, infront, 1)
        i1_pts_corr = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        self.i1_pts_draw = np.reshape(self.i1_pts_draw[infront==True], (-1, 2))
        self.i2_pts_draw = np.reshape(self.i2_pts_draw[infront==True], (-1, 2))
        
        return True, P1, projections[ind], i1_pts_corr, i2_pts_corr
    
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
        drone_angles[0] = -angles[2]
        drone_angles[1] = angles[0]
        drone_angles[2] = angles[1]
        return drone_angles
        
    def coord_drone_to_image_axis(self, angles):
        drone_angles = angles.copy()
        drone_angles[0] = angles[1]
        drone_angles[1] = angles[2]
        drone_angles[2] = -angles[0]
        return drone_angles
    
    def update_quaternion(self, R):
        """Updates the cumulative local axis stored in self.quaternion"""
        
        
        
        # Make homogenous rotation matrix from R
        R4 = np.diag([0., 0., 0., 1.])
        R4[:3, :3] = R
        
        
        
        '''
        br = tf.TransformBroadcaster() #create broadcaster
        br.sendTransform((0, 0, 0),
                         self.quaternion,
                         rospy.Time.now(), # NB: 'now' is not the same as time data was sent from drone
                         "image_rotation",
                         "world")
        '''
        
        # Output cumulative angles
        #angles = tf.transformations.euler_from_matrix(R4, axes='sxyz')
        #print angles
        success, angles = self.rotation_to_euler(R)
        angles = self.coord_image_to_drone_axis(angles)
        self.image_angle_overlay = "Image " + str(angles[0]*180/np.pi) + ", " + str(angles[1]*180/np.pi) + ", " + str(angles[2]*180/np.pi)
        print self.image_angle_overlay
        
        
        
        # [0]=roll, [1]=pitch, [2]=yaw
        angles = tf.transformations.euler_from_quaternion(self.relative_quat, axes='sxyz')
        self.angle_overlay = "Dead " + str(angles[0]*180/np.pi) + ", " + str(angles[1]*180/np.pi) + ", " + str(angles[2]*180/np.pi)
        print self.angle_overlay
        
        # Update Quaternion
        if abs(angles[0]) < np.pi/2 and abs(angles[1]) < np.pi/2 and abs(angles[2]) < np.pi/2:
            quat = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
            #print tf.transformations.euler_from_quaternion(quat)
            self.quaternion = tf.transformations.quaternion_multiply(quat,self.quaternion)
            angles = tf.transformations.euler_from_quaternion(self.quaternion)
            print "Cumulative quat : ", angles
            self.image_cumu_overlay = "Cumu  " + str(angles[0]*180/np.pi) + ", " + str(angles[1]*180/np.pi) + ", " + str(angles[2]*180/np.pi)
        
        angles = tf.transformations.euler_from_quaternion(self.world_to_drone_quaternion, axes='sxyz')      
        
        
        return angles
    
    def publish_cloud(self, points):
        cloud = PointCloud()
        cloud.header.stamp = self.tf_time_stamp # Should copy img header
        cloud.header.frame_id = "/ardrone_base_link" # Should be front camera really
        
        for i, p in enumerate(points): # Ideally done without a loop
            cloud.points.append(gm.Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        
        self.cloud_pub.publish(cloud)

        
    def templateTrack(self):
        self.t_debug_text = []
        grey_now = self.grey_now
        
        """====================================================================
        Match points with template (reusing previously calculated data)
        ===================================================================="""
        t_i1_pts, t_i2_pts = self.match_points(self.template_kp, self.kp2, self.template_desc, self.desc2)
        if t_i1_pts == None or len(t_i1_pts) < 32:
            print "===================="
            print "Too few template matches"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts, t_i2_pts)
            return
            
        self.t_debug_text.append("No of matches " + str(len(t_i1_pts)))    
            
        """================================================================
        Undistort points using known camera calibration
        ================================================================"""
        if self.calibrated:
            # Undistort points using calibration data
            t_i1_pts_undistorted = cv2.undistortPoints(np.array([t_i1_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)
            t_i2_pts_undistorted = cv2.undistortPoints(np.array([t_i2_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)
        else:
            print "WARNING: No calibration info. Cannot Continue"
            return  
            
        t_i1_pts_undistorted = np.array([t_i1_pts,])
        t_i2_pts_undistorted = np.array([t_i2_pts,])
        
        '''
        """================================================================
        Centre-reference pixel coord (necessary for easy R|t recovery)
        ================================================================"""
        t_i1_pts_undistorted[0][:,0] -= self.grey_template.shape[1]/2
        t_i1_pts_undistorted[0][:,1] -= self.grey_template.shape[0]/2
        t_i2_pts_undistorted[0][:,0] -= self.grey_previous.shape[1]/2
        t_i2_pts_undistorted[0][:,1] -= self.grey_previous.shape[0]/2
        '''
        
        '''
        """====================================================================
        Scale template to real world known size
        ===================================================================="""
        real_size_x = 0.57#0.63
        real_size_y = 0.57#0.44
        
        t_i1_pts_undistorted[0][:,0] /= self.grey_template.shape[1]
        t_i1_pts_undistorted[0][:,1] /= self.grey_template.shape[0]
        t_i1_pts_undistorted[0][:,0] *= real_size_x
        t_i1_pts_undistorted[0][:,1] *= real_size_y
        '''
        
        '''            
        """================================================================
        Compute Planar Homography
        ================================================================"""
        
        H, t_i1_pts_corr, t_i2_pts_corr = self.extract_homography(t_i1_pts_undistorted, t_i2_pts_undistorted)
        print "homo : ", H
        if t_i2_pts_corr == None or len(t_i2_pts_corr) < 4:
            print "Failed to extract homography"
            return        
        
        
        self.homography_to_pose(H)
        
        # Restore co-ords
        t_i1_pts_corr[:,0] += self.grey_template.shape[1]/2
        t_i1_pts_corr[:,1] += self.grey_template.shape[0]/2
        t_i2_pts_corr[:,0] += self.grey_previous.shape[1]/2
        t_i2_pts_corr[:,1] += self.grey_previous.shape[0]/2
        '''
        
        '''
        """================================================================
        Plot extracted perspective projection via hmography
        ================================================================"""
        
        # The corners of the template in centre zeroed pixels
        corners = np.array([[[-self.grey_template.shape[1]/2,-self.grey_template.shape[0]/2],
                           [self.grey_template.shape[1]/2, -self.grey_template.shape[0]/2],
                           [self.grey_template.shape[1]/2, self.grey_template.shape[0]/2],
                           [-self.grey_template.shape[1]/2, self.grey_template.shape[0]/2]]], dtype=np.float32)
                           
        # Transform to actual view
        c = cv2.perspectiveTransform(corners, H)[0]
        
        # Revert to corner zeroed pixels
        c[:,0]+=self.grey_previous.shape[1]/2
        c[:,1]+=self.grey_previous.shape[0]/2      
                           
        # Draw perspective projection
        img2 = stackImagesVertically(self.grey_template, grey_now)
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        imh = self.grey_template.shape[0]
        cv2.line(img2,(int(c[0,0]), int(c[0,1])+imh), (int(c[1,0]), int(c[1,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])+imh), (int(c[2,0]), int(c[2,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])+imh), (int(c[3,0]), int(c[3,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])+imh), (int(c[0,0]), int(c[0,1])+imh), (255, 255 , 255), 2)
        '''
        

        """================================================================
        Implant world size in pixel co-ords
        ================================================================"""        
        temp_time = time.time()
        
        real_size_x = 0.57#0.63
        real_size_y = 0.57#0.44
        
        t_x = real_size_x*((t_i1_pts_undistorted.T[0]/self.grey_template.shape[1])-0.5)
        t_y = real_size_y*((t_i1_pts_undistorted.T[1]/self.grey_template.shape[0])-0.5)
        t_z = np.zeros(t_x.shape)
        t_i1_pts_scaled = np.array(np.hstack((t_x, t_y, t_z)), dtype=np.float32)
        
        
        
        """================================================================
        # Calculate pose and translation to matched template
        ================================================================"""
        
        R, t, inliers = cv2.solvePnPRansac(t_i1_pts_scaled, np.array(t_i2_pts, dtype=np.float32), self.cameraMatrix, self.distCoeffs)
        
        if inliers== None:
            print "===================="
            print "Template not found"
            print "===================="
            self.template_visualise(grey_now)
            return
        t_i1_pts_corr = t_i1_pts_undistorted[0][inliers[:,0]]
        t_i2_pts_corr = t_i2_pts_undistorted[0][inliers[:,0]]
        
        self.t_debug_text.append("No of fits " + str(len(t_i1_pts_corr)))
        if len(t_i1_pts_corr) < 20:
            print "===================="
            print "Too few fits"
            print "===================="
            self.template_visualise(grey_now)
            return
        
        R, J = cv2.Rodrigues(R)
        
        success, angles = self.rotation_to_euler(R)
        angles*=180/np.pi
        
        
        print "Template"
        print "R : ", R
        print "t : ", t
        print "angles : ", angles
        
        
        """================================================================
        # Checks on validity
        ================================================================"""
        # Check for reasonable distance
        if t[2] < 0.35:
            print "===================="
            print "False match - too close"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
        if t[2] > 4:
            print "===================="
            print "False match - too far"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
            
        
        # Check forward facing (potential faster solution exists)
        # Get plane of projected corners
        # Check if plane normal dot depth is negative
        length = 0.57
        width = length
        depth = 0.14
    
        # The corners of the template in anticlockwise convention        
        corners = np.array([[-length/2,-width/2,0,1],
                       [+length/2, -width/2,0,1],
                       [+length/2, +width/2,0,1],
                       [-length/2, +width/2,0,1]]).T
        # Rotate-translate                
        P =  np.diag([1.,1.,1.,1.])
        Pinner = np.hstack((R, t))
        P[:3, :4] =Pinner
        corners_rot= P.dot(corners)
        # Normalise
        corners_rot = corners_rot.T[:, :3]
        # Get plane normal
        AB = corners_rot[1]-corners_rot[0]
        AC = corners_rot[2]-corners_rot[0]
        print AC
        plane_normal = np.cross(AB,AC)
        print "plane_normal : ", plane_normal
        # Backward test
        if plane_normal[2] < 0:
            # Template backward facing
            print "===================="
            print "False match - backward facing"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
        # Acceptable angle test
        plane_normal_mag = np.sqrt(plane_normal.transpose().dot(plane_normal))
        theta = np.arccos(plane_normal[2]/plane_normal_mag)
        if theta > 1.308996939: # > 75deg:
            print "===================="
            print "False match - too much angle"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
        
        
        """================================================================
        # Update distance to template
        ================================================================"""
        
        self.mag_dist = np.sqrt(t.T.dot(t)[0,0])
        self.t_debug_text.append("Dist " + str(self.mag_dist))
        
        
        #imh = self.grey_template.shape[0]
        #img2 = stackImagesVertically(self.grey_template, grey_now)
        #cv2.putText(img2, mag_dist_text, (25,imh+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
    
        
        
        '''
        
        """====================================================================
        Restore co-ordinates to pixel
        ===================================================================="""
        
        p_x = ((t_i1_corr_scaled.T[0]/real_size_x)+0.5)*self.grey_template.shape[1]
        p_y = ((t_i1_corr_scaled.T[1]/real_size_x)+0.5)*self.grey_template.shape[0]
        t_i1_pts_corr = np.array(np.hstack((p_x, p_y)), dtype=np.float32)
        '''
        
        """====================================================================
        Draw Matches
        ===================================================================="""
        self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr, True, R, t)
    
    
    def template_overlay(self, R, t, img2):
        """====================================================================
        Plot perspective projection without homography
        ===================================================================="""
        length = 0.57
        width = length
        depth = 0.14
        
        square_side = 0.1
        
        # The corners of the template 
        if self.corners == None:       
            self.corners = np.array(  [[-length/2,-width/2,0,1],
                                       [-length/2, +width/2,0,1],
                                       [+length/2, +width/2,0,1],
                                       [+length/2, -width/2,0,1],
                                       [-length/2,-width/2,+depth,1],
                                       [-length/2, +width/2,+depth,1],
                                       [+length/2, +width/2,+depth,1],
                                       [+length/2, -width/2,+depth,1],
                                       [-square_side,-square_side,0,1],
                                       [-square_side, +square_side,0,1],
                                       [+square_side, +square_side,0,1],
                                       [+square_side, -square_side,0,1],
                                       [-square_side, -square_side,-2*square_side,1],
                                       [-square_side, +square_side,-2*square_side,1],
                                       [+square_side, +square_side,-2*square_side,1],
                                       [+square_side, -square_side,-2*square_side,1]]).T
        
        # Set up projection from extracted R and t                   
        P =  np.diag([1.,1.,1.,1.])
        Pinner = np.hstack((R, t))
        P[:3, :4] =Pinner
        
        # Reverse project corners to pixel space (effectively K.[R|t].X)
        # Instead of having inputting the real world corners with camera origin
        # We consider the template to be centred on origin xy plane at z = 0
        sub = P.dot(self.corners)[:3]
        c = self.cameraMatrix.dot(sub).T
        # Normalise
        for xy in c:
            xy/=xy[2]
        
        imh = self.grey_template.shape[0]
            
        # Draw perspective projection
        
        # Actual template square
        cv2.line(img2,(int(c[0,0]), int(c[0,1])+imh), (int(c[1,0]), int(c[1,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])+imh), (int(c[2,0]), int(c[2,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])+imh), (int(c[3,0]), int(c[3,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])+imh), (int(c[0,0]), int(c[0,1])+imh), (255, 255 , 0), 1)
        
        # Projected back face of box
        cv2.line(img2,(int(c[4,0]), int(c[4,1])+imh), (int(c[5,0]), int(c[5,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[5,0]), int(c[5,1])+imh), (int(c[6,0]), int(c[6,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[6,0]), int(c[6,1])+imh), (int(c[7,0]), int(c[7,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[7,0]), int(c[7,1])+imh), (int(c[4,0]), int(c[4,1])+imh), (255, 255 , 0), 1)
        
        # Links between front and back face
        cv2.line(img2,(int(c[0,0]), int(c[0,1])+imh), (int(c[4,0]), int(c[4,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])+imh), (int(c[5,0]), int(c[5,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])+imh), (int(c[6,0]), int(c[6,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])+imh), (int(c[7,0]), int(c[7,1])+imh), (255, 255 , 0), 1)
        
        '''
        cv2.line(img2,(int(c[8,0]), int(c[8,1])+imh), (int(c[9,0]), int(c[9,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[9,0]), int(c[9,1])+imh), (int(c[10,0]), int(c[10,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[10,0]), int(c[10,1])+imh), (int(c[11,0]), int(c[11,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[11,0]), int(c[11,1])+imh), (int(c[8,0]), int(c[8,1])+imh), (255, 255 , 0), 1)
        
        cv2.line(img2,(int(c[12,0]), int(c[12,1])+imh), (int(c[13,0]), int(c[13,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[13,0]), int(c[13,1])+imh), (int(c[14,0]), int(c[14,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[14,0]), int(c[14,1])+imh), (int(c[15,0]), int(c[15,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[15,0]), int(c[15,1])+imh), (int(c[12,0]), int(c[12,1])+imh), (255, 255 , 0), 1)
        
        cv2.line(img2,(int(c[8,0]), int(c[8,1])+imh), (int(c[12,0]), int(c[12,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[9,0]), int(c[9,1])+imh), (int(c[13,0]), int(c[13,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[10,0]), int(c[10,1])+imh), (int(c[14,0]), int(c[14,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[11,0]), int(c[11,1])+imh), (int(c[15,0]), int(c[15,1])+imh), (255, 255 , 0), 1)
        '''


    def template_visualise(self, img, pts1=None, pts2=None,isTemplate=False, R=None, t=None):
        """Carries out the visualisation routines for template tracking.
        Draws stacked images with tracked points and template overlay is found
        
        isTemplate is True on match and requires accompanying R and t
        """
        # Clone image for drawing
        img2 = stackImagesVertically(self.grey_template, img)
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        imh = self.grey_template.shape[0]
        
        # Draw template overlay
        if isTemplate:
            self.template_overlay(R, t, img2)
        
        # Draw matches
        if pts1 != None and pts2 != None:
            for p1, p2 in zip(pts1, pts2):
                cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
        
        # Write debug text
        for i, l in enumerate(self.t_debug_text):
            cv2.putText(img2, str(l), (img2.shape[1]/2,img2.shape[0]-25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        
        cv2.imshow("template", img2)    
        
        
        
    def featureTrack(self, img):
        """Takes a cv2 numpy array image and compared to a previously
        buffered image. Features are extracted from each frame, 
        undistorted and matched.
        Multiple view geometry is then used to calculate the rotation and
        translation of the camera between frames and the position of observed
        points"""
        
        times = []
        time_offset = time.time()
        
        DEF_SET_DATA = False # Switches in fixed data
        DEF_TEMPLATE_MATCH = True  # Switches template match - should be ROS param
        DEF_THREADING = True # Enables threading of template matching
        
        # Initialise previous image buffer
        if self.grey_previous == None:
            self.grey_previous = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
        
        # Skip frames. Need to add ROS parameter to allow setting        
        self.frameskip += 1
        if self.frameskip < 11:
            return            
        self.frameskip = 0
        
            
        # Convert working images to monochrome
        grey_previous = self.grey_previous
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Initialise features and descriptor
        if self.kp1 == None:
            pts1 = self.fd.detect(grey_now)
            self.kp1, self.desc1 = self.de.compute(grey_now, pts1)
            return

        # Swap in artificial data is necessary
        if DEF_SET_DATA:
            img1 = cv2.imread("/home/alex/testData/0.jpg")
            img2 = cv2.imread("/home/alex/testData/1.jpg")
            grey_now = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            grey_previous = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            pts1 = self.fd.detect(grey_previous)
            self.kp1, self.desc1 = self.de.compute(grey_previous, pts1)
            
        self.grey_now = grey_now

        times.append(time.time()-time_offset)
        """====================================================================
        Rotation and translation from navdata
        ===================================================================="""
        self.update_tf()
        #self.projection_from_navdata()
        
        times.append(time.time()-time_offset)
        

        """====================================================================
        Find matched points in both images
        ===================================================================="""
        success, i1_pts, i2_pts = self.find_and_match_points(grey_now)
        self.i1_pts_draw = i1_pts
        self.i2_pts_draw = i2_pts
        if not success:
            return
        #print "No of matched points : ", len(i1_pts)
        times.append(time.time()-time_offset)
        
        if DEF_TEMPLATE_MATCH:    
            # Carry out template match
            if DEF_THREADING:
                template_thread = threading.Thread(target = self.templateTrack)
                template_thread.start()
            else:
                self.templateTrack()
        times.append(time.time()-time_offset)
        
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
        
        times.append(time.time()-time_offset)
        
        '''
        i1_pts_norm = self.make_homo(i1_pts_undistorted[0])
        i2_pts_norm = self.make_homo(i2_pts_undistorted[0])
        i1_pts_norm = self.inverseCameraMatrix.dot(i1_pts_norm.T)[:2].T
        i2_pts_norm = self.inverseCameraMatrix.dot(i2_pts_norm.T)[:2].T
        print "No of undistorted points : ", len(i1_pts_norm)
        #print self.cameraMatrix.dot(i1_pts_norm)[:2].T
        '''
        

        
        """============================================================
        Extract F and filter outliers
        ============================================================"""
        E, i1_pts_corr, i2_pts_corr= self.extract_fundamental(i1_pts_undistorted, i2_pts_undistorted)
        if (i1_pts_corr == None or len(i1_pts_corr) < 1):
            print "No inliers consistent with F"
            self.grey_previous = grey_now
            self.kp1, self.desc1 = self.kp2, self.desc2
            return
        print "No of corrected points : ", len(i1_pts_corr)
        
        times.append(time.time()-time_offset)
        
        
        '''
        """====================================================================
        Normalise with inverse K, this means computing F gives E
        This should avoid numerical problems in determining F
        ===================================================================="""
        i1_temp = self.inverseCameraMatrix.dot(self.make_homo(i1_pts_undistorted).transpose()).transpose()[:, :2]        
        i2_temp = self.inverseCameraMatrix.dot(self.make_homo(i2_pts_undistorted).transpose()).transpose()[:, :2]
        E, i1_pts_corr, i2_pts_corr = self.extract_fundamental(i1_pts_undistorted, i2_pts_undistorted)
        '''

        """============================================================
        # Examine quality of F
        # Reject if error is too high and go to next frame
        ============================================================"""   
        avg_error = self.compute_F_error(E, i1_pts_corr.transpose(), i2_pts_corr.transpose())        
        print "Avg F error : ", avg_error
        if (abs(avg_error)>1): # Bottom out on high error
            print "===================="
            print "F Error too high"
            print "===================="
            self.grey_previous = grey_now
            self.kp1, self.desc1 = self.kp2, self.desc2
            return
        times.append(time.time()-time_offset)
        
        #============================================================================================================DOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOM
        self.tf_triangulate_points(i1_pts_corr, i2_pts_corr)
        times.append(time.time()-time_offset)
            
        """============================================================
        # Extract P1 and P2 via E
        ============================================================"""
        success, P1, P2, i1_pts_final, i2_pts_final = self.extract_projections(E, i1_pts_corr, i2_pts_corr)
        if not success: # Bottom out on fail
            self.grey_previous = grey_now
            self.kp1, self.desc1 = self.kp2, self.desc2
            return
        #print "No of projected points : ", len(i1_pts_final)
        
        R = P2[:,:3]
        #print "Rotation Matrix : ", R
        t = P2[:,3:4]
        t_scaled = t*self.drone_coord_trans[2]/t[2]
        times.append(time.time()-time_offset)
        
        """============================================================
        # Update cumulative orientation quaternion 
        ============================================================"""                
        angles = self.update_quaternion(R) 
        times.append(time.time()-time_offset)
        
        '''
        """====================================================================
        # Calculate 3D points (No point as scale unknown)
        ===================================================================="""
        points4D = cv2.triangulatePoints(P1, P2, i1_pts_final.transpose(), i2_pts_final.transpose())
        # normalise homogenous coords
        points4D /= points4D[3]
        
        # From camera one frame of ref
        X1 = P1.dot(points4D)
        med1 = 4*np.median(X1[2])
        # From camera two frame of ref
        X2 = P2.dot(points4D)
        med2 = 4*np.median(X2[2])
        
        # Remove unrealistically far back points
        reasonable = (X1[2]<med1) & (X2[2]<med2)
        reasonable = np.array([reasonable]).transpose()
        reasonable = np.append(reasonable, reasonable, 1)
        i1_pts_final = np.reshape(i1_pts_corr[reasonable==True], (-1, 2))
        i2_pts_final = np.reshape(i2_pts_corr[reasonable==True], (-1, 2))
        self.i1_pts_draw = np.reshape(self.i1_pts_draw[reasonable==True], (-1, 2))
        self.i2_pts_draw = np.reshape(self.i2_pts_draw[reasonable==True], (-1, 2))
        
        
        
        # Reform
        points3D1 = zip(*X1)
        points3D2 = zip(*X2)
        '''
        
        '''
        """============================================================
        # Publish point cloud
        ============================================================"""
        #self.cloud_from_navdata(i1_pts_final, i2_pts_final)
        print "3D1 format : ", points3D1
        self.publish_cloud(points3D1)
        

        #print "final : ", i1_pts_final
        '''
        '''
        """====================================================================
        # Convert back to pixel co-ords
        ===================================================================="""
        i1_pts_pixel = self.make_homo(i1_pts_final)
        i2_pts_pixel = self.make_homo(i2_pts_final)
        i1_pts_pixel = self.cameraMatrix.dot(i1_pts_pixel.T)[:2].transpose()
        i2_pts_pixel = self.cameraMatrix.dot(i2_pts_pixel.T)[:2].transpose()
        '''
        
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
        for p1, p2 in zip(self.i1_pts_draw, self.i2_pts_draw):
            county += 1
            cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
            
            
        # Draw artificial horizon for lower image
        h0 = grey_now.shape[0]
        w0 = grey_now.shape[1]
        h = int((3*h0/2)*(1-math.sin(angles[1])))
        cv2.line(img2, (w0/2, h), (int(w0/2+l*math.cos(angles[0])), int(h+l*math.sin(angles[0]))), (255, 255, 255), 1)
        cv2.line(img2, (w0/2, h), (int(w0/2-l*math.cos(angles[0])), int(h-l*math.sin(angles[0]))), (255, 255, 255), 1)
        
        
        # Overlay text (more readable than console)
        cv2.putText(img2, self.angle_overlay, (25,25), cv2.FONT_HERSHEY_PLAIN, 1, (255, 127, 255))
        cv2.putText(img2, self.image_angle_overlay, (25,50), cv2.FONT_HERSHEY_PLAIN, 1, (255, 127, 255))
        cv2.putText(img2, "tf t :"+str(self.drone_coord_trans), (25,75), cv2.FONT_HERSHEY_PLAIN, 1, (255, 127, 255))
        cv2.putText(img2, "im t :"+str(t_scaled), (25,100), cv2.FONT_HERSHEY_PLAIN, 1, (255, 127, 255))
        cv2.putText(img2, self.image_cumu_overlay, (25,25+imh), cv2.FONT_HERSHEY_PLAIN, 1, (255, 127, 255))
        
        # Draw
        cv2.imshow("track", img2)
        cv2.waitKey(3)
        print "=============\r\nDrawn\r\n============="
        times.append(time.time()-time_offset)
        
        """====================================================================
        # Update previous image buffer
        ===================================================================="""
        self.grey_previous = grey_now
        self.pts1 = self.pts2
        self.kp1, self.desc1 = self.kp2, self.desc2
        times.append(time.time()-time_offset)
        
        if DEF_THREADING and DEF_TEMPLATE_MATCH:
            """================================================================
            # Wait for threading to complete
            ================================================================"""
            idle = 0
            idle_step = 2
            if template_thread.isAlive():
                cv2.waitKey(idle_step)
                idle+=idle_step
                print "Main thread waiting for template thread : ", idle, "us"
        
        
        times.append(time.time()-time_offset)
        print times
    
    
    def update_tf(self):
        """Updates the cache of most recent pose information"""
        t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        self.tf_time_stamp = t
        
        # This is world w.r.t ardrone_base_link but yields a result consistent with ardrone_base_link w.r.t world
        # Documentation, or more likely my understanding of it is wrong (the other way round gives world origin in drone coords)
        position, self.world_to_drone_quaternion = self.tf.lookupTransform("world", "ardrone_base_link", t)
        
        gamma = tf.transformations.euler_from_quaternion(self.world_to_drone_quaternion, axes='sxyz')[2]
        #self.temp_text = position
        
        # Get change in position
        print "position", position
        print "prev_position", self.prev_position
        if self.prev_position != None:
            # Difference in position in world axis
            trans = np.array(([position[0] - self.prev_position[0]],
                                            [position[1] - self.prev_position[1]],
                                            [0.]))
            ''' This assumes flat drone
            # Convert to drone image-proc axis (z = drone forward , x = left to right horiz, y = down)
            self.drone_coord_trans = np.array([(trans[0]*math.cos(gamma)+trans[1]*math.sin(gamma)),
                                               [0.],
                                               (trans[0]*math.cos(gamma)-trans[1]*math.sin(gamma))])
            '''
            
            R = tf.transformations.quaternion_matrix(self.world_to_drone_quaternion)[:3, :3]
            # Into ardrone_base_link co-ords
            trans = R.dot(trans)
            # Re-order axis into image co-ords
            self.drone_coord_trans = np.array([-trans[1], -trans[2], trans[0]])
            
            
            
            
            '''
            trans = np.array([[position[0] - self.prev_position[0]],
                                            [position[1] - self.prev_position[1]],
                                            [position[2] - self.prev_position[2]],
                                            [1.]])
            # Rotate to drone axis
            rot = tf.transformations.quaternion_matrix(self.world_to_drone_quaternion)
            trans = rot.dot(trans)
            # Re-map to image axis convention
            self.tf_translation = trans[:3]
            trans[0] = -trans[1]
            trans[1] = -trans[2]
            trans[2] = trans[0]l
            
            self.tf_translation = trans
            '''
            
            # Get relative quaternion
            # qmid = qbefore-1.qafter
            self.relative_quat = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(self.prev_quat), self.world_to_drone_quaternion)
        self.prev_position = position
        self.prev_quat = self.world_to_drone_quaternion


       
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
        
        print "Triangulation"
        # For ease of reading until names are made consistent and clear
        t = self.drone_coord_trans
        #t = np.array([[-0.235],[0],[0]])#
        print "Translation : ", t
        
        # Get rotation matrix for crone co-ord axis
        # Rebuild R
        # Flip eulers to image axis
        angles = np.array(tf.transformations.euler_from_quaternion(self.relative_quat))        
        angles = self.coord_drone_to_image_axis(angles)
        R_cam1_to_cam2 = tf.transformations.euler_matrix(angles[0],angles[1],angles[2], axes='sxyz')[:3, :3]        
        #R_cam1_to_cam2 = np.diag([1,1,1])#
        R = R_cam1_to_cam2
        #print "Rotation Matrix : ", R_cam1_to_cam2
        P_cam1_to_cam2 = np.hstack((R_cam1_to_cam2, t))
        T = P_cam1_to_cam2
        print P_cam1_to_cam2
        print self.P
        
        '''
        empty = np.array([[0],[0],[0]])
        ab = np.reshape(self.inverseCameraMatrix.dot(self.make_homo(pts1).transpose()), (3, -1))
        a = ab[0]
        b = ab[1]
        cd = np.reshape(self.inverseCameraMatrix.dot(self.make_homo(pts2).transpose()), (3, -1))
        c = cd[0]
        d = cd[1]
        
        X = (-t[0] + c*t[2])/(R[0,0]-c*R[2,0]-(b/a)*(R[0,1]-c*R[2,1]+(1/b)*(R[0,2] - c*R[2,2])))
        Y = (a/b)*X
        Z = (1/a)*X

        
        #print Z[1]
        
        #print "x,y,z : ", X, ", ", Y, ", ", Z
        '''
        
        
        #print self.cameraMatrix.shape
        PP1 = np.hstack((self.cameraMatrix, np.array([[0.],[0.],[0,]])))
        PP2 = self.cameraMatrix.dot(P_cam1_to_cam2)
        points3D_image = self.triangulate_points(pts1.transpose(), pts2.transpose(), PP1, PP2)[:3]
        
        infront = points3D_image[2] > 0
        #print infront
        # Filter points
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()
        #print infront.shape
        #print points3D_image
        
        points3D_image= np.reshape(points3D_image[infront==True], (3, -1))
        #print points3D_image
        
        points3D= zip(*np.vstack((points3D_image[2], -points3D_image[0], -points3D_image[1])))
        #print len(points3D)
        
        
        self.publish_cloud(points3D)
        
        
        
        
    
    '''    
    def cloud_from_navdata(self, i1_pts_final, i2_pts_final):
        """Produces P1 and P2 from deadreckon data"""
        # Use camera1 as origin viewpoint
        P1 = np.append(np.identity(3), [[0], [0], [0]], 1)        
        
        R = tf.transformations.quaternion_matrix(self.relative_quat)[:3, :3]
        t = self.tf_translation
        P2 = np.append(R, t, 1)
        
        """====================================================================
        # Calculate 3D points
        ===================================================================="""
        points4D = cv2.triangulatePoints(P1, P2, i1_pts_final.transpose(), i2_pts_final.transpose())
        # normalise homogenous coords
        points4D /= points4D[3]
        
        # From camera one frame of ref
        X1 = P1.dot(points4D)
        
        # From camera two frame of ref
        X2 = P2.dot(points4D)
        
        # Reform
        points3D1 = zip(*X1)
        points3D2 = zip(*X2)        
        
        """============================================================
        # Publish point cloud
        ============================================================"""
        self.publish_cloud(points3D1)
    '''
    
    
    def speakDistance(self, d):
        print "bing-----------------------------------------------------------"
        """Audibly reads the template distance"""
        if self.mag_dist == None:
            text = "Marker not seen"
        else:
            text = "Distance to marker is "
        text+=str(self.mag_dist)
        os.system('espeak "'+text+'" --stdout | paplay')
    

    def imgproc(self, d):
        """Converts the ROS published image to a cv2 numpy array
        and passes to FeatureTracker"""
        # ROS to cv image
        bridge = CvBridge()
        cvimg = bridge.imgmsg_to_cv(d,"bgr8")
        # cv to cv2 numpy array image
        npimg = np.asarray(cvimg)
        # Pass to FeatureTracker
        self.featureTrack(npimg)
        
        
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

  
def connect(m):
    rospy.init_node('Feature_Tracker')
    rospy.Subscriber('/ardrone/front/image_raw',Image,m.imgproc)
    rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, m.setCameraInfo)
    rospy.Subscriber('/xboxcontroller/button_y',Empty,m.speakDistance)


def run():
    # Initialise tracker
    m = FeatureTracker()
    # Initialise ROS node
    connect(m)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
