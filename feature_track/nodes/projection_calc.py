#!/usr/bin/env python

import roslib; roslib.load_manifest('feature_track')
import rospy
import numpy as np
import cv2
import std_msgs.msg
from custom_msgs.msg import StampedFeaturesMatches
from sensor_msgs.msg import CameraInfo
import time

class ProjectionCalc:
    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        rospy.Subscriber('/feature_matcher/matches',StampedFeaturesMatches,self.on_got_matches)
        self.match_pub = rospy.Publisher('/projection_calc/projections',StampedFeaturesMatches)

    def on_got_matches(self, sfm):
        self.time_prev = time.time()
        # Get np formatted msg
        F, pts1, pts2, desc1, desc2 = self.decode_message(sfm)
        
        success, P1, P2, pts1_E, pts2_E, desc1_E, desc2_E = self.extract_projections(F, pts1, pts2, desc1, desc2)
        if not success or len(pts1_E) < 4:
            return
        
        self.publish_proj(P1, P2, pts1_E, pts2_E, desc1_E, desc2_E, sfm)
        
        print "Motion from images done ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) "

    def decode_message(self, sfm):
        # Need to re-numpy the array kp & descriptors
        kp1 = np.reshape(np.array(sfm.points1), (-1, 2))
        kp2 = np.reshape(np.array(sfm.points2), (-1, 2))
        if (sfm.descriptors_matcher == 'BruteForce'):
            desc1 = np.reshape(np.array(sfm.descriptors1, np.float32), (-1, sfm.descriptors_stride))   
            desc2 = np.reshape(np.array(sfm.descriptors2, np.float32), (-1, sfm.descriptors_stride))  
        elif (sfm.descriptors_matcher == 'BruteForce-Hamming'):
            desc1 = np.reshape(np.array(sfm.descriptors1, np.uint8), (-1, sfm.descriptors_stride))  
            desc2 = np.reshape(np.array(sfm.descriptors2, np.uint8), (-1, sfm.descriptors_stride))
        F = np.reshape(np.array(sfm.F, np.float32), (3, 3))
        return F, kp1, kp2, desc1, desc2
        
    def publish_proj(self, P1, P2, pts1, pts2, desc1, desc2, sfm):
        sfm.points1 = pts1.reshape(-1,).tolist()
        sfm.points2 = pts2.reshape(-1,).tolist()
        sfm.descriptors1 = desc1.reshape(-1,).tolist()
        sfm.descriptors2 = desc2.reshape(-1,).tolist()
        sfm.descriptors_stride = desc1.shape[1]
        sfm.P1 = P1.reshape(-1,).tolist()
        sfm.P2 = P2.reshape(-1,).tolist()
        self.match_pub.publish(sfm)

    def extract_projections(self, F, i1_pts_corr, i2_pts_corr, desc1, desc2):
        """
        Uses known camera calibration to extract E
        Produces 4 possible P2 via linear algebra
        Isolates best P2 by projecting data points
        Filters out conflicting points
        """
        
        # Camera Matrices to extract essential matrix and then normalise
        E = self.cameraMatrix.transpose().dot(F.dot(self.cameraMatrix))
        E /= E[2,2]
        
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
        ==============================================================="""  
        

        ind = 0
        maxfit = 0
        secfit = 0
        for i, P2 in enumerate(projections):
            points4D = self.unfiltered_triangulate_points(i1_pts_corr.transpose(), i2_pts_corr.transpose(),P1,P2)
            d1 = np.dot(self.cameraMatrix.dot(P1),points4D)[2]
            d2 = np.dot(self.cameraMatrix.dot(P2),points4D)[2]
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
        mask_prepped = np.resize(infront.T, (desc1.shape[1],desc1.shape[0])).T
        infront = np.append(infront, infront, 1)
        i1_pts_final = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_final = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        
        # Filter descriptors
        desc1_final = np.reshape(desc1[mask_prepped==True], (-1, desc1.shape[1]))
        desc2_final = np.reshape(desc2[mask_prepped==True], (-1, desc2.shape[1]))
        
        return True, P1, projections[ind], i1_pts_final, i2_pts_final, desc1_final, desc2_final
        
    def unfiltered_triangulate_points(self,x1,x2,P1,P2):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""
        n = x1.shape[1]
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        # Triangulate for each pair
        X = np.array([self.unfiltered_triangulate_point(x1[:,i],x2[:,i], P1, P2) for i in range(x1.shape[1])]) # Looping here is probably unavoidable
        return X.T
        
    def unfiltered_triangulate_point(self, x1,x2, P1, P2): 
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
        
        return np.hstack(X / X[3])
    
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
        self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
        self.distCoeffs = np.array([ci.D], dtype=np.float32)
        self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
        print "                    Calibration Initialised\r\n"

def run():
    rospy.init_node('Projection_Calc')
    # Initialise controller
    
    # Get parameters
    
    # Print startup info
    print "\r\n"
    print "======================= Projection Calc ==========================="
    print " Auto-detecting matcher type from feature type"
    print "==================================================================="
    print "\r\n"
    
    pc = ProjectionCalc()
    
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
