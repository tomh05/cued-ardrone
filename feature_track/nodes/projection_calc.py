#!/usr/bin/env python

#============================== Projection Calc ===============================
# 
# This script calculates the essential matrix and then projection matrix
# between two given frames with their matches
#
#==============================================================================
# 
# This routine was the single most intensive part of the entire image pipeline
# The main issue is that it must triangulate 4N points where there are 4 
# matches and this is more than 4 times worse than actual 3D point cloud
# triangulation as N is greater at this point in the pipeline.
#
# A such a probabilistic rejection method was added. This sequentially tests
# points for each of the 4 projections. The lower bound for the best projection
# is then used as a hypothesis to test the other projections after each cycle, 
# and projections are rejected on a 3 sigma confidence margin (99.6%).
# 
# e.g. We have M=200 samples and have tested N=25. Projections have passed:
#       P1=22, P2=18 , P3=0, P4=4
# We hypothesis that the best case is P1=22 out of the 200 (rather than the 25)
# as a lower bound. We establish the standard deviation as:
#       sigma = sqrt(P1/M(1-P1/M)/M)
# P2, P3 and P4 are hypothesis tested with:
#       std_dev_from_hypothesis = (P#/M - P1/M)/sigma
# And rejected if the magnitude is greater than 3 (it must always be negative)
# 
# This boosts performance by a factor of 3-4x and appears to yield the same
# results consistantly. The rejection method is purely speculative and whilst
# it appears to work and seems intuitive, it may well be unsound.
#
# Probabilistic rejection should be capable of ~40 fps
# Full calculation should be capable of ~10 fps
#==============================================================================

import roslib; roslib.load_manifest('feature_track')
import rospy
import numpy as np
import cv2
import std_msgs.msg
from geometry_msgs.msg import Point32
from custom_msgs.msg import StampedFeaturesMatches
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud
import time
import tf

class ProjectionCalc:
    
    def __init__(self, prob):
        # Initialise ROS node
        self.connect()
        self.use_prob = prob
        
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        rospy.Subscriber('/feature_matcher/matches',StampedFeaturesMatches,self.on_got_matches)
        self.cloud_pub2 = rospy.Publisher('/projection_calc/relative_cloud', PointCloud)
        self.match_pub = rospy.Publisher('/projection_calc/projections',StampedFeaturesMatches)
        self.br = tf.TransformBroadcaster()

    def on_got_matches(self, sfm):
        print "---------------------------------------------------------------"
        self.time_prev = time.time()
        # Get np formatted msg
        F, pts1, pts2, desc1, desc2 = self.decode_message(sfm)
        
        self.header1 = sfm.header1
        self.header2 = sfm.header2
        
        success, P1, P2, pts1_E, pts2_E, desc1_E, desc2_E = self.extract_projections(F, pts1, pts2, desc1, desc2)
        if not success or len(pts1_E) < 4:
            print "Motion failed ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) "
            return
        
            
        
        R = np.diag((1.,1.,1.,1.))
        R[:3,:3] = P1[:3,:3]
        quat = tf.transformations.quaternion_from_matrix(R)
        self.br.sendTransform((0,0,0), 
                         # translation happens first, then rotation
                         quat,
                         sfm.header2.stamp,
                         "/viewprev",
                         "/ardrone_base_frontcam")
        
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
        
        W = np.array([[0., -1., 0.],[1., 0., 0.], [0., 0., 1.]])
        Z = np.array([[0., 1., 0.],[-1., 0., 0.], [0., 0., 0.]])

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
        if self.use_prob:
            P2, infront = self.probabilistic_projection(i1_pts_corr, i2_pts_corr, P1, projections)
        else:
            P2, infront = self.normal_projection(i1_pts_corr, i2_pts_corr, P1, projections)

        
        # Filter points
        mask_prepped = np.resize(infront.T, (desc1.shape[1],desc1.shape[0])).T
        infront = np.append(infront, infront, 1)
        i1_pts_final = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_final = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        
        # Filter descriptors
        desc1_final = np.reshape(desc1[mask_prepped==True], (-1, desc1.shape[1]))
        desc2_final = np.reshape(desc2[mask_prepped==True], (-1, desc2.shape[1]))
        
        return True, P1, P2, i1_pts_final, i2_pts_final, desc1_final, desc2_final
        
    def normal_projection(self, i1_pts_corr, i2_pts_corr, P1, projections):       
        print i1_pts_corr.T.shape[1]
        #time_prev = time.time()
        ind = 0
        maxfit = 0
        secfit = 0
        PIs = np.array([0,0,0,0])
        for i, P2 in enumerate(projections):
            points4D = self.unfiltered_triangulate_points(i1_pts_corr.transpose(), i2_pts_corr.transpose(),self.cameraMatrix.dot(P1),self.cameraMatrix.dot(P2))
            print points4D.T
            # d1 = P1.dot(points4D) # d1 = points4D
            d1 = points4D[2]
            d2 = P2.dot(points4D)[2]
            PI = sum(np.logical_and(d1>0,d2>0))
            PIs[i] = PI
            #print "Support for P2 ", i, " : ", PI
            if PI > maxfit:
                secfit = maxfit
                maxfit = PI
                ind = i
                infront = (d1>0) & (d2>0)
        #print maxfit
        #print "P2"
        #print projections[ind]
        infront = np.array([infront]).transpose()
        #print "Normal3 done ( ", np.around(((time.time()-time_prev)*1000),1), "ms) "
        print PIs
        
        """====================================================================
        # Relative Point Cloud
        ===================================================================="""        
        cloud = PointCloud()
        cloud.header = self.header1
        # Reshape for easy clouding
        
        points = self.unfiltered_triangulate_points(i1_pts_corr.transpose(), i2_pts_corr.transpose(),self.cameraMatrix.dot(P1),self.cameraMatrix.dot(projections[ind]))
        
        sub = zip(*np.vstack((points[0], points[1], points[2])))
        
        # Build relative cloud
        for i, p in enumerate(sub):
            cloud.points.append(Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        self.cloud_pub2.publish(cloud) 
        
        
        return projections[ind], infront
        
    def make_homo(self, pts):
        pts = np.append(pts,np.array([np.ones(pts.shape[0])]).T, 1)
        return pts
    
    def probabilistic_projection(self, i1_pts_corr, i2_pts_corr, P1, projections):
        """====================================================================
        # Probabilistic P selection
        # Only test enough points to be relatively certain
        ===================================================================="""
        #time_prev = time.time()
        # Make homogenous
        x1 = np.append(i1_pts_corr.transpose(), np.array([np.ones(i1_pts_corr.transpose().shape[1])]), 0)
        x2 = np.append(i2_pts_corr.transpose(), np.array([np.ones(i2_pts_corr.transpose().shape[1])]), 0)
        # Initialise
        PI = np.array([0,0,0,0])
        rejected = np.array([False, False, False, False])
        rejected_count = 0
        triangulators = []
        infronts = []
        points4D = []
        KP1 = self.cameraMatrix.dot(P1)
        Kprojections = []
        for i, P2 in enumerate(projections):
            Kprojections.append(self.cameraMatrix.dot(P2))
            triangulators.append(Triangulator(KP1, Kprojections[i]))
            infronts.append(np.array(np.zeros((1,x1.shape[1])), dtype=np.bool).T)
            points4D.append(np.zeros((4,x1.shape[1]),dtype=np.float32))
        
        
        # Rejection iterate
        for i in  range(x1.shape[1]):
            for j, KP2 in enumerate(Kprojections):
                if not rejected[j]:
                    point4D = np.array([triangulators[j].triangulate(x1[:,i],x2[:,i], KP1, KP2)]).T
                    points4D[j][:,i:i+1]=point4D
                    d1 = point4D[2]
                    d2 = projections[j].dot(point4D)[2]
                    front = (d1>0) and (d2>0)
                    infronts[j][i] = front
                    PI[j] = PI[j] + int(front)


            if (i > 10 and rejected_count < 3):
                maxpi = 0
                index = -1
                for k, pi in enumerate(PI):
                    if not rejected[k] and pi >= maxpi:
                        maxpi = pi
                        index = k
                if maxpi > 0:
                    p_test = maxpi/float(x1.shape[1])
                    sigma = np.sqrt(p_test*(1.-p_test)/float(x1.shape[1]))
                    fail = False
                    for k, pi in enumerate(PI):
                        if not rejected[k] and k != index:
                            p = pi/float(x1.shape[1])
                            z = (p-p_test)/sigma
                            if abs(z) > 3:
                                rejected[k] = True
                                rejected_count = rejected_count + 1
        #print "Probabilistic done ( ", np.around(((time.time()-time_prev)*1000),1), "ms) "
        
        """====================================================================
        # Relative Point Cloud
        ===================================================================="""        
        cloud = PointCloud()
        cloud.header = self.header1
        # Reshape for easy clouding
        points = points4D[index]
        sub = zip(*np.vstack((points[0], points[1], points[2])))
        
        # Build relative cloud
        for i, p in enumerate(sub):
            cloud.points.append(Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        self.cloud_pub2.publish(cloud) 
        
        
        print PI
        return projections[index], infronts[index]
        
    def unfiltered_triangulate_points(self,x1,x2,P1,P2):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        # Triangulate for each pair
        triangulator = Triangulator(P1, P2)
        X = np.array([triangulator.triangulate(x1[:,i],x2[:,i], P1, P2) for i in range(x1.shape[1])]) # Looping here is probably unavoidable
        return X.T
    
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
        self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
        self.distCoeffs = np.array([ci.D], dtype=np.float32)
        self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
        print "                    Calibration Initialised\r\n"
            
class Triangulator:
        def __init__(self, P1, P2):
            self.M = np.zeros((4,4))
            self.off = np.array([P1[0], P1[1], P2[0], P2[1]])
            
        def triangulate(self, x1, x2, P1, P2):
            # Compose matrix representing simultaneous equations
            self.M[0] = x1[0]*P1[2]
            self.M[1] = x1[1]*P1[2]
            self.M[2] = x2[0]*P2[2]
            self.M[3] = x2[1]*P2[2]
            self.M = self.M - self.off
            # Compute SVD
            U,S,V = np.linalg.svd(self.M)        
            # numpy SVD is ordered from largest to smallest (for S)
            # so least squares solution will always lie in the last column of V
            # BUT since numpy SVD returns V transpose not V, it is the last row
            X = V[-1,:]
            return X / X[3]

def run():
    rospy.init_node('Projection_Calc')
    # Initialise controller
    
    # Get parameters
    prob = rospy.get_param('~prob', True)
    
    # Print startup info
    print "\r\n"
    print "======================= Projection Calc ==========================="
    print " Probabilistic rejection : ", prob, " - Set with _prob"
    print "==================================================================="
    print "\r\n"
    
    pc = ProjectionCalc(prob)
    
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
