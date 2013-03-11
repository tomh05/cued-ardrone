#!/usr/bin/env python

#=============================== Positioner====================================
# 
# This script locates the drone in world co-ords from previously triangulated
# positions
#
#==============================================================================
#
#==============================================================================

import roslib; roslib.load_manifest('scan')
import rospy
import numpy as np
import cv2
import tf
import time
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import Described3DPoints
from custom_msgs.msg import StampedFeaturesWithImage

class Positioner:    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Set up descriptor matcher
        self.matcher_type = 'None'
        self.kp = None
        
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        self.br = tf.TransformBroadcaster() 
        rospy.Subscriber('/accumulator/absolute_described_cloud',Described3DPoints,self.on_got_cloud)
        rospy.Subscriber('/feature_extractor/features', StampedFeaturesWithImage, self.on_got_features)
        self.pose_pub = rospy.Publisher('/positioner/pose', PoseStamped)
        
    def on_got_cloud(self, cloud):
        #print "Received new cloud"
        self.decode_and_buffer_message(cloud)
        
    def on_got_features(self, sfwi):
        kp, desc = self.get_keypoints_and_descriptors(sfwi)
        """Attempts to locate world position by matching observed image points to 
        previously triangulated points"""
        
        if self.kp == None:
            return

        
        kp_matched, pts3D_matched = self.match_points(kp, self.points, desc, self.desc)
        print "Cross matches: ", len(kp_matched)

        
        """================================================================
        # Calculate pose and translation to matched template
        ================================================================"""
        
        if len(self.kp) < 5:
            "Not enough matches to localise"
            return
        
        print pts3D_matched.shape
        print kp_matched.shape
        
        R, t, inliers = cv2.solvePnPRansac(np.array(pts3D_matched, dtype=np.float32).T, np.array(kp_matched, dtype=np.float32), self.cameraMatrix, self.distCoeffs)
        
        
        if inliers == None:
            print "===================="
            print "Template not found"
            print "===================="
            return False
        
        
        
        print "No. of inliers: ", len(inliers)   
        if len(inliers) > 16: 
            
            
            R, J = cv2.Rodrigues(R)        
            Rhomo = np.diag((1., 1., 1., 1.))
            Rhomo[:3, :3] = R
            
            t_position = -R.T.dot(t)
            print "t: ", t_position
        
            
  
            header = sfwi.header
            
            # Publish tf
            quat = tf.transformations.quaternion_inverse(tf.transformations.quaternion_from_matrix(Rhomo))
            header.stamp = sfwi.header.stamp
            header.frame_id = '/world'
            
            self.br.sendTransform((t_position[0],t_position[1],t_position[2]), 
                         # translation happens first, then rotation
                         quat,
                         header.stamp,
                         "/ardrone_from_cloud",
                         "/world")
                         
            ps = PoseStamped()
            ps.pose.position.x  = t_position[0]
            ps.pose.position.y  = t_position[1]
            ps.pose.position.z  = t_position[2]
            ps.pose.orientation.x = quat[0]
            ps.pose.orientation.y = quat[1]
            ps.pose.orientation.z = quat[2]
            ps.pose.orientation.w = quat[3]
            ps.header = sfwi.header
            self.pose_pub.publish(ps)
    
    
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
            print "Auto-switching to ", self.matcher_type
        return kp, desc
    
    def get_subset(self, position_i, quat_i_to_w, quat_w_to_i):
        camera_relative = np.add(tf.transformations.quaternion_matrix(quat_w_to_i)[:3,:3].dot(self.mean), -position_i)
        mask = camera_relative[2] > 0
        desc_masked = self.desc[mask, :]
        kp_masked = self.kp[mask, :]
        return kp_masked, desc_masked, np.cumsum(np.array(mask, dtype=np.int16))-1

    
    def decode_and_buffer_message(self, msg):
        # Need to re-numpy the float32 array descriptors were transmitted in
        self.points = np.reshape(np.array(msg.points3D), (3, -1))
        self.kp = np.reshape(np.array(msg.points2D), (2, -1)).T
        if (msg.descriptors_matcher == 'BruteForce'):
            self.desc= np.reshape(np.array(msg.descriptors, np.float32), (-1, msg.descriptors_stride))   
        elif (msg.descriptors_matcher == 'BruteForce-Hamming'):
            self.desc = np.reshape(np.array(msg.descriptors, np.uint8), (-1, msg.descriptors_stride))
        # Switch matcher if necessary
        if (self.matcher_type != msg.descriptors_matcher):
            self.matcher_type = msg.descriptors_matcher
            self.dm = cv2.DescriptorMatcher_create(self.matcher_type)
            print "Auto-switching to ", self.matcher_type
        
    def match_points(self, kp, pts3D, desc1, desc2):
        """Matches the specified point with already triangulated points"""
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
        kp_matched = kp[i1_indices,:]
        pts3D_matched = pts3D[:, i2_indices]
        
        return kp_matched, pts3D_matched
        
    def extract_fundamental_and_filter_inliers(self, i1_pts_undistorted, i2_pts_undistorted, indices1, indices2):
        """Extract fundamental matrix and then remove outliers"""        
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 15, param2 = 0.99)
        # Filter indices
        indices1_corr = indices1[mask.flatten()==1]
        indices2_corr = indices2[mask.flatten()==1]
        return indices1_corr, indices2_corr
        
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
        self.virtual_projection = np.hstack((self.cameraMatrix, np.zeros((3,1))))
        self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
        self.distCoeffs = np.array([ci.D], dtype=np.float32)
        self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
        print "                    Calibration Initialised\r\n"


def run():
    rospy.init_node('Positioner')
    
    # Print startup info
    print "\r\n"
    print "=========================== Positioner ==========================="
    print " "
    print "==================================================================="
    print "\r\n"
    
    # Initialise controller
    p = Positioner()    
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
