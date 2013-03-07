#!/usr/bin/env python

#=============================== Accumulator===================================
# 
# This script handles the accumulation and averaging of point clouds
#
#==============================================================================
#
# It compares new 3D points against the existing 3D points that are infront
# of the camera. It then updates the estimate for the actual position. Points
# are maintained as a mean with variance, but for the mean is simply used for
# all outputs at present. The 3D points are reprojected into a virtual image
# plane to provide keypoints in a common frame for the accumulated cloud,
# which are necessary for fitting fundamentals etc.
#
# This module aims to produce a set of averaged 3D points and corresponding 
# descriptors. Additionally, this is useful both for visualisation but also
# positioning from the cloud where it is particularly desireable that no two
# 3D points that can be viewed simultaneously have the matching descriptors
#
# The code is designed such that processing cost should scale with number of 
# unique points not total points as updates are iterative. Additionally, the
# structure is designed such that the entire cloud could be resampled (for
# example for changed drone position estimates after loop closure) but this
# would be fairly costly as all previous calcs would essentially need to be 
# repeated. In this case a non-iterative solution would probably be used.
#
#==============================================================================

import roslib; roslib.load_manifest('accumulator')
import rospy
import numpy as np
import cv2
import tf
import time
import std_msgs.msg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import Described3DPoints

class Positioner:    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Set up descriptor matcher
        self.matcher_type = 'None'
        # Initialise list of point clouds and list of descriptor sets
        self.frames = []
        self.acc = []
        self.mean = None # 3xM
        self.var = None # 3xM
        self.desc = None # Mxdesc_length
        self.first_time = True
        self.count = 0
        
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        rospy.Subscriber('/scan/relative_described_cloud',Described3DPoints,self.on_got_cloud)
        self.cloud_pub = rospy.Publisher('/accumulator/absolute_cloud',PointCloud)
        self.desc_pub = rospy.Publisher('/accumulator/absolute_described_cloud', Described3DPoints)
        
    def on_got_cloud(self, cloud):
        self.time_prev = time.time()
        prevtime = time.time()
        #print "Received new cloud"
        self.decode_and_buffer_message(cloud)
        new = self.frames[-1]
        #print "Decoded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        # Transform new points into world frame
        sub = np.add(new.points, new.position_i)
        new_pts = tf.transformations.quaternion_matrix(new.quat_i_to_w)[:3,:3].dot(sub)
        #print "Worlded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        self.count = self.count + new_pts.shape[1]
        
        if (self.first_time):
            self.first_time = False
            for i in range(new_pts.shape[1]):
                self.acc.append(Accumulated())
                self.acc[-1].pts = new_pts[:,i:i+1]
                self.acc[-1].pts2 = np.square(new_pts[:,i:i+1])
            self.mean = new_pts
            self.var = np.zeros(self.mean.shape)
            self.desc = new.desc
            self.kp = self.project_virtual_keypoints()
            self.publish_cloud()
            return
        
        # Get subset of points that lie infront of new pose
        # i.e. points that could have been seen
        kp_masked, desc_masked, shifts = self.get_subset(new.position_i, new.quat_i_to_w, new.quat_w_to_i)
        #print "Sectioned ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        original_length = self.mean.shape[1]
        
        # Match points
        kp1, kp2, indices1, indices2 = self.match_points(self.frames[-1].kp, kp_masked, new.desc, desc_masked)
        #print "Matched ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
            
        
        # Reconstruct indices for full self.acc, self.desc     
        for i in range(len(indices2)):
            indices2[i] = shifts[indices2[i]]
        #print "Rebuilt ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        # Get inliers with RANSAC fitted F
        #indices1, indices2 = self.extract_fundamental_and_filter_inliers(kp1, kp2, indices1, indices2)
        #print "Fundamentalled ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        #prevtime = time.time()
        
        mask = np.ones(new_pts.shape[1])
        Ns = np.zeros(len(indices2), dtype=np.float32)
        merged_count = 0
        for i in range(len(indices1)):
            # Mark point as averaged and update count
            mask[indices1[i]] = 0
            merged_count = merged_count + 1
            Ns[i] = self.acc[indices2[i]].pts.shape[1]
            # Append new info
            self.acc[indices2[i]].pts = np.hstack((self.acc[indices2[i]].pts, new_pts[:,indices1[i]:indices1[i]+1]))
            self.acc[indices2[i]].pts2 = np.hstack((self.acc[indices2[i]].pts2, np.square(new_pts[:,indices1[i]:indices1[i]+1])))
            '''# E(i)[X2] = Var(i)[X] + (E(i)[X])^2
            self.var[:,indices2[i]:indices2[i]+1] = self.var[:,indices2[i]:indices2[i]+1] + np.square(self.mean[:,indices2[i]:indices2[i]+1])
            # E(i+1)[X] = (E(i)[X]*(N-1) + xnew)/N
            self.mean[:,indices2[i]:indices2[i]+1] = (self.mean[:,indices2[i]:indices2[i]+1]*(self.acc[indices2[i]].pts.shape[1]-1.)+self.acc[indices2[i]].pts[:,-1:])/float(self.acc[indices2[i]].pts.shape[1])
            # Var(i+1)[X] = (E(i)[X2]*(N-1)+x2new)/N - (E(i+1)[X])^2
            self.var[:,indices2[i]:indices2[i]+1]  = (self.var[:,indices2[i]:indices2[i]+1]*(self.acc[indices2[i]].pts.shape[1]-1.)+self.acc[indices2[i]].pts2[:,-1:])/float(self.acc[indices2[i]].pts.shape[1]) - np.square(self.mean[:,indices2[i]:indices2[i]+1])'''
        # Efficiently iteratively update all means and vars
        # E(i)[X2] = Var(i)[X] + (E(i)[X])^2
        self.var[:,indices2] = self.var[:,indices2] + np.square(self.mean[:,indices2])
        # E(i+1)[X] = (E(i)[X]*(N-1) + xnew)/N
        self.mean[:,indices2] = (self.mean[:,indices2]*(Ns-1.)+new_pts[:,indices1])/Ns
        # Var(i+1)[X] = (E(i)[X2]*(N-1)+x2new)/N - (E(i+1)[X])^2
        self.var[:,indices2]  = self.var[:,indices2]*(Ns-1.)+np.square(new_pts[:,indices1])/Ns - np.square(self.mean[:,indices2])
        
        #print "Averaged ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        # Pre-allocate new mean and var array by adding ones for new cols
        self.mean = np.hstack((self.mean, np.ones((3,new_pts.shape[1]-merged_count))))
        self.var = np.hstack((self.var, np.ones((3,new_pts.shape[1]-merged_count))))
        
        # Add non-matched (i.e. new unique) points to the buffer
        added = 0
        for i in range(new_pts.shape[1]):
            if mask[i] == 1:
                self.acc.append(Accumulated())
                self.acc[-1].pts = new_pts[:,i:i+1]
                self.acc[-1].pts2 = np.square(new_pts[:,i:i+1])
                self.mean[:,original_length+added] = new_pts[:,i]
                self.var[:,original_length+added] = np.array([0.,0.,0.])
                self.desc = np.vstack((self.desc, new.desc[i,:]))
                added = added + 1
        #print "Newed ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
                
        self.publish_cloud()
        #print "Clouded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        self.kp = self.project_virtual_keypoints()
        #print "Re-projected ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        print "Point Cloud Accumulated ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) - (",self.mean.shape[1]," unique points out of ", self.count," )"
        
    def get_subset(self, position_i, quat_i_to_w, quat_w_to_i):
        camera_relative = np.add(tf.transformations.quaternion_matrix(quat_w_to_i)[:3,:3].dot(self.mean), -position_i)
        mask = camera_relative[2] > 0
        desc_masked = self.desc[mask, :]
        kp_masked = self.kp[mask, :]
        return kp_masked, desc_masked, np.cumsum(np.array(mask, dtype=np.int16))-1

    
    def decode_and_buffer_message(self, msg):
        # Need to re-numpy the float32 array descriptors were transmitted in
        self.points = np.reshape(np.array(msg.points3D), (3, -1))
        frame.kp = np.reshape(np.array(msg.points2D), (2, -1)).T
        if (msg.descriptors_matcher == 'BruteForce'):
            frame.desc= np.reshape(np.array(msg.descriptors, np.float32), (-1, msg.descriptors_stride))   
        elif (msg.descriptors_matcher == 'BruteForce-Hamming'):
            frame.desc = np.reshape(np.array(msg.descriptors, np.uint8), (-1, msg.descriptors_stride))
        # Switch matcher if necessary
        if (self.matcher_type != msg.descriptors_matcher):
            self.matcher_type = msg.descriptors_matcher
            self.dm = cv2.DescriptorMatcher_create(self.matcher_type)
            self.frames = []
            print "Auto-switching to ", self.matcher_type
        frame.desc_matcher = msg.descriptors_matcher
        frame.header = msg.header
        frame.position_i = np.array([msg.position_i]).T
        frame.quat_w_to_i = msg.quat_w_to_i
        frame.quat_i_to_w = msg.quat_i_to_w
        self.frames.append(frame)
        
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
        if len(comb) == 0:
            return None, None, [], []
        
        
        i1_indices = comb[0]
        i2_indices = comb[1]
        
        # Order pairs
        i1_pts = kp1[i1_indices,:]
        i2_pts = kp2[i2_indices,:]
        
        return i1_pts, i2_pts, np.array(i1_indices), np.array(i2_indices)
        
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
    rospy.init_node('Accumulator')
    
    # Print startup info
    print "\r\n"
    print "=========================== Accumulator ==========================="
    print " "
    print "==================================================================="
    print "\r\n"
    
    # Initialise controller
    a = Accumulator()    
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
