#!/usr/bin/env python

#=============================== Accumulator===================================
# 
# This script handles the accumulation and averaging of point clouds
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
from custom_msgs.msg import Described3DPoints

class Frame:
    def __init__(self):
        self.header = None
        self.position_i = None
        self.quat_w_to_i = None
        self.quat_i_to_w = None
        self.points = None
        self.desc = None
        self.desc_matcher = None
        
class Accumulated:
    def __init__(self):
        self.pts =  None# This will be a 3xN numpy array, which is 
                        # arguably more efficient than a Mx3xN numpy due to less
                        # reallocation as points are added
        self.pts2 = None    # squared points
        self.mean = None 
        self.var = None 

class Accumulator:    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Set up descriptor matcher
        self.matcher_type = 'None'
        # Initialise list of point clouds and list of descriptor sets
        self.frames = []
        self.acc = []
        self.desc = None
        self.first_time = True
        
    def connect(self):
        rospy.Subscriber('/scan/relative_described_cloud',Described3DPoints,self.on_got_cloud)
        print "subbed"
        self.cloud_pub = rospy.Publisher('/accumulator/absolute_cloud',PointCloud)
        self.testb_pub = rospy.Publisher('/cloud_compare/test_b',PointCloud)
        self.testc_pub = rospy.Publisher('/cloud_compare/test_c', PointCloud)
        
    def on_got_cloud(self, cloud):
        self.time_prev = time.time()
        prevtime = time.time()
        print "Received new cloud"
        self.decode_and_buffer_message(cloud)
        new = self.frames[-1]
        print "Decoded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        # Transform new points into world frame
        sub = np.add(new.points, new.position_i)
        new_pts = tf.transformations.quaternion_matrix(new.quat_i_to_w)[:3,:3].dot(sub)
        print "Worlded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        if (self.first_time):
            self.first_time = False
            for i in range(new_pts.shape[1]):
                self.acc.append(Accumulated())
                self.acc[-1].pts = new_pts[:,i:i+1]
                self.acc[-1].pts2 = np.square(new_pts[:,i:i+1])
                self.acc[-1].mean = new_pts[:,i]
                self.acc[-1].var = np.array([0.,0.,0.])#np.mean(acc.pts2, axis=1) - np.square(np.mean(acc.pts, axis=1))
            self.desc = new.desc
            print "---\r\n",self.acc[-1].pts
            return
        
        
        # Get subset of points that lie infront of new pose
        # i.e. points that could have been seen
        #pts, desc = self.get_subset(new.position_i, new.quat_i_to_w, new.quat_w_to_i)
        
        
        # Match points
        indices1, indices2 = self.match_points(new.desc, self.desc)
        print "Matched ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        mask = np.ones(new_pts.shape[1])
        # Average matched points
        for i in range(len(indices1)):
            # Mark point as averaged
            mask[indices1[i]] = 0
            # Append new info
            self.acc[indices2[i]].pts = np.hstack((self.acc[indices2[i]].pts, new_pts[:,indices1[i]:indices1[i]+1]))
            #print "merged: ", self.acc[indices2[i]].pts
            self.acc[indices2[i]].pts2 = np.hstack((self.acc[indices2[i]].pts2, np.square(new_pts[:,indices1[i]:indices1[i]+1])))
            #print "merged2: ", self.acc[indices2[i]].pts2
            # This could be done iteratively instead
            self.acc[indices2[i]].var = self.acc[indices2[i]].var + np.square(self.acc[indices2[i]].mean) # E(i)[X2] = Var(i)[X] + (E(i)[X])2
            self.acc[indices2[i]].mean = (self.acc[indices2[i]].mean*(self.acc[indices2[i]].pts.shape[1]-1.)+self.acc[indices2[i]].pts[:,-1])/float(self.acc[indices2[i]].pts.shape[1])
            self.acc[indices2[i]].var = (self.acc[indices2[i]].var*(self.acc[indices2[i]].pts.shape[1]-1.)+self.acc[indices2[i]].pts2[:,-1])/float(self.acc[indices2[i]].pts.shape[1]) - np.square(self.acc[indices2[i]].mean)
            #self.acc[indices2[i]].var2 = np.mean(self.acc[indices2[i]].pts2, axis=1) - np.square(np.mean(self.acc[indices2[i]].pts, axis=1))
        print "Averaged ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
            
        # Add non-matched (i.e. new unique) points to the buffer
        for i in range(new_pts.shape[1]):
            if mask[i] == 1:
                self.acc.append(Accumulated())
                self.acc[-1].pts = new_pts[:,i:i+1]
                self.acc[-1].pts2 = np.square(new_pts[:,i:i+1])
                self.acc[-1].mean = new_pts[:,i]
                self.acc[-1].var = np.array([0.,0.,0.])
                self.desc = np.vstack((self.desc, new.desc[i,:]))
        print "Newed ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
                
        self.publish_cloud()
        print "Clouded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        print "Point Cloud Accumulated ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) "
                
    def publish_cloud(self):      
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = "/world"
        
        # Build relative cloud
        for i, acc in enumerate(self.acc):
            cloud.points.append(Point32())
            cloud.points[-1].x = acc.mean[0]
            cloud.points[-1].y = acc.mean[1]
            cloud.points[-1].z = acc.mean[2]
        self.cloud_pub.publish(cloud) 
        
    def points_to_cloud(self, pts):
        cloud = PointCloud()
        cloud.header.frame_id = "/world"
        cloud.header.stamp = rospy.Time.from_sec(np.random.sample())
        
        # Reshape for easy clouding
        sub = zip(*np.vstack((pts[0], pts[1], pts[2])))

        # Build absolute cloud
        for i, p in enumerate(sub):
            cloud.points.append(Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
            
        return cloud
        
    def approx_register(self, pts1, pts2):
        # De-couple translation by subtracting centrioids
        mean1 = np.array([np.mean(pts1, axis=1)]).T
        mean2 = np.array([np.mean(pts2, axis=1)]).T
        pts1_norm = np.add(pts1, -mean1)
        pts2_norm = np.add(pts2, -mean2)
        # Get covariance between point sets
        r1 = np.sum(np.multiply(pts1_norm[0:1].T,(pts2_norm.T)), axis=0)
        r2 = np.sum(np.multiply(pts1_norm[1:2].T,(pts2_norm.T)), axis=0)
        r3 = np.sum(np.multiply(pts1_norm[2:3].T,(pts2_norm.T)), axis=0)
        H = np.vstack((r1,r2,r3))
        # Recover least squares R
        U,S,Vt = np.linalg.svd(H)
        R = Vt.T.dot(U.T)
        # Recover t that match least squares R
        t = mean2 - R.dot(mean1)
        print np.hstack((R,t))
        return R, t
        
    
    def publish_matches(self, count, header1, header2):
        stamped_stamped_int = StampedStampedInt()
        stamped_stamped_int.header1 = header1
        stamped_stamped_int.header2 = header2
        stamped_stamped_int.count = count
        self.match_pub.publish(stamped_stamped_int)
    
    def decode_and_buffer_message(self, msg):
        frame = Frame()
        # Need to re-numpy the float32 array descriptors were transmitted in
        frame.points = np.reshape(np.array(msg.points), (3, -1))
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
        
    def match_points(self, desc1, desc2):
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
        
        
        return i1_indices, i2_indices
        
    def extract_fundamental(self, i1_pts_undistorted, i2_pts_undistorted, pts1_3D, pts2_3D):
        """Extract fundamental matrix and then remove outliers"""        
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 3, param2 = 0.99)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_corr = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        # NB: transposes are necessary for resize to replicate entries correctly
        mask_3D = np.resize(mask.T, (pts1_3D.shape[1],pts2_3D.shape[0])).T 
        pts1_3D_corr = np.reshape(pts1_3D[mask_3D], (-1, pts1_3D.shape[1]))
        pts2_3D_corr = np.reshape(pts2_3D[mask_3D], (-1, pts2_3D.shape[1]))
        return F, i1_pts_corr, i2_pts_corr, pts1_3D_corr, pts2_3D_corr


def run():
    rospy.init_node('Accumulator')
    # Initialise controller
    a = Accumulator()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
