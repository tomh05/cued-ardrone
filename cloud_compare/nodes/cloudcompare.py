#!/usr/bin/env python

#=============================== Cloud Compare ================================
# 
# This script handles the 2D feature matching of point clouds
#
#==============================================================================
#
# Point clouds are received with a SIFT descriptor corresponding to every 3D
# point. It is assumed that each point cloud in itself is accurate. 
#
# To determine whether point clouds correspond to previously captured point
# clouds, features are matched between each possible pairing of clouds. The
# fundamental matrix is then RANSAC fitted to determine whether the
# correspondencies maintain correct ordering in the overall image. The
# number of matches is then published along with relative point cloud.
#
# These values are intended to be used to determine which point clouds should
# be tried for registration and merging using pcl registration algorithms.
#
#==============================================================================
#
# As 2D features are never refined, we need to only process new comparisons.
# This corresponds to (N-1) sets of descriptor matches per new cloud where
# there are a total of N clouds. This could be further reduced by only 
# considering pairs of clouds close in a euclidean sense.
#
#==============================================================================

import roslib; roslib.load_manifest('cloud_compare')
import rospy
import numpy as np
import cv2
import std_msgs.msg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from custom_msgs.msg import DescribedPointCloud
from custom_msgs.msg import StampedStampedInt

class CloudComparer:
    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Set up descriptor matcher
        self.matcher_type = 'None'
        # Initialise list of point clouds and list of descriptor sets
        self.kp_desc_buffer = []
        
    def connect(self):
        rospy.Subscriber('/scan/relative_described_cloud',DescribedPointCloud,self.on_got_cloud)
        self.match_pub = rospy.Publisher('/cloud_compare/matches',StampedStampedInt)
        self.testa_pub = rospy.Publisher('/cloud_compare/test_a',PointCloud)
        self.testb_pub = rospy.Publisher('/cloud_compare/test_b',PointCloud)
        self.testc_pub = rospy.Publisher('/cloud_compare/test_c', PointCloud)
        
    def on_got_cloud(self, cloud):
        # Get np formatted descriptors
        new_kp, new_desc, new_pts_3D = self.decode_message(cloud)
        
        for kp, desc, header, pts_3D in self.kp_desc_buffer:
            print desc.shape
            print new_desc.shape
            pts1, pts2, pts1_3D, pts2_3D = self.match_points(kp, new_kp, desc, new_desc, pts_3D, new_pts_3D)
            if len(pts1) > 8:
                print len(pts1), " Desc matches"
                F, pts1_corr, pts2_corr, pts1_3D_corr, pts2_3D_corr = self.extract_fundamental(pts1, pts2, pts1_3D, pts2_3D)
                if len(pts1_corr) > 8:
                    print len(pts1_corr), " F matches"
                    R, t = self.approx_register(pts1_3D_corr.T, pts2_3D_corr.T)
                    c = np.add(new_pts_3D, -t.T).T
                    c = R.T.dot(c)        
                    cloud_a = self.points_to_cloud(pts_3D.T)
                    cloud_b = self.points_to_cloud(new_pts_3D.T)
                    cloud_c = self.points_to_cloud(c)
                    self.testa_pub.publish(cloud_a)
                    self.testb_pub.publish(cloud_b)
                    self.testc_pub.publish(cloud_c) 
                    self.publish_matches(len(pts1_corr), header, cloud.header) 
        
        self.kp_desc_buffer.append(( new_kp, new_desc, cloud.header, new_pts_3D))
        
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
    
    def decode_message(self, cloud):
        # Need to re-numpy the float32 array descriptors were transmitted in
        kp = np.reshape(np.array(cloud.points), (-1, 2))
        cloud_points = np.reshape(np.array(cloud.cloud_points).T, (-1, 3))
        if (cloud.descriptors_matcher == 'BruteForce'):
            desc = np.reshape(np.array(cloud.descriptors, np.float32), (-1, cloud.descriptors_stride))   
        elif (cloud.descriptors_matcher == 'BruteForce-Hamming'):
            desc = np.reshape(np.array(cloud.descriptors, np.uint8), (-1, cloud.descriptors_stride))
        # Switch matcher if necessary
        if (self.matcher_type != cloud.descriptors_matcher):
            self.matcher_type = cloud.descriptors_matcher
            self.dm = cv2.DescriptorMatcher_create(self.matcher_type)
            self.kp_desc_buffer = []
            print "Auto-switching to ", self.matcher_type
        return kp, desc, cloud_points
        
    def match_points(self, kp1, kp2, desc1, desc2, pts1_3D, pts2_3D):
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
        
        # Order pairs
        i1_pts = kp1[i1_indices,:]
        i2_pts = kp2[i2_indices,:]
        
        # Filter 3D points
        pts1_3D_mat = np.array(list(pts1_3D[i] for i in i1_indices))
        pts2_3D_mat = np.array(list(pts2_3D[i] for i in i2_indices))
        
        return i1_pts, i2_pts, pts1_3D_mat, pts2_3D_mat
        
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
    rospy.init_node('Cloud_Comparer')
    # Initialise controller
    c = CloudComparer()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
