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
from custom_msgs.msg import DescribedPointCloud
from custom_msgs.msg import StampedStampedInt

class CloudComparer:
    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Set up descriptor matcher
        self.dm = cv2.DescriptorMatcher_create('BruteForce')
        # Initialise list of point clouds and list of descriptor sets
        self.cloud_buffer = []
        self.kp_desc_buffer = []
        
    def connect(self):
        rospy.Subscriber('/scan/relative_described_cloud',DescribedPointCloud,self.on_got_cloud)
        self.match_pub = rospy.Publisher('/cloud_compare/matches',StampedStampedInt)
        
    def on_got_cloud(self, cloud):
        # Get np formatted descriptors
        new_kp, new_desc = self.get_keypoints_and_descriptors(cloud)
        
        for kp, desc, header in self.kp_desc_buffer:
            print desc.shape
            print new_desc.shape
            pts1, pts2 = self.match_points(kp, new_kp, desc, new_desc)
            if len(pts1) > 4:
                print len(pts1), " Desc matches"
                F, pts1_corr, pts2_corr = self.extract_fundamental(pts1, pts2)
                if len(pts1_corr) > 0:
                    print len(pts1_corr), " F matches"
                    self.approx_register()
                    self.publish_matches(len(pts1_corr), header, cloud.cloud.header) 
        
        self.kp_desc_buffer.append(( new_kp, new_desc, cloud.cloud.header))
        
    def approx_register(self, pts1, pts2):
        # De-couple mean 
    
    def publish_matches(self, count, header1, header2):
        stamped_stamped_int = StampedStampedInt()
        stamped_stamped_int.header1 = header1
        stamped_stamped_int.header2 = header2
        stamped_stamped_int.count = count
        self.match_pub.publish(stamped_stamped_int)
    
    def get_keypoints_and_descriptors(self, cloud):
        # Need to re-numpy the float32 array descriptors were transmitted in
        kp = np.reshape(np.array(cloud.kp), (-1, 2))
        desc = np.reshape(np.array(cloud.descriptors, np.float32), (-1, cloud.desc_stride))        
        return kp, desc
        
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
        i1_indices = comb[0]
        i2_indices = comb[1]
        
        # Order pairs
        i1_pts = kp1[i1_indices,:]
        i2_pts = kp2[i2_indices,:]
        
        return i1_pts, i2_pts
        
    def extract_fundamental(self, i1_pts_undistorted, i2_pts_undistorted):
        """Extract fundamental matrix and then remove outliers"""        
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 3, param2 = 0.99)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_corr = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        return F, i1_pts_corr, i2_pts_corr


def run():
    rospy.init_node('Cloud_Comparer')
    # Initialise controller
    c = CloudComparer()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
