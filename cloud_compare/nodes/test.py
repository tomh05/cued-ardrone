#!/usr/bin/env python

#=============================== Cloud Test ===================================
# 
# This script generates point clouds and fake matches to test registration
#
#==============================================================================

import roslib; roslib.load_manifest('cloud_compare')
import rospy
import numpy as np
import cv2
import std_msgs.msg
import tf
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from custom_msgs.msg import DescribedPointCloud
from custom_msgs.msg import StampedStampedInt

class CloudComparer:
    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Create sample set
        self.create_point_model()
        # Set up timed callback
        rospy.Timer(rospy.Duration(10), self.on_timer_callback)
        
    def connect(self):
        self.match_pub = rospy.Publisher('/cloud_compare/matches',StampedStampedInt)
        self.cloud_pub = rospy.Publisher('/scan/relative_cloud',PointCloud)
        self.testa_pub = rospy.Publisher('/cloud_compare/test_a',PointCloud)
        self.testb_pub = rospy.Publisher('/cloud_compare/test_b',PointCloud)
        self.testc_pub = rospy.Publisher('/cloud_compare/test_c', PointCloud)
    
    def create_point_model(self):
        self.cloud_model = None
        
        #cloud.header.frame_id = "/world"
        #cloud.header.stamp = rospy.Time.from_sec(np.random.sample())
        
        xmin = 0.; xmax = 2.
        ymin = 2.; ymax = 2.
        zmin = 1.; zmax = 3.
        count = 256
        
        # Create random points
        self.cloud_model = np.array([np.random.rand(1, count)[0]*(xmax - xmin)+xmin,
                                     np.random.rand(1, count)[0]*(ymax - ymin)+ymin,
                                     np.random.rand(1, count)[0]*(zmax - zmin)+zmin])
            
    def sample_point_model(self, sigma=0.):  
        mask_gen = [np.random.rand(1, self.cloud_model.shape[1])<.5]
        mask = np.resize(mask_gen, (self.cloud_model.shape[0], self.cloud_model.shape[1]))
        masked_points = np.reshape(self.cloud_model[mask==True], (self.cloud_model.shape[0],-1))
        if sigma != 0.:
            noise = sigma*np.random.rand(masked_points.shape[0], masked_points.shape[1])
            masked_points = masked_points + noise
        return masked_points, mask_gen
        
    def transform_points(self, pts):
        t = np.random.rand(3, 1)
        a = np.random.rand(3,1)*np.pi*0.3
        R = tf.transformations.euler_matrix(a[0],a[1],a[2])[:3, :3]
        sub = R.dot(pts)
        sub = np.add(sub.T, t.T).T
        print "P:\r\n", np.hstack((R,t))
        return sub
        
    def approx_register(self, pts1, pts2, mask_a, mask_b):
        # This is an absolutely horrific way of recovering ordered lists of
        # matched points. Thankfully there's no need to clean this up as this
        # node is purely for debugging
        mask = np.logical_and(mask_a, mask_b)[0][0]
        # Re build pts1
        cum_a = np.cumsum(np.array(mask_a, np.int16))        
        cum_a_mask = np.prod((cum_a,np.array(mask, np.int16)), axis=0)
        cum_a_mask = cum_a_mask[cum_a_mask != 0]
        pts1_matched = []
        for i in cum_a_mask.tolist():
            pts1_matched.append(pts1.T[i-1].tolist())
        pts1_matched = np.array(pts1_matched).T
        # Re build pts2
        cum_b = np.cumsum(np.array(mask_b, np.int16))        
        cum_b_mask = np.prod((cum_b,np.array(mask, np.int16)), axis=0)
        cum_b_mask = cum_b_mask[cum_b_mask != 0]
        pts2_matched = []
        for i in cum_b_mask.tolist():
            pts2_matched.append(pts2.T[i-1].tolist())
        pts2_matched = np.array(pts2_matched).T
        
        print "Cross matches: ", pts1_matched.shape[1]
        
        
        # De-couple translation by subtracting centrioids
        mean1 = np.array([np.mean(pts1_matched, axis=1)]).T
        mean2 = np.array([np.mean(pts2_matched, axis=1)]).T
        print mean1
        print mean2
        pts1_norm = np.add(pts1_matched, -mean1)
        pts2_norm = np.add(pts2_matched, -mean2)
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
        
    
    def on_timer_callback(self, event):
        a, mask_a = self.sample_point_model()
        b, mask_b = self.sample_point_model(0.5)
        b = self.transform_points(b)
        R, t = self.approx_register(a,b, mask_a, mask_b)
        c = np.add(b.T, -t.T).T
        c = R.T.dot(c)        
        cloud_a = self.points_to_cloud(a)
        cloud_b = self.points_to_cloud(b)
        cloud_c = self.points_to_cloud(c)
        self.cloud_pub.publish(cloud_a)
        self.cloud_pub.publish(cloud_b)
        self.testa_pub.publish(cloud_a)
        self.testb_pub.publish(cloud_b)
        self.testc_pub.publish(cloud_c)
        self.publish_matches(min(a.shape[1], b.shape[1]), cloud_a.header, cloud_b.header)
        print "Published\r\n"
        
        
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
    
    def publish_matches(self, count, header1, header2):
        stamped_stamped_int = StampedStampedInt()
        stamped_stamped_int.header1 = header1
        stamped_stamped_int.header2 = header2
        stamped_stamped_int.count = count
        self.match_pub.publish(stamped_stamped_int)


def run():
    rospy.init_node('Cloud_Comparer')
    # Initialise controller
    c = CloudComparer()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
