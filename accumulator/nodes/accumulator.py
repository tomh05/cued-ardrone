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
import os
import time
import math
import std_msgs.msg
import message_filters
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import Described3DPoints
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from copy import deepcopy

class Frame:
    def __init__(self):
        self.header = None
        self.position_i = None
        self.quat_w_to_i = None
        self.quat_i_to_w = None
        self.points = None
        self.kp = None
        self.desc = None
        self.desc_matcher = None
        
class Accumulated:
    def __init__(self):
        # There are M Accumulateds (one per unique 3D point)
        # There are N_m pts supporting each unique point
        self.pts =  None # This will be a 3xN numpy array
                         # [x_1, ...,x_N]
                         # [y_1, ...,y_N]
                         # [z_2, ...,z_N]
                         # This form is mainly used for programmatic simplicity
                         # May be more efficient than a Mx3xN due to smaller
                         # reallocations when points are added?
        self.pts2 = None # squared points

class Accumulator:    
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
        self.R = np.diag((1.,1.,1.))
        self.t = np.zeros((3,1))
        # Get nodes folder
        self.directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/debug'
        
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        rospy.Subscriber('/scan/relative_described_cloud',Described3DPoints,self.on_got_cloud)
        self.cloud_pub = rospy.Publisher('/accumulator/absolute_cloud',PointCloud)
        self.desc_pub = rospy.Publisher('/accumulator/absolute_described_cloud', Described3DPoints)
        self.fov_pub = rospy.Publisher('/accumulator/fov', PolygonStamped)
        self.testa_pub = rospy.Publisher('/cloud_compare/test_a',PointCloud)
        self.testb_pub = rospy.Publisher('/cloud_compare/test_b',PointCloud)
        self.testc_pub = rospy.Publisher('/cloud_compare/test_c', PointCloud)
        self.sub_pub = rospy.Publisher('/accumulator/sub', PointCloud)
        self.marker_pub = rospy.Publisher('/accumulator/marker', MarkerArray)
        
    def on_got_cloud(self, cloud):
        self.links = []
        self.stamp = cloud.header.stamp
        self.time_prev = time.time()
        prevtime = time.time()
        #print "Received new cloud"
        """
        # Decode Cloud
        """
        self.decode_and_buffer_message(cloud)
        new = self.frames[-1]
        #print "Decoded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        """
        # Transform new points into world frame
        """
        sub = np.add(new.points, new.position_i)
        new_pts = tf.transformations.quaternion_matrix(new.quat_i_to_w)[:3,:3].dot(sub)
        
        """
        # Refine cloud world position using accumulated R,t
        """
        #new_pts = np.add(new_pts, self.t)
        #new_pts

               
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
            self.prev_pts = new_pts
            return
            
            
        self.print_to_file(new_pts)
        self.match_to_prev_frame()   
        
        
        
        
        
        
        
             
        return #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            
            
            
        """
        # Publish accumulated-cloud pre-update
        """
        self.publish_cloud()
            
        #self.position_from_cloud_iterative(new_pts, self.frames[-1].kp)
            
        '''    
        # Build relative cloud
        for i in range(new_pts.shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = new_pts[0,i]
            cloud.points[-1].y = new_pts[1,i]
            cloud.points[-1].z = new_pts[2,i]
        self.testa_pub.publish(cloud)
        
        # Build relative cloud
        for i in range(self.prev_pts.shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = self.prev_pts[0,i]
            cloud.points[-1].y = self.prev_pts[1,i]
            cloud.points[-1].z = self.prev_pts[2,i]
        self.testa_pub.publish(cloud)
        
        self.prev_pts = new_pts
        '''
        
        """
        # Get subset of points that lie infront of new pose
        """
        # i.e. points that could have been seen
        pts_masked, kp_masked, desc_masked, shifts = self.get_subset_fov(new.position_i, new.quat_i_to_w, new.quat_w_to_i)
        #print "Sectioned ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        original_length = self.mean.shape[1]
        """
        # Match points
        """
        kp1, kp2, indices1, indices2 = self.match_points(self.frames[-1].kp, kp_masked, new.desc, desc_masked)
        #print "Matched ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        '''
        cloud = PointCloud()
        cloud.header.stamp = cloud.header.stamp
        cloud.header.frame_id = "/world"
        
        # Build active subset cloud
        for i in range(pts_masked.shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = pts_masked[0,i]
            cloud.points[-1].y = pts_masked[1,i]
            cloud.points[-1].z = pts_masked[2,i]
        self.sub_pub.publish(cloud)
        '''
        
        """
        # Reconstruct indices for full self.mean, self.desc (are currently for fov subset)
        """
        for i in range(len(indices2)):
            indices2[i] = indices2[i] + shifts[indices2[i]]
        #print "Rebuilt ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        
        cloud = PointCloud()
        cloud.header.stamp = cloud.header.stamp
        cloud.header.frame_id = "/world"
        
        # Build active subset cloud
        for i in range(self.mean[:,indices2].shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = (self.mean[:,indices2])[0,i]
            cloud.points[-1].y = (self.mean[:,indices2])[1,i]
            cloud.points[-1].z = (self.mean[:,indices2])[2,i]
            #print cloud.points[-1]
        self.sub_pub.publish(cloud)
        
        
        print "No of matches: ", len(indices2)
        

        
        
        
        
        
        
        
        
        
        
        
        merged_count = 0
        mask = np.ones(new_pts.shape[1])
        Ns = np.zeros(len(indices2), dtype=np.float32)
        
        if len(indices2) > 0:
            
            self.position_from_cloud(self.mean[:,indices2], kp1)
            
            """
            # Get inliers with RANSAC fitted F
            """
            indices1F, indices2F = self.extract_fundamental_and_filter_inliers(kp1, kp2, indices1, indices2)
            #print "Fundamentalled ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
            #prevtime = time.time()
            
            print "No of F fits: ", len(indices1F)
            
            if indices1F != None and len(indices1F) > 0:
                """
                # Publish matches for Rviz
                """
                
                line_list = Marker()
                line_list.header.frame_id = "/world"
                line_list.header.stamp = cloud.header.stamp
                line_list.action = 0 # Add/modify
                line_list.pose.orientation.w = 1.0
                line_list.type = 5 # LINE_LIST
                # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                line_list.scale.x = 0.01;

                # Line list is red
                line_list.color.r = 1.0;
                line_list.color.a = 1.0;



                # Create the vertices for the points and lines
                for i, ind in enumerate(indices1):
                
                  p = Point32()
                  p.x = new_pts[0,indices1[i]]
                  p.y = new_pts[1,indices1[i]]
                  p.z = new_pts[2,indices1[i]]
                  line_list.points.append(deepcopy(p))
                  p.x = self.mean[0,indices2[i]]
                  p.y = self.mean[1,indices2[i]]
                  p.z = self.mean[2,indices2[i]]
                  line_list.points.append(deepcopy(p))
                
                self.marker_pub.publish(line_list)
                
            if indices1F != None and len(indices1F) > 0:
                """
                # Publish matches for Rviz
                """
                
                line_list = Marker()
                line_list.header.frame_id = "/world"
                line_list.header.stamp = cloud.header.stamp
                line_list.action = 0 # Add/modify
                line_list.pose.orientation.w = 1.0
                line_list.type = 5 # LINE_LIST
                # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                line_list.scale.x = 0.01;

                # Line list is red
                line_list.color.b = 1.0;
                line_list.color.a = 1.0;



                # Create the vertices for the points and lines
                for i, ind in enumerate(indices1F):
                
                  p = Point32()
                  p.x = new_pts[0,indices1F[i]]
                  p.y = new_pts[1,indices1F[i]]
                  p.z = new_pts[2,indices1F[i]]
                  line_list.points.append(deepcopy(p))
                  p.x = self.mean[0,indices2F[i]]
                  p.y = self.mean[1,indices2F[i]]
                  p.z = self.mean[2,indices2F[i]]
                  line_list.points.append(deepcopy(p))
                
                self.marker2_pub.publish(line_list)
            
            # Attempt to position from existing points
            #self.position_from_cloud(self.mean[:,indices2], kp1)
            

            
            '''
            """
            # 2D overhead Rigid body alignment
            """
            if len(indices1F) > 8:
                
                
                pts1 = new_pts[:,indices1F]
                pts2 = self.mean[:,indices2F]
                
                cloud = PointCloud()
                cloud.header.stamp = cloud.header.stamp
                cloud.header.frame_id = "/world"
                
                # Build relative cloud
                for i in range(pts1.shape[1]):
                    cloud.points.append(Point32())
                    cloud.points[-1].x = pts1[0,i]
                    cloud.points[-1].y = pts1[1,i]
                    cloud.points[-1].z = pts1[2,i]
                self.testa_pub.publish(cloud)
                
                cloud = PointCloud()
                cloud.header.stamp = cloud.header.stamp
                cloud.header.frame_id = "/world"
                
                # Build relative cloud
                for i in range(pts2.shape[1]):
                    cloud.points.append(Point32())
                    cloud.points[-1].x = pts2[0,i]
                    cloud.points[-1].y = pts2[1,i]
                    cloud.points[-1].z = pts2[2,i]
                self.testb_pub.publish(cloud)
                
                
                R, t = self.approx_register_2D_t(new_pts[:,indices1F],self.mean[:,indices2F])
                print R
                print t
                new_pts = R.dot(new_pts)
                new_pts = np.add(new_pts, t)
                self.R = R
                self.t = t
                
                cloud = PointCloud()
                cloud.header.stamp = cloud.header.stamp
                cloud.header.frame_id = "/world"
                
                
                # Build relative cloud
                for i in range(new_pts.shape[1]):
                    cloud.points.append(Point32())
                    cloud.points[-1].x = new_pts[0,i]
                    cloud.points[-1].y = new_pts[1,i]
                    cloud.points[-1].z = new_pts[2,i]
                self.testc_pub.publish(cloud)
                
                
                cloud = PointCloud()
                cloud.header.stamp = cloud.header.stamp
                cloud.header.frame_id = "/world"
            else:
                #new_pts = self.R.dot(new_pts)
                new_pts = np.add(new_pts, self.t)
            '''
            
            # Get inliers with RANSAC fitted F
            #indices1F, indices2F = self.extract_fundamental_and_filter_inliers(kp1, kp2, indices1, indices2)
            #print "Fundamentalled ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
            #prevtime = time.time()
            #print len(indices1F)
            
            
            
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
                
        #self.publish_cloud()
        #print "Clouded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        self.kp = self.project_virtual_keypoints()
        #print "Re-projected ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        print "Point Cloud Accumulated ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) - (",self.mean.shape[1]," unique points out of ", self.count," )"  
        
    def print_to_file(self, pts3D):
        """ Takes 3D points in a 3xN arrangement and produces a 1cm resolution
        z flattened mono bitmap. x-y position is recorded relative to the min
        & max instances in the point set"""
        mins = np.floor(100*np.min(pts3D[:2,:],1))
        maxs = np.ceil(100*np.max(pts3D[:2,:],1))
        minx = int(mins[0])
        maxx = int(maxs[0])
        miny = int(mins[1])
        maxy = int(maxs[1])
        
        # Re-map pts3D into map co-ordinate system
        # Strip z
        pts = pts3D[:2]
        # Scale to integer cm
        pts = np.array(100.*pts, dtype=np.int16)
        # Flip y (np y is down, which confuses visualisation) & shift x
        pts[1,:] = pts[1,:] - miny
        pts[1,:] = (maxy - miny) - pts[1,:]
        pts[0,:] = pts[0,:] - minx
        # Pre-allocate white image
        img = 255*np.ones(shape=(maxy-miny+1,maxx-minx+1), dtype = np.uint8)
        
        # Draw points (this indexes the image using the prepared 2D points)
        img[pts[1,:],pts[0,:]] = 0
        cv2.imwrite(self.directory+'\\'+str(len(self.frames))+'.png', img)
        #cv2.imshow('test',img)
        #cv2.waitKey(100)
    
    
    def publish_links(self, pts1, pts2, indices1, indices2, rgba):
        """
        # Publish matches for Rviz
        """
        line_list = Marker()
        line_list.id = -(len(self.links) + 1)
        line_list.header.frame_id = "/world"
        line_list.header.stamp = self.stamp
        line_list.action = 0 # Add/modify
        line_list.pose.orientation.w = 1.0
        line_list.type = 5 # LINE_LIST
        # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = 0.001*(rgba[0]+6*rgba[1]+3*rgba[2]);

        # Line list is red
        line_list.color.r = rgba[0]
        line_list.color.g = rgba[1]
        line_list.color.b = rgba[2]
        line_list.color.a = rgba[3]

        # Create the vertices for the points and lines
        for i, ind in enumerate(indices1):
        
          p = Point32()
          p.x = pts1[0,indices1[i]]
          p.y = pts1[1,indices1[i]]
          p.z = pts1[2,indices1[i]]
          line_list.points.append(deepcopy(p))
          p.x = pts2[0,indices2[i]]
          p.y = pts2[1,indices2[i]]
          p.z = pts2[2,indices2[i]]
          line_list.points.append(deepcopy(p))
        
        self.links.append(line_list)
    
    def match_to_prev_frame(self):
        new = self.frames[-1]
        old = self.frames[-2]
        kp1, kp2, indices1, indices2 = self.match_points(new.kp, old.kp, new.desc, old.desc)
        
        # Convert 3D points to world space
        sub = np.add(new.points, new.position_i)
        pts1 = tf.transformations.quaternion_matrix(new.quat_i_to_w)[:3,:3].dot(sub)
        sub = np.add(old.points, old.position_i)
        pts2 = tf.transformations.quaternion_matrix(old.quat_i_to_w)[:3,:3].dot(sub)
        
        cloud = PointCloud()
        cloud.header.stamp = self.stamp
        cloud.header.frame_id = "/world"
        
        # Build new pts cloud
        for i in range(pts1.shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = pts1[0,i]
            cloud.points[-1].y = pts1[1,i]
            cloud.points[-1].z = pts1[2,i]
        self.testa_pub.publish(cloud)
        print "Published a"
        
        cloud = PointCloud()
        cloud.header.stamp = self.stamp
        cloud.header.frame_id = "/world"
        
        # Build old pts cloud
        for i in range(pts2.shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = pts2[0,i]
            cloud.points[-1].y = pts2[1,i]
            cloud.points[-1].z = pts2[2,i]
        self.testb_pub.publish(cloud)
        print "Published b"
        
        if indices1 != None and len(indices1) > 0:
            # This sorting has no functional effect but is useful for
            # debugging and has negligible cost
            argindex = indices1.argsort()
            indices1 = indices1[argindex]
            indices2 = indices2[argindex]
            print "No of Matches: ", len(indices1)
            self.publish_links(pts1, pts2, indices1, indices2, (1.,0.,0.,1.))
            
            """
            # Filter mismatched based on height, 3D distances and 
            """
            deltas = pts1[:,indices1]-pts2[:,indices2]
            median = np.array([np.median(deltas, 1)]).T
            
            # Filter on z
            maskz = np.abs(deltas[2,:]) < 0.25
            indices1D = indices1[maskz]
            indices2D = indices2[maskz]
            print "No of z ed: ", len(indices1D)
            
            # Update deltas & medians
            deltas = pts1[:,indices1D]-pts2[:,indices2D]
            median = np.array([np.median(deltas, 1)]).T
            
            # Filter on angle in x-y plane
            thetas = np.arctan2(deltas[1,:],deltas[0,:])
            median_theta = np.arctan2(median[1],median[0])
            
            dthetas = np.abs(thetas - median_theta)
            premask = dthetas > np.pi
            dthetas[premask] = np.abs(dthetas[premask] - 2*np.pi)
            maskt = dthetas < 22.5*np.pi/180.
            indices1D = indices1D[maskt]
            indices2D = indices2D[maskt]
            print "No of thetaed: ", len(indices1D)
            
            # Update deltas & medians
            deltas = pts1[:,indices1D]-pts2[:,indices2D]
            median = np.array([np.median(deltas, 1)]).T
            
            # Filter on magnitude in x-y plane
            # Note: Calculation is done on squared magnitudes so ratios are not
            # as they appear. A factor of 2 in this space is sqrt(2)
            median2 = np.sum(np.square(median[:2,:]),0)
            deltas2 = np.sum(np.square(deltas[:2,:]),0)
            mask1 = deltas2 < np.square(2.)*median2
            mask2 = deltas2 > np.square(0.5)*median2
            mask = np.logical_and(mask1, mask2)
            indices1D = indices1D[mask]
            indices2D = indices2D[mask]
            print "No of mag ed: ", len(indices1D)            
            
            self.publish_links(pts1, pts2, indices1D, indices2D, (0.,0.,1.,1.))
            
            """================================================================
            # Get inliers with RANSAC fitted F
            ================================================================"""
            subkp1 = new.kp[indices1D,:]
            subkp2 = old.kp[indices2D,:]
            indices1F, indices2F = self.extract_fundamental_and_filter_inliers(subkp1,subkp2, indices1D, indices2D)
            
            
            self.publish_links(pts1, pts2, indices1F, indices2F, (0.,1.,0.,1.))
            if indices1F != None and len(indices1F) > 0:
                print "No of F fits: ", len(indices1F)
                
                """
                # 2D overhead Rigid body alignment
                """
                if len(indices1F) > 4:
                    
                    # Care on subpoints
                    # R and t are calculated on subpts but apply to all pts
                    
                    # pts1 are the new pts
                    # pts2 are the old pts
                    # We need to shift the new pts onto the old pts
                    
                    R, t = self.approx_register_2D(pts1[:,indices1F],pts2[:,indices2F])
                    print "R\r\n", R
                    print "t\r\n", t
                    new_pts = R.dot(pts1)
                    new_pts = np.add(new_pts, t)
                    
                    
                    self.R = R
                    self.t = t

                    posw = tf.transformations.quaternion_matrix(new.quat_i_to_w)[:3,:3].dot(new.position_i)
                    print "Original position\r\n", posw
                    posw = R.dot(posw)
                    posw = np.add(posw, t)
                    print "Refined position\r\n", posw
                    
                    
                    cloud = PointCloud()
                    cloud.header.stamp = self.stamp
                    cloud.header.frame_id = "/world"
                    
                    
                    # Build relative cloud
                    for i in range(new_pts.shape[1]):
                        cloud.points.append(Point32())
                        cloud.points[-1].x = new_pts[0,i]
                        cloud.points[-1].y = new_pts[1,i]
                        cloud.points[-1].z = new_pts[2,i]
                    self.testc_pub.publish(cloud)
                    
                    
                    cloud = PointCloud()
                    cloud.header.stamp = self.stamp
                    cloud.header.frame_id = "/world"
                    
        self.marker_pub.publish(self.links)
    
    def position_from_cloud(self, pts3D_matched, kp_matched):

        
        """================================================================
        # Calculate pose and translation to matched template
        ================================================================"""
        
        if len(self.kp) < 5:
            "Not enough matches to localise"
            return
        
        
        R, t, inliers = cv2.solvePnPRansac(np.array(pts3D_matched, dtype=np.float32).T, np.array(kp_matched, dtype=np.float32), self.cameraMatrix, self.distCoeffs, reprojectionError=15.)
        
        
        if inliers == None:
            print "===================="
            print "Template not found"
            print "===================="
            return
        
        
        print "No. of inliers: ", len(inliers)   
        if len(inliers) > 4: 
            
            
            R, J = cv2.Rodrigues(R)        
            Rhomo = np.diag((1., 1., 1., 1.))
            Rhomo[:3, :3] = R
            
            t_position = -R.T.dot(t)
            print "t: ", t_position
        
            
  
            #header = sfwi.header
            
            # Publish tf
            #quat = tf.transformations.quaternion_inverse(tf.transformations.quaternion_from_matrix(Rhomo))
            #self.position_i = t_position
            #self.quat_w_to_i = quat 
            
    def position_from_cloud_iterative(self, new_pts, new_kp):
        """ This operates by cycling through all previous frames as recieved
        with the intention of producing an averaged pose estimate. This is
        incredibly costly as it requires a full matching for every frame in
        the index
        
        Fundamental extraction is NOT necessary nor desirable"""
        
        Rinit = tf.transformations.quaternion_matrix(self.frames[-1].quat_w_to_i)[:3,:3].T
        tinit = tf.transformations.quaternion_matrix(self.frames[-1].quat_i_to_w)[:3.,:3].dot(self.frames[-1].position_i)
        print "tinit: ", tinit
        print "Rinit:\r\n", Rinit
        Rinit, jacobian = cv2.Rodrigues(Rinit)
        print Rinit
        
        print "Attempting to position from all previous frames"
        for i, frame in enumerate(self.frames):
            if i < len(self.frames)-1:
            # For each previous frame
                # Transform to world frame
                sub = np.add(frame.points, frame.position_i)
                pts = tf.transformations.quaternion_matrix(frame.quat_i_to_w)[:3,:3].dot(sub)
                # Match
                kp1, kp2, indices1, indices2 = self.match_points(new_kp, frame.kp, self.frames[-1].desc, frame.desc)
                if len(indices1) > 8:
                    # Attempt to extract pose
                    
                    R, t, inliers = cv2.solvePnPRansac(np.array(pts[:,indices2], dtype=np.float32).T, np.array(kp1, dtype=np.float32), self.cameraMatrix, self.distCoeffs, rvec=Rinit, tvec=tinit, useExtrinsicGuess=True, reprojectionError=15.)
                    print "No. of inliers: ", len(inliers)  
                     
                    if inliers != None and len(inliers) > 8: 
                        R, J = cv2.Rodrigues(R)        
                        Rhomo = np.diag((1., 1., 1., 1.))
                        Rhomo[:3, :3] = R
                        t_position = -R.T.dot(t)
                        print "t: ", t_position
                        print "R:\r\n", R.T    
        
    def get_subset(self, position_i, quat_i_to_w, quat_w_to_i):
        camera_relative = np.add(tf.transformations.quaternion_matrix(quat_w_to_i)[:3,:3].dot(self.mean), -position_i)
        mask = camera_relative[2] > 0
        desc_masked = self.desc[mask, :]
        kp_masked = self.kp[mask, :]
        return kp_masked, desc_masked, np.cumsum(np.array(mask, dtype=np.int16))-1
        
    def get_subset_fov(self, position_i, quat_i_to_w, quat_w_to_i):
        # Convert accumulated points into local camera frame
        camera_relative = np.add(tf.transformations.quaternion_matrix(quat_w_to_i)[:3,:3].dot(self.mean), -position_i)
        mask_infront = camera_relative[2,:] > 0.
        camera_relative = camera_relative/camera_relative[2]
        
        bottom_left =  self.inverseCameraMatrix.dot(np.array([[-80.],[405.],[ 1.]]))
        top_right =  self.inverseCameraMatrix.dot(np.array([[720.],[-45.],[1.]]))
        
        offset = tf.transformations.quaternion_matrix(quat_i_to_w)[:3,:3].dot(position_i)
        spoly = PolygonStamped()
        poly = Polygon()
        p = Point32()
        p.x = bottom_left[0]*10.; p.y = top_right[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        p.x = bottom_left[0]*10.; p.y = bottom_left[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        p.x = top_right[0]*10.; p.y = bottom_left[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        p.x = top_right[0]*10.; p.y = top_right[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        p.x = bottom_left[0]*10.; p.y = top_right[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        p.x = 0.01; p.y = 0.01; p.z = 0.01
        poly.points.append(deepcopy(p))
        p.x = bottom_left[0]*10.; p.y = bottom_left[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        p.x = 0.01; p.y = 0.01; p.z = 0.01
        poly.points.append(deepcopy(p))
        p.x = top_right[0]*10.; p.y = bottom_left[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        p.x = 0.01; p.y = 0.01; p.z = 0.01
        poly.points.append(deepcopy(p))
        p.x = top_right[0]*10.; p.y = top_right[1]*10; p.z = 10.
        poly.points.append(deepcopy(p))
        spoly.polygon = poly
        spoly.header.frame_id = '/ardrone_base_frontcam'
        spoly.header.stamp = self.stamp
        #spoly.header.frame_id = '/world2'
        self.fov_pub.publish(spoly)
        
        print camera_relative.shape
        mask_left = camera_relative[0,:] > bottom_left[0]
        mask_bottom = camera_relative[1,:] < bottom_left[1]
        mask_right = camera_relative[0,:] < top_right[0]
        mask_top = camera_relative[1,:] > top_right[1]
        mask_hori = np.logical_and(mask_left, mask_right)
        mask_vert = np.logical_and(mask_top, mask_bottom)
        mask = np.logical_and(mask_hori, mask_vert)
        mask = np.logical_and(mask, mask_infront)
        
        # Invert mask so high for skipped values
        shifts = np.invert(mask)
        # Cumulative sum inverted mask to get shifts
        shifts = np.cumsum(np.array(shifts, dtype=np.int16))
        # Filter out non-skipped shifts
        shifts = shifts[mask]
        
        # shifts is such that the index in the full (self.mean) = index in the 
        # fov (pts_masked) + shifts[index in fov]
        
        
        desc_masked = self.desc[mask, :]
        kp_masked = self.kp[mask, :]
        pts_masked = self.mean[:,mask]
        
        return pts_masked, kp_masked, desc_masked, shifts
                
    def publish_cloud(self):      
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = "/world"
        
        # Build relative cloud
        for i in range(self.mean.shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = self.mean[0,i]
            cloud.points[-1].y = self.mean[1,i]
            cloud.points[-1].z = self.mean[2,i]
        self.cloud_pub.publish(cloud)
        
        described_cloud = Described3DPoints()
        described_cloud.header = cloud.header
        described_cloud.points3D = self.mean.reshape(-1,).tolist()
        described_cloud.points2D = self.kp.reshape(-1,).tolist()
        described_cloud.descriptors = self.desc.reshape(-1,).tolist()
        described_cloud.descriptors_stride = self.desc.shape[1]
        described_cloud.descriptors_matcher = self.frames[-1].desc_matcher
        self.desc_pub.publish(described_cloud)
        
    def project_virtual_keypoints(self):
        homo_mean = np.vstack((self.mean, np.ones((1,self.mean.shape[1]))))
        kp = self.virtual_projection.dot(homo_mean)
        kp = (kp/kp[2])[:2]
        return kp.T
        
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
        return R, t
        
    def approx_register_2D(self, pts1, pts2):
        # Pts are 3D world co-ords
        # Gets only R and t in rotation about z and t in x-y plane
        # De-couple translation by subtracting centrioids
        
        
        
        # pts1 = new points
        # pts2 = target frame points
        mean1 = np.array([np.mean(pts1, axis=1)]).T
        mean2 = np.array([np.mean(pts2, axis=1)]).T
        print "dmean:\r\n", mean2-mean1
        pts1_norm = np.add(pts1, -mean1)[:2]
        pts2_norm = np.add(pts2, -mean2)[:2]
        # Get covariance between point sets
        H = pts1_norm.dot(pts2_norm.T)
        # Recover least squares R
        U,S,Vt = np.linalg.svd(H)
        det = np.linalg.det(Vt.T.dot(U.T))
        print "det: ", det
        R2 = Vt.T.dot(np.diag((1.,det)).dot(U.T))
        R = np.diag((1.,1.,1.))
        R[:2,:2] = R2
        # Recover t that match least squares R
        t = np.zeros((3,1))
        print mean1
        print mean2
        t = (mean2 - R.dot(mean1))
        
        return R, t
        
    def approx_register_2D_t(self, pts1, pts2):
        # Pts are 3D world co-ords
        
        mean1 = np.array([np.mean(pts1, axis=1)]).T
        mean2 = np.array([np.mean(pts2, axis=1)]).T
        t = np.zeros((3,1))
        t[:2] = (mean2 - mean1)[:2]
        R = np.diag((1.,1.,1.))
        
        return R, t
        
    def approx_register_t(self, pts1, pts2):
        # De-couple translation by subtracting centrioids
        delta = np.add(-pts1, pts2)
        t = np.array([np.mean(delta, axis=1)]).T
        print t.shape
        R = np.diag((1.,1.,1.))
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
        frame.points = np.reshape(np.array(msg.points3D), (3, -1))
        print "Duplicate Test: ", len(np.unique(frame.points[0,:])) - len(frame.points[0,:])
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
        
    def match_2D3D_points(self, kp, pts3D, desc1, desc2):
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
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 10, param2 = 0.99)
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
