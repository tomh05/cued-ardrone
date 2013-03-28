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
    """ Stores all received clouds untouched """
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
    """ Stores all the points associated with a given 2D3D feature 
    Only written not read at present, but allows for resampling """
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

class Accumulator:    
    def __init__(self):
        # Initialise ROS node
        self.connect()
        # Set up descriptor matcher
        self.matcher_type = 'None'
        # Initialise list of received info
        self.frames = []
        # Initialise list of accumulated info
        self.acc = []
        self.mean = None # 3xM
        self.var = None # 3xM
        self.desc = None # Mxdesc_length
        self.first_time = True
        self.count = 0
        # Initialise accumulated correction
        self.R = np.diag((1.,1.,1.))
        self.t = np.zeros((3,1))
        # Get nodes folder for debug
        self.directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'\\debug'
        
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        rospy.Subscriber('/scan/relative_described_cloud',Described3DPoints,self.on_got_cloud)        
        self.desc_pub = rospy.Publisher('/accumulator/absolute_described_cloud', Described3DPoints)
        self.desc_shifted_pub = rospy.Publisher('/accumulator/shifted_described_cloud', Described3DPoints)
        self.fov_pub = rospy.Publisher('/accumulator/fov', PolygonStamped)
        self.marker_pub = rospy.Publisher('/accumulator/marker', MarkerArray)
        self.br = tf.TransformBroadcaster()
        """
        # Cloud publishers
        """
        # Cloud for complete accumulated points for latest frame
        self.cloud_pub = rospy.Publisher('/accumulator/absolute_cloud',PointCloud)
        # Cloud for complete accumulate points for previous frame
        # (This is useful for seeing what is being matched to)
        self.cloud_pre_pub = rospy.Publisher('/accumulator/absolute_pre_cloud',PointCloud)
        # Cloud for accumulated points that match new points (pre filtering)
        self.cloud_matched_pub = rospy.Publisher('/accumulator/matched', PointCloud)        
        # Cloud for new points shifted by accumulated correction
        self.cloud_newpts_pub = rospy.Publisher('/accumulator/newpts',PointCloud)
        # Cloud for new points after full alignment
        self.cloud_shifted_pub = rospy.Publisher('/accumulator/shifted',PointCloud)
        
    def on_got_cloud(self, cloud):
        """ Main callback. Carries out full accumulation operation """
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
        # Refine new points
        new_pts = self.R.dot(new_pts)
        new_pts = np.add(new_pts, self.t)
        # Estimate refined drone position
        Riwcorr = self.R.dot(tf.transformations.quaternion_matrix(new.quat_i_to_w)[:3,:3])
        position_wcorr = Riwcorr.dot(new.position_i) + self.t
        position_icorr = Riwcorr.T.dot(position_wcorr)
        Rhomo = np.diag((1.,1.,1.,1.))
        Rhomo[:3,:3] = Riwcorr
        quat_i_to_wcorr = tf.transformations.quaternion_from_matrix(Rhomo)
        quat_w_to_icorr = tf.transformations.quaternion_from_matrix(Rhomo.T)
        self.br.sendTransform((position_wcorr[0],position_wcorr[1],position_wcorr[2]), 
                         # translation happens first, then rotation
                         quat_i_to_wcorr,
                         self.stamp,
                         "/ardrone_corrected_cam",
                         "/world") 
               
        #print "Worlded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        self.count = self.count + new_pts.shape[1]
        if (self.first_time):
            # Accumulated = first cloud on first run
            self.first_time = False
            for i in range(new_pts.shape[1]):
                self.acc.append(Accumulated())
                self.acc[-1].pts = new_pts[:,i:i+1]
            self.mean = new_pts
            self.var = np.zeros(self.mean.shape)
            self.desc = new.desc
            self.kp = self.project_virtual_keypoints()
            self.publish_cloud_accumulated()
            self.prev_pts = new_pts
            return
            
        """
        # Publish accumulated-cloud pre-update
        """
        self.publish_cloud(self.mean, self.cloud_pre_pub)
        
        """
        # Get subset of points that lie in fov of new pose
        """
        pts_masked, kp_masked, desc_masked, shifts = self.get_subset_fov(position_icorr, quat_i_to_wcorr, quat_w_to_icorr)
        #print "Sectioned ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()        
        
        """
        # Match points (NB: indices2 is from fov subset)
        """
        kp1, kp2, indices1, indices2 = self.match_points(self.frames[-1].kp, kp_masked, new.desc, desc_masked)
        #print "Matched ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        """
        # Reconstruct indices2 for full self.mean & self.desc
        """
        for i in range(len(indices2)):
            indices2[i] = indices2[i] + shifts[indices2[i]]
        #print "Rebuilt ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        """
        # Constrain 2D matches using 3D info
        """
        indices1, indices2 = self.constrain_matches(self.frames[-1].kp, self.kp, new_pts, self.mean, indices1, indices2)        
        
        merged_count = 0
        mask = np.ones(new_pts.shape[1])
        Ns = np.zeros(len(indices2), dtype=np.float32)
        original_length = self.mean.shape[1]        
        if len(indices2) > 6:      
            """
            # Register new pts to accumulated cloud
            """
            R, t = self.approx_register_2D(new_pts[:,indices1],self.mean[:,indices2])
            new_pts = R.dot(new_pts)
            new_pts = np.add(new_pts, t)
            
            """
            # Reject indices if registration was rejected (i.e: Rot === 0)
            """
            if R[0,0] == 1. and t[0] == 0.:
                print "Safety detected"
                indices1 = np.array([],dtype=np.float32)
                indices2 = np.array([],dtype=np.float32)
            else:
                """
                # Accumulate refinements to position
                """
                self.R = R.dot(self.R)
                self.t = R.dot(self.t)+t
                
                """
                # Add new pts into the listings
                """
                for i in range(len(indices1)):
                    # Mark point as averaged and update count
                    mask[indices1[i]] = 0
                    merged_count = merged_count + 1
                    Ns[i] = self.acc[indices2[i]].pts.shape[1]
                    # Append new info
                    self.acc[indices2[i]].pts = np.hstack((self.acc[indices2[i]].pts, new_pts[:,indices1[i]:indices1[i]+1]))
                """
                # Efficiently iteratively update all means and vars simultaneously
                """
                # E(i)[X2] = Var(i)[X] + (E(i)[X])^2
                self.var[:,indices2] = self.var[:,indices2] + np.square(self.mean[:,indices2])
                # E(i+1)[X] = (E(i)[X]*(N-1) + xnew)/N
                self.mean[:,indices2] = (self.mean[:,indices2]*(Ns-1.)+new_pts[:,indices1])/Ns
                # Var(i+1)[X] = (E(i)[X2]*(N-1)+x2new)/N - (E(i+1)[X])^2
                self.var[:,indices2]  = self.var[:,indices2]*(Ns-1.)+np.square(new_pts[:,indices1])/Ns - np.square(self.mean[:,indices2])                
                #print "Averaged ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
                prevtime = time.time()
        
        """
        # Add non-matched (i.e. new unique) points to the buffer
        """
        # Pre-allocate new mean and var array for new cols
        self.mean = np.hstack((self.mean, np.empty((3,new_pts.shape[1]-merged_count))))
        self.var = np.hstack((self.var, np.empty((3,new_pts.shape[1]-merged_count))))
        added = 0
        for i in range(new_pts.shape[1]):
            if mask[i] == 1:
                self.acc.append(Accumulated())
                self.acc[-1].pts = new_pts[:,i:i+1]
                self.mean[:,original_length+added] = new_pts[:,i]
                self.var[:,original_length+added] = np.array([0.,0.,0.])
                self.desc = np.vstack((self.desc, new.desc[i,:]))
                added = added + 1
        #print "Newed ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()
        
        """
        # Publish accumulated cloud along with described version
        """
        self.publish_cloud_accumulated()
        #print "Clouded ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        prevtime = time.time()        
        
        """
        # Update the accumulated cloud's keypoints
        """
        self.kp = self.project_virtual_keypoints()
        
        #print "Re-projected ( ", np.around(((time.time()-prevtime)*1000),2), "ms) "
        print "Point Cloud Accumulated ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) - (",self.mean.shape[1]," unique points out of ", self.count," )"
        
        """
        # Publish aligned points for use in gridding
        """
        self.publish_cloud(new_pts, self.cloud_shifted_pub)
        self.br.sendTransform((position_wcorr[0],position_wcorr[1],position_wcorr[2]),quat_i_to_wcorr,self.stamp,"/ardrone_aligned_cam","/world") 
        position_wcorr = Riwcorr.dot(new.position_i) + self.t
        position_icorr = Riwcorr.T.dot(position_wcorr)        
        Rhomo = np.diag((1.,1.,1.,1.))
        Rhomo[:3,:3] = Riwcorr
        quat_i_to_wcorr = tf.transformations.quaternion_from_matrix(Rhomo)
        quat_w_to_icorr = tf.transformations.quaternion_from_matrix(Rhomo.T)
        Riwcorr = self.R.dot(tf.transformations.quaternion_matrix(new.quat_i_to_w)[:3,:3])
        camera_relative = np.add(Riwcorr.T.dot(new_pts), -position_icorr)
        described_cloud = Described3DPoints()
        described_cloud.header.stamp = self.stamp
        described_cloud.header.frame_id = "/ardrone_aligned_cam"
        described_cloud.points3D = camera_relative.reshape(-1,).tolist()
        described_cloud.position_i = position_icorr.reshape(-1,).tolist()
        described_cloud.quat_i_to_w = quat_i_to_wcorr
        described_cloud.quat_w_to_i = quat_w_to_icorr
        self.desc_shifted_pub.publish(described_cloud)
        self.publish_cloud(camera_relative, self.cloud_shifted_pub, '/ardrone_aligned_cam')
        
    def print_to_file(self, pts3D): # DEBUG ONLY
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
    
    
    def add_links(self, pts1, pts2, indices1, indices2, rgba):
        """
        # Preps links for publishing in single marker array
        # This function should be called for each line_list
        # Followed by actually publishing the self.links array
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
    
    @staticmethod
    def trimmed_mean(array, percentage = 0.3):
        """ Returns the mean of the array values that fall in the the mid
        percentage range """
        # Sort array for x,y,z independently
        # NB: A column no longer corresponds to a point
        array.sort()
        lower = np.floor(array.shape[1]*percentage/2.)
        upper = array.shape[1]-lower
        return np.array([np.mean(array[:, lower:upper],1)]).T
        
    
    def constrain_matches(self, rawkp1, rawkp2, pts1, pts2, indices1, indices2):
        """ Ensures 2D matches comply with the 3D structure
        1 = new, 2 = old (accumulated) """
        
        kp1 = rawkp1[indices1,:]
        kp2 = rawkp2[indices2,:]
        
        # Publish clouds for the pts considered
        self.publish_cloud(pts1, self.cloud_newpts_pub)
        self.publish_cloud(pts2, self.cloud_matched_pub)
        
        # Create empty indices to return in case of failure
        indices1F = np.array([],dtype=np.float32)
        indices2F = np.array([],dtype=np.float32)
        if indices1 != None and len(indices1) > 0:
            # This sorting has no functional effect but is useful for
            # debugging and has negligible cost
            argindex = indices1.argsort()
            indices1 = indices1[argindex]
            indices2 = indices2[argindex]
            
            print "No of Matches: ", len(indices1)
            self.add_links(pts1, pts2, indices1, indices2, (1.,0.,0.,1.))
            
            """
            # Filter mismatched based on height, 3D distances and 
            """
            deltas = pts1[:,indices1]-pts2[:,indices2]
            median = np.array([np.median(deltas,1)]).T
            
            # Filter on z
            maskz = np.abs(deltas[2,:]) < 0.25
            indices1D = indices1[maskz]
            indices2D = indices2[maskz]
            print "No of z ed: ", len(indices1D)
            
            # Update deltas & medians
            deltas = pts1[:,indices1D]-pts2[:,indices2D]
            median = np.array([np.median(deltas,1)]).T
            
            # Filter on angle in x-y plane
            thetas = np.arctan2(deltas[1,:],deltas[0,:])
            print thetas
            median_theta = np.arctan2(median[1],median[0])
            print median_theta
            dthetas = np.abs(thetas - median_theta)
            premask = dthetas > np.pi
            dthetas[premask] = np.abs(dthetas[premask] - 2*np.pi)
            maskt = dthetas < 22.5*np.pi/180.
            indices1D = indices1D[maskt]
            indices2D = indices2D[maskt]
            print "No of thetaed: ", len(indices1D)
            
            # Update deltas & medians
            deltas = pts1[:,indices1D]-pts2[:,indices2D]
            median = np.array([np.median(deltas,1)]).T
            
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
            
            self.add_links(pts1, pts2, indices1D, indices2D, (0.,0.,1.,1.))
            
            """================================================================
            # Get inliers with RANSAC fitted F
            ================================================================"""            
            if indices1D != None and len(indices1D) > 0:
                subkp1 = rawkp1[indices1D,:]
                subkp2 = rawkp2[indices2D,:]
                indices1F, indices2F = self.extract_fundamental_and_filter_inliers(subkp1,subkp2, indices1D, indices2D)            
            self.add_links(pts1, pts2, indices1F, indices2F, (0.,1.,0.,1.))
        self.marker_pub.publish(self.links)
        return indices1F, indices2F 
        
    def get_subset(self, position_i, quat_i_to_w, quat_w_to_i):
        """ Returns the subset of the accumulated points that is infront of the
        position of the drone"""
        # Convert accumulated points into local camera frame
        camera_relative = np.add(tf.transformations.quaternion_matrix(quat_w_to_i)[:3,:3].dot(self.mean), -position_i)
        # Mask for points infront (z>0)
        mask = camera_relative[2] > 0
        desc_masked = self.desc[mask, :]
        kp_masked = self.kp[mask, :]        
        # Invert mask so high for skipped values
        shifts = np.invert(mask)
        # Cumulative sum inverted mask to get shifts
        shifts = np.cumsum(np.array(shifts, dtype=np.int16))
        # Filter out non-skipped shifts
        shifts = shifts[mask]       
        """ shifts is such that the index in the full (self.mean) = index in the 
        fov (pts_masked) + shifts[index in fov]""" 
        return kp_masked, desc_masked, shifts
        
    def get_subset_fov(self, position_i, quat_i_to_w, quat_w_to_i):
        """ Returns the subset of the accumulated poitns that is in the fov of
        the position of the drone"""
        # Convert accumulated points into local camera frame
        camera_relative = np.add(tf.transformations.quaternion_matrix(quat_w_to_i)[:3,:3].dot(self.mean), -position_i)
        mask_infront = camera_relative[2,:] > 0.
        camera_relative = camera_relative/camera_relative[2]
        # Pre-calculate bounds in psuedo-homogenous frame (z scaled)
        bottom_left =  self.inverseCameraMatrix.dot(np.array([[0.],[360.],[ 1.]]))
        top_right =  self.inverseCameraMatrix.dot(np.array([[640.],[0.],[1.]]))        
        if True: # Publish fov region wireframe
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
            spoly.header.frame_id = '/ardrone_corrected_cam'
            spoly.header.stamp = self.stamp
            self.fov_pub.publish(spoly)
        # Mask by fov
        mask_left = camera_relative[0,:] > bottom_left[0]
        mask_bottom = camera_relative[1,:] < bottom_left[1]
        mask_right = camera_relative[0,:] < top_right[0]
        mask_top = camera_relative[1,:] > top_right[1]
        mask_hori = np.logical_and(mask_left, mask_right)
        mask_vert = np.logical_and(mask_top, mask_bottom)
        mask = np.logical_and(mask_hori, mask_vert)
        mask = np.logical_and(mask, mask_infront)
        desc_masked = self.desc[mask, :]
        kp_masked = self.kp[mask, :]
        pts_masked = self.mean[:,mask]
        # Invert mask so high for skipped values
        shifts = np.invert(mask)
        # Cumulative sum inverted mask to get shifts
        shifts = np.cumsum(np.array(shifts, dtype=np.int16))
        # Filter out non-skipped shifts
        shifts = shifts[mask]
        """ shifts is such that the index in the full (self.mean) = index in the 
        fov (pts_masked) + shifts[index in fov]"""
        return pts_masked, kp_masked, desc_masked, shifts
                
    def publish_cloud_accumulated(self): 
        """ Publish the accumulated cloud both as a PointCloud and a described
        point cloud"""
        cloud = PointCloud()
        cloud.header.stamp = self.stamp
        cloud.header.frame_id = "/world"
        
        # Build absolute cloud
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
        
    def publish_cloud(self, pts, publisher, frame_id="/world"):
        """ Publish the passed pts as a cloud on publisher """
        cloud = PointCloud()
        cloud.header.stamp = self.stamp
        cloud.header.frame_id = frame_id
        
        # Build relative cloud
        for i in range(pts.shape[1]):
            cloud.points.append(Point32())
            cloud.points[-1].x = pts[0,i]
            cloud.points[-1].y = pts[1,i]
            cloud.points[-1].z = pts[2,i]
        publisher.publish(cloud)
        
    def project_virtual_keypoints(self):
        """ Calculate virtual keypoints for a camera at the origin observing
        the accumulated cloud """
        homo_mean = np.vstack((self.mean, np.ones((1,self.mean.shape[1]))))
        kp = self.virtual_projection.dot(homo_mean)
        kp = (kp/kp[2])[:2]
        return kp.T
        
    @staticmethod
    def approx_register_3D(pts1, pts2):
        """ Calculate 3D rigid body transformation between paired point lists
        pts1 and pts2. As height is known absolutely, this is undesirable as 
        may change height"""
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
        
    @staticmethod
    def approx_register_3D_t( pts1, pts2): # pts1 = new points, pts2 = target frame points
        """ Calculate 3D translation between paired point lists pts1 and pts2 """
        # De-couple translation by subtracting centrioids
        delta = np.add(-pts1, pts2)
        t = np.array([np.mean(delta, axis=1)]).T
        R = np.diag((1.,1.,1.))
        return R, t
        
    @staticmethod
    def approx_register_2D(pts1, pts2): # pts1 = new points, pts2 = target frame points
        """ Calculate 2D rigid body transformation between paired point lists
        pts1 and pts2 in the x-y plane (think plan view). This effectively
        assumed z in already correct """        
        # De-couple translation by subtracting centrioids
        mean1 = np.array([np.mean(pts1, axis=1)]).T
        mean2 = np.array([np.mean(pts2, axis=1)]).T
        pts1_norm = np.add(pts1, -mean1)[:2]
        pts2_norm = np.add(pts2, -mean2)[:2]
        # Get covariance between point sets
        H = pts1_norm.dot(pts2_norm.T)
        # Recover least squares R
        U,S,Vt = np.linalg.svd(H)
        det = np.linalg.det(Vt.T.dot(U.T))
        R2 = Vt.T.dot(np.diag((1.,det)).dot(U.T))
        R = np.diag((1.,1.,1.))
        R[:2,:2] = R2
        # Recover t that match least squares R
        t = np.zeros((3,1))
        t[:2,:] = (mean2[:2,:] - R[:2,:2].dot(mean1[:2,:]))
        """
        # Safety Check
        """
        # Reject transform if R too great (>45deg)
        # Reject transform if t too great (>3m)
        # These are incremental so should remain small
        theta = np.abs(np.arccos(R[0,0]))
        if theta > 45*np.pi/180. :#or np.sum(np.square(t)) > 3.) and False:
            print "Safety Triggered"
            R = np.diag((1.,1.,1.))
            t = np.zeros((3,1))
        return R, t
        
    @staticmethod
    def approx_register_2D_t(pts1, pts2):
        # Pts are 3D world co-ords
        mean1 = np.array([np.mean(pts1, axis=1)]).T
        mean2 = np.array([np.mean(pts2, axis=1)]).T
        t = np.zeros((3,1))
        t[:2] = (mean2 - mean1)[:2]
        R = np.diag((1.,1.,1.))
        
        return R, t
    
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
        
    @staticmethod
    def extract_fundamental_and_filter_inliers(i1_pts_undistorted, i2_pts_undistorted, indices1, indices2):
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
