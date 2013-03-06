#!/usr/bin/env python

#================================= Scan =======================================
# 
# This script carries out a single 'scan' proceedure.
#
#==============================================================================
#
# Triangulation in arbitrary flight was found to be infeasible (at least as 
# implemented). 
#
# This attempts to take advantage to stable well defined drone aspects
#
# The drone is capable of determining absolute elevation using the ultrasound
# and absolute orientation using the magnetometer
# 
# Additionally the drone is capable of two smooth movements:
#   It can ascend and descent levelly
#   It can yaw levelly
# Lateral motion tilts the drone and is less predictable
#
# The two absolute measurements and reliable motions happend to coincide and 
# are exploited by this
#
#==============================================================================
#
# A 'scan' is as follows:
#   The drone stops and holds position to remove intertia
# 1)The drone records an image (frame1)
#   The drone ascends to an increased elevation
#   The drone records an image (frame2)
#   Matches are triangulated from frame1 to frame2 using known elevation change
#   The drone yaws 90deg
# 2)The drone records an image (frame1)
#   The drone descends to the original elevation
#   The drone records an image (frame2)
#   Matches are triangulated from frame1 to frame2 using known elevation change
#   The drone yaws 90deg
# 3)Stages 1 and 2 are repeated, leaving the drone in its original position
#
#   Good triangulation should have been carried out in all cardinal directions
#
#==============================================================================

import roslib; roslib.load_manifest('scan')
import rospy
import numpy as np
import geometry_msgs.msg as gm
import tf
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty
from std_msgs.msg import Header
from custom_msgs.msg import Described3DPoints
from custom_msgs.msg import StampedFeaturesMatches
from custom_msgs.msg import Visualisation
import math
import time
from copy import deepcopy

USE_OPTIMAL_CORRECTION = False
REPRO_ERROR_SQUARED = 100.
OPT_ERROR_SQUARED = 256.

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
        
            # Get projected pixel co-ords
            projected_pixels_homo = P1.dot(V[-1,:])
            projected_pixels = (projected_pixels_homo/projected_pixels_homo[2])[:2]
            # Get dif between proj and image
            error = projected_pixels-x1[:2]
            error_mag1 = error[0]*error[0]+ error[1]*error[1]
            
            # Same for frame 2
            projected_pixels_homo = P2.dot(V[-1,:])
            projected_pixels = (projected_pixels_homo/projected_pixels_homo[2])[:2]
            error = projected_pixels-x2[:2]
            error_mag2 = error[0]*error[0]+ error[1]*error[1]
            
            # max is more useful than average as we want a good fit to both
            error_max = max(error_mag1, error_mag2)        
            
            return np.hstack((X / X[3], error_max))

class PointTriangulator:
    def __init__(self):
        self.connect()
        
        self.rejection_reasons = np.array([0,0,0,0,0,0,0])
    
    def connect(self):     
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        self.tf = tf.TransformListener()
        rospy.Subscriber('/projection_calc/projections',StampedFeaturesMatches,self.on_got_matches)
        self.cloud_pub = rospy.Publisher('/scan/absolute_cloud', PointCloud)
        self.cloud_pub2 = rospy.Publisher('/scan/relative_cloud', PointCloud)
        self.desc_cloud_pub = rospy.Publisher('/scan/relative_described_cloud', Described3DPoints)
        self.vis_pub = rospy.Publisher('/scan/visualisation', Visualisation)

    def on_got_matches(self, sfm):
        """Takes a cv2 numpy array image and compared to a previously
        buffered image. Features are extracted from each frame, 
        undistorted and matched. The matches are then triangulated"""
        self.time_prev = time.time()
        
        self.descriptors_matcher = sfm.descriptors_matcher
        
        F, pts1, pts2, self.desc1, self.desc2 = self.decode_message(sfm)
        
        """====================================================================
        Rotation and translation from navdata
        ===================================================================="""
        self.get_change_in_tf(sfm.header1, sfm.header2)
        print sfm.header1.stamp, ", ", sfm.header2.stamp
        """====================================================================
        3D Triangulate matched pairs using navdata
        ===================================================================="""
        self.tf_triangulate_points(pts1, pts2, sfm)
        
        """====================================================================
        Pass visualiser data to visualiser node
        ===================================================================="""
        self.publish_visualisation(pts1, pts2, sfm)
        
        print self.rejection_reasons
        
        print "Point Cloud Triangulated ( ", np.around(((time.time()-self.time_prev)*1000),1), "ms) "
        
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
        image_P1 = np.reshape(np.array(sfm.P1, np.float32), (3, 4))
        image_P2 = np.reshape(np.array(sfm.P2, np.float32), (3, 4))
        print "P1:\r\n", image_P1
        print "P2:\r\n", image_P2
        P2_homo = np.diag((1.,1.,1.,1.))
        P2_homo[:3,:4] = image_P2
        self.image_P2 = np.linalg.inv(P2_homo)[:3,:4]
        print "rot: \r\n", np.array((tf.transformations.euler_from_matrix(self.image_P2[:3,:3])))*180./np.pi
        return F, kp1, kp2, desc1, desc2
    
    def publish_cloud(self, points, timestamp):
        """ Builds and publishes absolute and relative point clouds"""
        
        """====================================================================
        # Absolute Point Cloud
        ===================================================================="""
        cloud = PointCloud()
        cloud.header.stamp = timestamp
        cloud.header.frame_id = "/world"
        
        
        #print "Pre-shift points:\r\n ", points
        sub = np.add(points.T, self.position_i2).T
        sub = tf.transformations.quaternion_matrix(self.quat_i_to_w2)[:3,:3].dot(sub)
        #print "Post-shift points:\r\n ", sub
        
        # Reshape for easy clouding
        sub = zip(*np.vstack((sub[0], sub[1], sub[2])))

        # Build absolute cloud
        for i, p in enumerate(sub):
            cloud.points.append(gm.Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        self.cloud_pub.publish(cloud)
        
        """====================================================================
        # Relative Point Cloud
        ===================================================================="""        
        cloud = PointCloud()
        cloud.header.stamp = timestamp
        cloud.header.frame_id = "/ardrone_base_frontcam"
        # Reshape for easy clouding
        sub = zip(*np.vstack((points[0], points[1], points[2])))
        
        # Build relative cloud
        for i, p in enumerate(sub):
            cloud.points.append(gm.Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        self.cloud_pub2.publish(cloud) 
        
        print "Cloud  of ", len(cloud.points), "Published"
        
        """====================================================================
        # Relative Point Cloud with keypoints and descriptors
        ===================================================================="""
        described_cloud = Described3DPoints()
        described_cloud.header = cloud.header
        described_cloud.points3D = points.reshape(-1,).tolist()
        described_cloud.points2D = self.kp_triangulated.reshape(-1,).tolist()
        described_cloud.descriptors = self.desc_triangulated.reshape(-1,).tolist()
        described_cloud.descriptors_stride = self.desc_triangulated.shape[1]
        described_cloud.descriptors_matcher = self.descriptors_matcher
        described_cloud.position_i = self.position_i2
        described_cloud.quat_w_to_i = self.quat_w_to_i2
        described_cloud.quat_i_to_w = self.quat_i_to_w2
        self.desc_cloud_pub.publish(described_cloud)
        
    def publish_visualisation(self, pts1, pts2, sfm):
        vis = Visualisation()      
        vis.header1 = sfm.header1
        vis.header2 = sfm.header2
        vis.points1 = pts1.reshape(-1,).tolist()
        vis.points2 = pts2.reshape(-1,).tolist()
        vis.reprojected1 = []
        for x in self.reprojected_frame1:
            vis.reprojected1.append(x[0])
            vis.reprojected1.append(x[1])
        vis.reprojected2 = []
        for x in self.reprojected_frame2:
            vis.reprojected2.append(x[0])
            vis.reprojected2.append(x[1])
        self.vis_pub.publish(vis)
        
    def world_to_pixel_distorted(self, pts, R, t, K=None, k=None):
        """Takes 3D world co-ord and reverse projects using K and distCoeffs"""
        
        if k == None:
            k = self.distCoeffs
        #print "k : \r\n", k
        k1 = k[0][0] # Radial coeff 1
        k2 = k[0][1] # Radial coeff 2
        p1 = k[0][2] # Tangential coeff 1
        p2 = k[0][3] # Tangential coeff 2
        
            
        if K == None:
            K = self.cameraMatrix
        fx = K[0,0]
        fy = K[1,1]
        cx = K[0,2]                    
        cy = K[1,2]
        
        #print "k : \r\n", len(k)
        #print "pts : \r\n", pts
        
        # Set up projection from R and t                   
        P =  np.diag([1.,1.,1.,1.])
        Pinner = np.hstack((R, t))
        P[:3, :4] = Pinner
        
        # Resolve to world Frame
        sub = P.dot(pts)[:3]
        
        # First order
        x_dash = sub[0]/sub[2]
        #print "x_dash : \r\n", x_dash
        y_dash = sub[1]/sub[2]
        
        # Precalc terms (Note: This is significantly simpler than if we had all 8 distCoeffs)
        r_2 = x_dash*x_dash + y_dash*y_dash
        r_4 = r_2*r_2
        terms = (1+k1*r_2+k2*r_4)
        
        # Second Order
        x_dash2 = x_dash*terms+2*p1*x_dash*y_dash+p2*(r_2+2*x_dash*x_dash)
        y_dash2 = y_dash*terms+2*p2*x_dash*y_dash+p1*(r_2+2*y_dash*y_dash)
        
        # To pixel
        u = fx*x_dash2+cx
        v = fy*y_dash2+cy
        
        return np.array([u,v]).T
            
    def load_tf(self, header):
        """Loads pose for specified header"""
        """====================================================================
        # World co-ordinate handling (note -position_w != position_i)
        ===================================================================="""
        self.tf.waitForTransform("/world", header.frame_id, header.stamp, rospy.Duration(4))
        # Get tf lookup in reverse frame, this ensures translation is in world axis
        position_w, q1 = self.tf.lookupTransform( "/world", header.frame_id, header.stamp)
        position_w = np.array((position_w))
        # Flip quat to origin-to-drone-image
        quaternion_i_to_w = q1
        
        """====================================================================
        # Image co-ordinate handling
        ===================================================================="""
        
        position_i, qi = self.tf.lookupTransform(header.frame_id, "/world", header.stamp)
        position_i = -np.array((position_i))
        quaternion_w_to_i = qi
        
        return position_w, position_i, quaternion_i_to_w, quaternion_w_to_i

    def get_change_in_tf(self, header1, header2):       
        self.position_w1, self.position_i1, self.quat_i_to_w1, self.quat_w_to_i1 = self.load_tf(header1)
        self.position_w2, self.position_i2, self.quat_i_to_w2, self.quat_w_to_i2 = self.load_tf(header2)
                
        # Get relative quaternion
        # qmid = qafter.qbefore-1
        self.relative_quat = tf.transformations.quaternion_inverse(tf.transformations.quaternion_multiply(self.quat_w_to_i2, tf.transformations.quaternion_inverse(self.quat_w_to_i1)))
        
        # Difference in position in world co-ordinates
        trans = np.array(([(self.position_w2[0] - self.position_w1[0])],
                          [(self.position_w2[1] - self.position_w1[1])],
                          [(self.position_w2[2] - self.position_w1[2])]))
        # Rotate into frame1 co-ordinates
        R = tf.transformations.quaternion_matrix(self.quat_w_to_i1)[:3, :3]
        #print trans
        trans = R.dot(trans)
        
        self.image_coord_trans = np.array([trans[0], trans[1], trans[2]])
   
    def tf_triangulate_points(self, pts1, pts2, sfm):
        time_start = time.time()
        """ Triangulates 3D points from set of matches co-ords using relative
        camera position determined from tf"""
        # Working in image co-ordinates throughout
        
        # These are arrays of the triangulated points reprojected back to the
        # image plane
        self.reprojected_frame1 = []
        self.reprojected_frame2 = []
        
        # Get t from tf data
        # Note R and t are inverted as they were calculated from frame1 to frame2
        # in frame2 co-ordinates and we are treating frame2 as the origin
        t = self.image_coord_trans
        t2 = self.image_P2[:,3:4]
        print "trans t: ", t.T
        mag = np.sqrt(t[0]*t[0]+t[1]*t[1]+t[2]*t[2])
        img_mag = np.sqrt(t2[0]*t2[0]+t2[1]*t2[1]+t2[2]*t2[2])
        scaled_t2 = mag*t2/img_mag
        print "trans i: ", scaled_t2.T
        
        # Get rotation matrix
        R = tf.transformations.quaternion_matrix(self.relative_quat)[:3, :3]
        print "angles t: ", 180.*np.array((tf.transformations.euler_from_matrix(R)))/np.pi
        R2 = self.image_P2[:3,:3]
        print "angles i: ", 180.*np.array((tf.transformations.euler_from_matrix(R2)))/np.pi
        
                
        # Bottom out if motion is insufficient
        mag = np.sqrt(t[0]*t[0]+t[1]*t[1])
        if mag < 0.13:
            print "Motion bad for triangulation :\r\n", str(t)
            self.rejection_reasons[0] = self.rejection_reasons[0]+1
            return None
            
        # Print pre-triangulated count
        pre_triangulated = len(pts1)+0.
        print "Pre-triangulated: ", pre_triangulated
            
        # Compose dead reckoned projection matrix
        P_cam1_to_cam2 = np.hstack((R, t))    
        print "Dead P:\r\n", P_cam1_to_cam2
        
        """
        Triangulate using pixel co-ord and K[R t]
        """
        # Factor in camera calibration
        PP2 = np.hstack((self.cameraMatrix, np.array([[0.],[0.],[0,]])))
        PP1 = self.cameraMatrix.dot(P_cam1_to_cam2)
        
        # Triangulate and reject based on error settings
        points3D_image = self.triangulate_points(pts1.transpose(), pts2.transpose(), PP1, PP2)
        if points3D_image == None or len(points3D_image) == 0:
            return            
        points3D_image = points3D_image[:3]        
        
        # Filter points that are behind the camera
        infront = points3D_image[2] > 0
        infront = np.array([infront]).transpose()
        points3D_image = np.reshape(points3D_image[np.hstack((infront, infront, infront)).T==True], (3, -1))
        # Filter descriptors
        self.desc_triangulated = np.reshape(self.desc_accepted[np.resize(infront.T, (self.desc_accepted.shape[1], self.desc_accepted.shape[0])).T==True], (-1, self.desc_accepted.shape[1]))
        self.kp_triangulated = np.reshape(self.kp_accepted[np.resize(infront.T,(self.kp_accepted.shape[1], self.kp_accepted.shape[0])).T==True], (self.kp_accepted.shape[0], -1))
        if points3D_image == None or len(points3D_image) == 0:
            print "No points infront of camera"
            self.rejection_reasons[3] = self.rejection_reasons[3]+1
            return
        
        # Output number of forward triangulated points
        forward_triangulated = len(points3D_image[0])+0.
        print "Forward triangulated: ", forward_triangulated    
        
        '''
        sq = np.square(points3D_image)
        sq = np.sqrt(sq[0]+sq[1]+sq[2])
        sq_sum = np.sum(sq)
        avg = sq_sum/points3D_image.shape[1]
        if avg > 5.:
            print "Points average too far"
            self.rejection_reasons[4] = self.rejection_reasons[4]+1
            return
        
        reasonable = sq < 4.
        reasonable = np.array([reasonable]).transpose()
        points3D_image = np.reshape(points3D_image[np.hstack((reasonable, reasonable, reasonable)).T==True], (3, -1))
        self.desc_triangulated = np.reshape(self.desc_triangulated[np.resize(reasonable.T, (self.desc_triangulated.shape[1], self.desc_triangulated.shape[0])).T==True], (-1, self.desc_triangulated.shape[1]))
        self.kp_triangulated = np.reshape(self.kp_triangulated[np.resize(reasonable.T,(self.kp_triangulated.shape[1], self.kp_triangulated.shape[0])).T==True], (self.kp_triangulated.shape[0], -1))
        if points3D_image == None or len(points3D_image) == 0:
            print "Points unreasonable"
            self.rejection_reasons[5] = self.rejection_reasons[5]+1
            return
        
        
        # Filter points that are too far in front
        infront = points3D_image[2] < 5.        
        infront = np.array([infront]).transpose()
        points3D_image = np.reshape(points3D_image[np.hstack((infront, infront, infront)).T==True], (3, -1))
        # Filter descriptors
        self.desc_triangulated = np.reshape(self.desc_triangulated[np.resize(infront.T, (self.desc_triangulated.shape[1], self.desc_triangulated.shape[0])).T==True], (-1, self.desc_triangulated.shape[1]))
        self.kp_triangulated = np.reshape(self.kp_triangulated[np.resize(infront.T,(self.kp_triangulated.shape[1], self.kp_triangulated.shape[0])).T==True], (self.kp_triangulated.shape[0], -1))
        if points3D_image == None or len(points3D_image) == 0:
            print "All Points too far"
            self.rejection_reasons[6] = self.rejection_reasons[0]+1
            return
        '''
        
        # Triangulated points reprojected back to the image plane
        self.reprojected_frame1 = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, R, t)        
        self.reprojected_frame2 = self.world_to_pixel_distorted(self.make_homo(points3D_image.T).T, np.diag((1.,1.,1.)), np.array([[0.,0.,0.]]).T)
        
        # Publish Cloud
        # Note: img2 camera is taken to be the origin, so time2 is used
        if forward_triangulated != 0 and (float(forward_triangulated)/pre_triangulated > 0.0):
            print "Publishing Point cloud---------------------------------------------------------"
            self.publish_cloud(points3D_image, sfm.header2.stamp)
        else:
            print "Poor triangulation"
            return
        print "Triangulation took: ", time.time() - time_start
            
    def find_fundamental_from_proj(self, P1, P2, f0 = 640.):
        F = np.diag((0.,0.,0.))
        W = np.diag((0.,0.,0., 0.))
        
        Pk = np.diag((1., 1., f0))
        P1 = Pk.dot(P1)
        P2 = Pk.dot(P2)
        
        W[0] = P1[1]
        W[1] = P1[2]
        W[2] = P2[1]
        W[3] = P2[2]
        F[0,0] = np.linalg.det(W)
        
        W[0] = P1[1]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[2]
        F[0,1] = -np.linalg.det(W)
        
        W[0] = P1[1]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[1]
        F[0,2] = np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[2]
        W[2] = P2[1]
        W[3] = P2[2]
        F[1,0] = -np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[2]
        F[1,1] = np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[2]
        W[2] = P2[0]
        W[3] = P2[1]
        F[1,2] = -np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[1]
        W[2] = P2[1]
        W[3] = P2[2]
        F[2,0] = np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[1]
        W[2] = P2[0]
        W[3] = P2[2]
        F[2,1] = -np.linalg.det(W)
        
        W[0] = P1[0]
        W[1] = P1[1]
        W[2] = P2[0]
        W[3] = P2[1]
        F[2,2] = np.linalg.det(W)
        
        
        # Normalise
        norm = np.sqrt(F[0,0]*F[0,0]+F[0,1]*F[0,1]+F[0,2]*F[0,2]+F[1,0]*F[1,0]+F[1,1]*F[1,1]+F[1,2]*F[1,2]+F[2,0]*F[2,0]+F[2,1]*F[2,1]+F[2,2]*F[2,2])
        F = F/norm
        
        
        return F
       
    def optimal_correction_triangulate_point(self, xvector1, xvector2, F, f0 = 640.):
        """This carries out the Optimal Correction Method outlined by Kanatami 
        et al in 'Triangulation from Two Views Revisited: Hartley-Sturm vs. 
        Optimal Correction'. This particular iterative method is significantly
        faster than Hartley-Sturm's and is stable in all cases.
        
        This method shifts the pixel locations the minimal distance such that 
        they both satisfy the epipolar constraint. This has the implicit effect
        of causing the triangulation to intersect exactly
        
        x1 and x2 are pixel co-ord homogenous matched points
        F is the fundamental
        f0 is a scale factor for numerical stability and ~ image width (pixels)
        """
        
        # NOTE: The x1, x2 vectors are different to those in the paper.
        # The paper uses x_vector = [ x/f0 ]   This uses:  [ x ]
        #                           [ y/f0 ]               [ y ]
        #                           [ 1    ]               [ 1 ]
        # This simply means some scalings f0 scaling are skipped
        
        
        # Flatten fundamental
        u = np.reshape(F, (1,-1))[0]
        
        
        # Precalc terms that do not change in iteration
        f0f0 = f0 * f0
        f10 = u[2]*f0
        f11 = u[5]*f0
        f20 = u[6]*f0
        f21 = u[7]*f0        
        uu = np.array([u[0]*u[0], u[1]*u[1], u[2]*u[2], u[3]*u[3], u[4]*u[4], u[5]*u[5], u[6]*u[6], u[7]*u[7]])
        u0u1 = u[0]*u[1]
        u0u2 = u[0]*u[2]
        u0u3 = u[0]*u[3]
        u0u6 = u[0]*u[6]
        u1u2 = u[1]*u[2]
        u1u4 = u[1]*u[4]
        u1u7 = u[1]*u[7]
        u3u4 = u[3]*u[4]
        u3u5 = u[3]*u[5]
        u3u6 = u[3]*u[6]
        u4u5 = u[4]*u[5]
        u4u7 = u[4]*u[7]
        
        # P is a vector of elementwise products of Xihat and u
        # Xihat is never individually calculated
        P8 = f0f0*u[8]
        
        # Q is the sum of constant diagonal terms in V0[Xi]
        Qconst = f0f0*(uu[2] + uu[5] + uu[6] + uu[7])
        
        
        
        # Convergence target.
        # ie. minimal error reduction considered to justify another iteration
        # Should be multiplied by f0f0 wrt actual desired limit
        convergence = 1.0e-8
        
        # Initialise values
        # E0 is initialised to ~ infinity
        # Should be multiplied by f0f0 wrt actual nominal E0
        E0 = 1.0e+10
        xhat1 = xvector1[0]
        yhat1 = xvector1[1]
        xhat2 = xvector2[0]
        yhat2 = xvector2[1]
        # Original x values are saved in this form to make the code cleaner
        ox1 = xvector1[0]
        oy1 = xvector1[1]
        ox2 = xvector2[0]
        oy2 = xvector2[1]
        xtwiddle1 = 0.
        ytwiddle1 = 0.
        xtwiddle2 = 0.
        ytwiddle2 = 0.
        
        done = False
        iterations = 0
        while(not done):
            iterations = iterations + 1
            # Compute P -> a vector of elementwise products of Xihat and u
            P = np.array([(xhat1*xhat2 + xhat2*xtwiddle1 + xhat1*xtwiddle2)*u[0],
                          (xhat1*yhat2 + yhat2*xtwiddle1 + xhat1*ytwiddle2)*u[1],
                          f0*(xhat1+xtwiddle1)*u[2],
                          (yhat1*xhat2 + xhat2*ytwiddle1 + yhat1*xtwiddle2)*u[3],
                          (yhat1*yhat2 + yhat2*ytwiddle1 + yhat1*ytwiddle2)*u[4],
                          f0*(yhat1+ytwiddle1)*u[5],
                          f0*(xhat2+xtwiddle2)*u[6],
                          f0*(yhat2+ytwiddle2)*u[7],
                          P8])
                             
            # Pre-calc intermediate terms
            xx1 = xhat1*xhat1
            xx2 = xhat2*xhat2
            yy1 = yhat1*yhat1
            yy2 = yhat2*yhat2
            xy1 = xhat1*yhat1
            xy2 = xhat2*yhat2
            f0x1 = f0*xhat1
            f0x2 = f0*xhat2
            f0y1 = f0*yhat1
            f0y2 = f0*yhat2
            
            # Calculate results of diagonal terms of (u, V0[Xi]u)
            diag = np.array([(xx1+xx2)*uu[0], (xx1+yy2)*uu[1], (yy1+xx2)*uu[3], (yy1+yy2)*uu[4], Qconst])
            # Calculate results of unique off-diagonal terms of (u, V0[Xi]u)
            # Matrix is symmetric so each value actually occurs twice
            cross = np.array([xy2*u0u1, f0x2*u0u2, xy1*u0u3, f0x1*u0u6,
                              f0y2*u1u2, xy1*u1u4, f0x1*u1u7,
                              xy2*u3u4, f0x2*u3u5, f0y1*u3u6,
                              f0y2*u4u5, f0y1*u4u7])
            
            # Evaluate (u, Xihat)
            uxi = np.sum(P)
            # Evaluate (u, V0[Xi]u)
            # Non-diagonal terms each occur twice
            uxiu = np.sum(diag) + 2.0*np.sum(cross)
            #      (u, Xihat)
            # C = ------------
            #     (u, V0[Xi]u)
            C = uxi/uxiu
            
            # Update twiddle values
            #
            # [ xtwiddle1 ] = C[ u0 u1 u2 ][ xhat2 ]
            # [ ytwiddle1 ]    [ u3 u4 u5 ][ yhat2 ]
            #                              [ f0    ]
            #
            # [ xtwiddle2 ] = C[ u0 u3 u6 ][ xhat1 ]
            # [ ytwiddle2 ]    [ u2 u5 u8 ][ yhat1 ]
            #                              [ f0    ]            
            xtwiddle1 = C*(u[0]*xhat2 + u[1]*yhat2+f10)
            ytwiddle1 = C*(u[3]*xhat2 + u[4]*yhat2+f11)
            xtwiddle2 = C*(u[0]*xhat1 + u[3]*yhat1+f20)
            ytwiddle2 = C*(u[1]*xhat1 + u[4]*yhat1+f21)
            
            # Evaluate reprojection error
            # This should technically be divided by f0f0 but this is avoided as
            # E is f0f0E0typical and convergence is f0f0convergencetypical
            E = xtwiddle1*xtwiddle1 + ytwiddle1*ytwiddle1 + xtwiddle2*xtwiddle2 + ytwiddle2*ytwiddle2
            
            # Check if iteration reduced error
            if abs(E-E0) > convergence:
                # Update error bound
                E0 = deepcopy(E)
                # Update difference terms
                xhat1 = ox1 - xtwiddle1
                yhat1 = oy1 - ytwiddle1
                xhat2 = ox2 - xtwiddle2
                yhat2 = oy2 - ytwiddle2
            else:
                # No benefit to further iteration
                done = True
                
        new_x1 = np.array((xhat1, yhat1, 1.0))
        new_x2 = np.array((xhat2, yhat2, 1.0))
        
        return np.hstack((new_x1, new_x2, E))
    
    def triangulate_points(self, x1,x2,P1,P2):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""        
        
        n = x1.shape[1]
        # Make homogenous
        x1_homo = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2_homo = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        
        if (USE_OPTIMAL_CORRECTION):
            print "bing"
            F = self.find_fundamental_from_proj(P1, P2)
            # Correct points
            corr = np.array([self.optimal_correction_triangulate_point(x1_homo[:,i],x2_homo[:,i], F) for i in range(n)])
            corr1 = corr[:, :3].T
            corr2 = corr[:, 3:6].T
            errors = corr[:, 6:7]
            
            # Create mask based on amount of shift required
            shift = x1_homo - corr1
            shift = shift*shift
            shift1 = shift[0]+shift[1]
            shift = x2_homo - corr2
            shift = shift*shift
            shift2 = shift[0]+shift[1]
            accepted = np.logical_and(shift1<OPT_ERROR_SQUARED, shift2<OPT_ERROR_SQUARED)
            
            # Filter points with too much implied shift
            accepted = np.array([accepted]).T
            pts_mask = np.hstack((accepted, accepted, accepted)).T
            x1_homo = np.reshape(x1_homo[pts_mask==True], (3, -1))
            x2_homo = np.reshape(x2_homo[pts_mask==True], (3, -1))
            
            n = len(x1_homo.T)
            print x1_homo.shape
            if x1_homo == None or n <= 1:
                print "No points correctable"
                self.rejection_reasons[1] = self.rejection_reasons[1]+1
                return None
            
            # Filter descriptors
            self.desc_accepted = np.reshape(self.desc1[np.resize(accepted.T, (self.desc1.shape[1],self.desc1.shape[0])).T==True], (-1, self.desc1.shape[1]))
            
            
                
            self.corr_triangulated = n+0.
            print "Corr-triangulated: ", self.corr_triangulated
        else:
            self.desc_accepted = self.desc1
        
        # Triangulate for each pair
        triangulator = Triangulator(P1, P2)
        Combi = np.array([triangulator.triangulate(x1_homo[:,i],x2_homo[:,i], P1, P2) for i in range(n)]) # Looping here is probably unavoidable
        # Extract 4D points
        X = Combi[:,:4]        
        # Create mask
        accepted = Combi[:, 4] < REPRO_ERROR_SQUARED
        
        # Filter points with too low accuracy (reprojected to actual image)
        accepted = np.array([accepted]).T
        X = np.reshape(X.T[np.hstack((accepted, accepted, accepted)).T==True], (3, -1))
        # Filter descriptors and keypoints
        self.desc_accepted = np.reshape(self.desc_accepted[np.resize(accepted.T, (self.desc_accepted.shape[1],self.desc_accepted.shape[0])).T==True], (-1, self.desc_accepted.shape[1]))
        self.kp_accepted = np.reshape(x1_homo[np.resize(accepted.T, (x1_homo.shape[1],x1_homo.shape[0])).T==True], (x1_homo.shape[0], -1))[:2]
        
        if X == None or len(X[0]) == 0:
            print "No points triangulatable"
            self.rejection_reasons[2] = self.rejection_reasons[2]+1
            return None
        
        self.triangulated = len(X[0])+0.
        print "Triangulated: ", self.triangulated
        
        return X
        
    def make_homo(self, pts):
        pts = np.append(pts,np.array([np.ones(pts.shape[0])]).T, 1)
        return pts
        
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
        self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
        self.distCoeffs = np.array([ci.D], dtype=np.float32)
        self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
        print "                    Calibration Initialised\r\n"


def run():
    rospy.init_node('Point_Triangulator')
    
    # Get parameters
    global USE_OPTIMAL_CORRECTION
    USE_OPTIMAL_CORRECTION = rospy.get_param('~opt', False)
    global OPT_ERROR_SQUARED
    OPT_ERROR_SQUARED = rospy.get_param('~shift', 16.)
    global REPRO_ERROR_SQUARED
    REPRO_ERROR_SQUARED = rospy.get_param('~error', 10.)
    
    # Print startup info
    print "\r\n"
    print "===================== Point Triangulator =========================="
    print " Allowing ", REPRO_ERROR_SQUARED, "reprojection error - Set with _error"
    print " Optimal Correction : ", USE_OPTIMAL_CORRECTION, " - Set with _opt"
    if USE_OPTIMAL_CORRECTION:
        print " Allowing optimal correction of ", OPT_ERROR_SQUARED, " - Set with _shift"
    print "==================================================================="
    print "\r\n"
    
    OPT_ERROR_SQUARED = float(OPT_ERROR_SQUARED*OPT_ERROR_SQUARED)
    REPRO_ERROR_SQUARED = float(REPRO_ERROR_SQUARED*REPRO_ERROR_SQUARED)
    
    # Initialise controller
    pt = PointTriangulator()
    
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
