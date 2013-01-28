#!/usr/bin/env python

#================================= Scan (new model)============================
# 
# This script handles triangulation under the new model. Feature matching is
# handled externally and this script runs blindly when it receives new data
# 
# Temporary controls have been added to allow use before hive mind
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
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2
import cv
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg as gm
import tf
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
import std_msgs.msg

from custom_msgs.msg import StampedFrames


def make_homo(pts):
    pts = np.append(pts,np.array([np.ones(pts.shape[0])]).T, 1)
    return pts
    
def stackImagesVertically(top_image, bottom_image):
    """Takes two cv2 numpy array images top_image, bottom_image
       Returns a cv2 numpy array image of the two stacked vertically
       Image dimensions need not match, output it left aligned"""
    # Get image dimensions
    h1, w1 = top_image.shape[:2]
    h2, w2 = bottom_image.shape[:2]
    # Create an empty array that is bounding box size of image stack
    stacked = np.zeros((h1+h2, max(w1,w2)), np.uint8)
    # Drop in the top_image
    stacked[:h1, :w1] = top_image
    # Drop in the bottom_image
    stacked[h1:h1+h2, :w2] = bottom_image
    return stacked

class Triangulator:
    def __init__(self):
        self.tf = tf.TransformListener()        
        self.calibrated = False
        self.connect()
        
    def connect(self):
        rospy.Subscriber('/feature_matcher/matches', StampedFrames ,self.process)
        rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, self.set_camera_info)
        self.cloud_pub = rospy.Publisher('/scan/absolute_cloud', PointCloud)
        self.cloud_pub2 = rospy.Publisher('/scan/relative_cloud', PointCloud)
        
    def extract_fundamental(self, i1_pts_undistorted, i2_pts_undistorted, i1_pts_draw, i2_pts_draw):
        """
        Extract fundamental matrix and then remove outliers
        FM_RANSAC should be good with lowish outliers
        FM_LMEDS may be more robust in some cases
        """
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 3, param2 = 0.99)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_masked = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        i1_pts_undistorted = np.array([i1_pts_masked])
        i2_pts_undistorted = np.array([i2_pts_masked])
        
        i1_pts_masked = np.reshape(i1_pts_draw[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_draw[mask_prepped==1], (-1, 2))
        i1_pts_draw_corr = np.array([i1_pts_masked])
        i2_pts_draw_corr = np.array([i2_pts_masked])
        
        """============================================================
        # Filter points that fit F using cv2.correctMatches
        # This unhelpfully overwrites np.nan over rejected entried
        # np.nan == np.nan returns false so have to use np.isnan(.)
        # NB: This check appears redundant as F is calculated to match
        ============================================================"""
                
        i1_pts_corr, i2_pts_corr = cv2.correctMatches(F, i1_pts_undistorted, i2_pts_undistorted)
        mask_nan = np.isnan(i1_pts_corr[0])
        i1_pts_corr = np.reshape(i1_pts_corr[0][mask_nan == False], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_corr[0][mask_nan == False], (-1, 2))
        i1_pts_draw_corr = np.reshape(i1_pts_draw_corr[0][mask_nan == False], (-1, 2))
        i2_pts_draw_corr = np.reshape(i2_pts_draw_corr[0][mask_nan == False], (-1, 2))
        
        return F, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr
        
    def extract_projections(self, F, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr):
        """
        Uses known camera calibration to extract E
        Produces 4 possible P2 via linear algebra
        Isolates best P2 by projecting data points
        Filters out conflicting points
        """
        
        # Camera Matrices to extract essential matrix and then normalise
        E = self.cameraMatrix.transpose().dot(F.dot(self.cameraMatrix))
        E /= E[2,2]
        #print "E", E
        
        W = np.array([[0, -1, 0],[1, 0, 0], [0, 0, 1]])
        Z = np.array([[0, 1, 0],[-1, 0, 0], [0, 0, 0]])
                    
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
        ===============================================================
        # A comparison between manually triangulated and cv2 tri found
        # different results. It turns out cv2 output un-normalised homo
        # co-ords (i.e. non-unity w)
        ============================================================"""  
        
        # First must normalise co-ords
        i1_pts_corr_norm = self.inverseCameraMatrix.dot(make_homo(i1_pts_corr).transpose()).transpose()[:,:2]
        i2_pts_corr_norm = self.inverseCameraMatrix.dot(make_homo(i2_pts_corr).transpose()).transpose()[:,:2]
                              
        ind = 0
        maxfit = 0
        secfit = 0
        for i, P2 in enumerate(projections):
            # infront accepts only both dimensions
            # WARNING: cv2.tri produces unnormalised homo coords
            points4D = cv2.triangulatePoints(P1, P2, i1_pts_corr_norm.transpose(), i2_pts_corr_norm.transpose())
            # normalise homogenous coords
            points4D /= points4D[3]
            #points4D = self.triangulate_points(i1_pts_corr.transpose(), i2_pts_corr.transpose(), P1, P2)
            d1 = np.dot(P1,points4D)[2]
            d2 = np.dot(P2,points4D)[2]
            PI = sum((d1>0) & (d2>0))
            #print "Support for P2 ", i, " : ", PI
            if PI > maxfit:
                secfit = maxfit
                maxfit = PI
                ind = i
                infront = (d1>0) & (d2>0)
        #if (maxfit < 4*secfit): maxfit ~= secfit is not actually a problem where translation is small, so cannot simply filter by it
        if maxfit < 4: # 8 Chosen as at least 8 points are needed to compute an effective fundamental matrix -> if we cannot satisfy at least 8 points we have serious issues
            print "==================================================="
            print "P2 not extracted"
            print "==================================================="
            return False, None, None, None, None, None, None            
        
        #print "P2"
        #print projections[ind]
        
        # Filter points
        infront = np.array([infront]).transpose()
        infront = np.append(infront, infront, 1)
        i1_pts_final = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_final = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        i1_pts_draw_final = np.reshape(i1_pts_draw_corr[infront==True], (-1, 2))
        i2_pts_draw_final = np.reshape(i2_pts_draw_corr[infront==True], (-1, 2))
        
        return True, P1, projections[ind], i1_pts_final, i2_pts_final, i1_pts_draw_final, i2_pts_draw_final
            
    def publish_cloud(self, points, timestamp):
        """ Builds and publishes absolute and relative point clouds"""
        
        """====================================================================
        # Absolute Point Cloud
        ===================================================================="""
        cloud = PointCloud()
        cloud.header.stamp = timestamp
        cloud.header.frame_id = "/world"
        
        sub = np.add(points.T, self.position_i1).T
        sub = tf.transformations.quaternion_matrix(self.quat_i_to_w_w1)[:3,:3].dot(sub)
        
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
    
    def get_change_in_tf(self, time1, time2):       
        
        """====================================================================
        # World co-ordinate handling (note -p1 != pi)
        ===================================================================="""
        self.tf.waitForTransform("world", "ardrone_base_frontcam", time2, rospy.Duration(4.))        
        # Get tf lookup in reverse frame, this ensures translation is in world axis
        p, q = self.tf.lookupTransform( "world", "ardrone_base_frontcam",time2)
        self.position_w2 = np.array((p))
        self.quat_i_to_w_w2 = tf.transformations.quaternion_inverse(q)
        
        #time2 is after time1 so no need to wait again
        # Get tf lookup in reverse frame, this ensures translation is in world axis
        p, q = self.tf.lookupTransform( "world", "ardrone_base_frontcam",time1)
        self.position_w1 = np.array((p))
        self.quat_i_to_w_w1 = tf.transformations.quaternion_inverse(q)
        
        print time1
        print time2
        
        """====================================================================
        # Image co-ordinate handling
        ===================================================================="""
        
        p, q = self.tf.lookupTransform("ardrone_base_frontcam", "world", time2)
        self.position_i2 = -np.array((p))
        self.quat_w_to_i_i2 = tf.transformations.quaternion_inverse(q)
        
        p, q = self.tf.lookupTransform("ardrone_base_frontcam", "world", time1)
        self.position_i1 = -np.array((p))
        self.quat_w_to_i_i1 = tf.transformations.quaternion_inverse(q)
        
        
        """====================================================================
        # Get relative motion in image co-ordinates
        ===================================================================="""
        
        # Difference in position in world co-ordinates
        trans = np.array(([(self.position_w2[0] - self.position_w1[0])],
                          [(self.position_w2[1] - self.position_w1[1])],
                          [(self.position_w2[2] - self.position_w1[2])]))
        print trans        
        R = tf.transformations.quaternion_matrix(self.quat_i_to_w_w1)[:3, :3]
        print R
        # Get difference in position in image co-ordinates
        trans = R.dot(trans)
        self.image_coord_trans = -np.array([trans[0], trans[1], trans[2]])        
        # Get relative quaternion using qmid = (qbefore^-1).qafter
        self.relative_quat = tf.transformations.quaternion_inverse(
                                tf.transformations.quaternion_multiply(
                                    tf.transformations.quaternion_inverse(
                                        self.quat_w_to_i_i1), 
                                        self.quat_w_to_i_i1))
    
    
    def process(self, stampedFrames):
        """Takes matched points. Filters points prior to triangulation"""
        print "Receiving data"
        
        self.debug_text = []
        self.upper_debug_text = []
        
        self.stampedFrames = stampedFrames
        
        
        i1_pts = np.reshape(np.array(stampedFrames.frame1.pts), (-1, 2))
        i2_pts = np.reshape(np.array(stampedFrames.frame2.pts), (-1, 2))
        
        print i1_pts
        print i2_pts
        
        i1_pts_draw = i1_pts
        i2_pts_draw = i2_pts
        
        
        """====================================================================
        Rotation and translation from navdata
        ===================================================================="""
        self.get_change_in_tf(stampedFrames.frame1.header.stamp, stampedFrames.frame2.header.stamp)
        
    
        print len(i1_pts), " matched points"

        """====================================================================
        Undistort points using known camera calibration
        ==================================================================="""
        if self.calibrated:
            # Undistort points using calibration data    
            i1_pts_undistorted = cv2.undistortPoints(np.array([i1_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)[0]
            i2_pts_undistorted = cv2.undistortPoints(np.array([i2_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)[0]
        else:
            print "WARNING: No calibration info. Cannot Continue"
            return       

        
        """============================================================
        Extract F and filter outliers
        ============================================================"""
        E, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr= self.extract_fundamental(i1_pts_undistorted, i2_pts_undistorted, i1_pts_draw, i2_pts_draw)
        if (i1_pts_corr == None or len(i1_pts_corr) < 1):
            print "No inliers consistent with F"
            return        
            
        print len(i1_pts_corr), " F fitted points"
            
        """============================================================
        # Extract P1 and P2 via E
        ============================================================"""
        success, P1, P2, i1_pts_final, i2_pts_final, i1_pts_draw_final, i2_pts_draw_final = self.extract_projections(E, i1_pts_corr, i2_pts_corr, i1_pts_draw_corr, i2_pts_draw_corr)
        if not success: # Bottom out on fail
            return
        
        R = P2[:,:3]
        self.image_based_R2 = R
        t = P2[:,3:4]
        self.image_based_t = t
        
        self.prev_i1_pts_final = i1_pts_final
        self.prev_i2_pts_final = i2_pts_final
        
        
        
        print len(i1_pts_final), " E fitted points"
        
        
        """====================================================================
        # Triangulate points and publish point cloud
        ===================================================================="""
        points3D_image = self.tf_triangulate_points(i1_pts_corr, i2_pts_corr)
        if points3D_image != None:
            self.publish_cloud(points3D_image, stampedFrames.frame1.header.stamp)
        
        
        
        """====================================================================
        # Plot fully tracked points
        # Only that fit with the calculated geometry are plotted
        # Note: This plots undistorted points on the distorted image
        ===================================================================="""
         # ROS to cv image
        bridge = CvBridge()
        cvimg = bridge.imgmsg_to_cv(stampedFrames.frame1.image,"bgr8")
        # cv to cv2 numpy array image
        grey_previous = cv2.cvtColor(np.asarray(cvimg), cv2.COLOR_BGR2GRAY)
        cvimg = bridge.imgmsg_to_cv(stampedFrames.frame2.image,"bgr8")
        # cv to cv2 numpy array image
        grey_now = cv2.cvtColor(np.asarray(cvimg), cv2.COLOR_BGR2GRAY)
        
        
        img2 = stackImagesVertically(grey_previous, grey_now)
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        imh = grey_previous.shape[0]
        county = 0
        l = 120
        
        # Draw lines linking fully tracked points
        for p1, p2 in zip(i1_pts_draw_corr, i2_pts_draw_corr):
            county += 1
            cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
        
        # Write debug text
        for i, l in enumerate(self.debug_text):
            cv2.putText(img2, str(l), (25,img2.shape[0]-25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        for i, l in enumerate(self.upper_debug_text):
            cv2.putText(img2, str(l), (25,25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        
        # Reproject triangulated points
        for p in self.reprojected_frame1:
            cv2.circle(img2, (int(p[0]),int(p[1])), 3, (220, 20, 60), 1)
        for p in self.reprojected_frame2:
            cv2.circle(img2, (int(p[0]),int(p[1])+imh), 3, (255, 182, 193), 1)
        
        # Draw
        cv2.imshow("track", img2)
        cv2.imwrite("temp.jpg", img2)
        print "=============\r\nDrawn\r\n============="
        
        # Render Windows
        cv2.waitKey(10)

    def tf_triangulate_points(self, pts1, pts2):
        """ Triangulates 3D points from set of matches co-ords using relative
        camera position determined from tf"""
        
        # ---------------------------------------------------------------------
        # Working in image (1st frame) co-ordinates throughout
        # ---------------------------------------------------------------------
        
        # These are arrays of the triangulated points reprojected back to the
        # image plane
        self.reprojected_frame1 = []
        self.reprojected_frame2 = []
        
        # Pull tf data
        t = self.image_coord_trans
        self.debug_text.append("trans: "+str(t))
        R = tf.transformations.quaternion_matrix(self.relative_quat)[:3, :3]
        
        # Debug
        angles = np.array(tf.transformations.euler_from_quaternion(self.relative_quat))
        angles = angles*180./np.pi        
        self.debug_text.append("rot: "+str(angles)) 
        
        """
        Triangulating using Kinv premultiplied pixel co-ord and [R t]
        """
        # Pre-multiply co-ords
        pts1_norm = self.inverseCameraMatrix.dot(make_homo(pts1).transpose()).transpose()[:,:2]
        pts2_norm = self.inverseCameraMatrix.dot(make_homo(pts2).transpose()).transpose()[:,:2]       
        P1 = np.hstack((np.array([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]), np.array([[0.],[0.],[0,]])))
        P2 = np.hstack((R, t))
        print pts2_norm
        points4D = cv2.triangulatePoints(P1, P2, pts1_norm.transpose(), pts2_norm.transpose())
        points3D_image = (points4D/points4D[3])[:3]
        
        # Output number of triangulated points
        triangulated = len(points3D_image[0])+0.
        self.debug_text.append(triangulated)
        
        # Filter points that are behind the camera
        infront = points3D_image[2] > 0.        
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()        
        points3D_image= np.reshape(points3D_image[infront==True], (3, -1))
        
        # Triangulated points reprojected back to the image plane
        self.reprojected_frame1 = self.world_to_pixel_distorted(make_homo(points3D_image.T).T, np.diag((1,1,1)), np.array([[0],[0],[0]]))        
        self.reprojected_frame2 = self.world_to_pixel_distorted(make_homo(points3D_image.T).T, R, t)
        
        # Output number of forward triangulated points
        forward_triangulated = len(points3D_image[0])+0.
        self.debug_text.append(forward_triangulated)
        
        
        # Publish Cloud
        # Note: img1 camera is taken to be the origin, so time1 is used
        if triangulated != 0 and (forward_triangulated/triangulated > 0.5):
            print "Publishing Point cloud"
            return points3D_image
        else:
            print "Poor triangulation"
            return None
    
    def set_camera_info(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        if not self.calibrated:
            if len(ci.D) == 0:
                return
            self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
            self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
            self.distCoeffs = np.array([ci.D], dtype=np.float32)
            self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
            self.calibrated = True    
            print "Calibration Initialised"
    

class ScanController:    
    def __init__(self):
        self.image = None
        
        # Initialise triangulator
        self.triangulator = Triangulator()
        
        # Initialise ROS node
        self.connect()
        
    def get_frame(self, empty):
        print "Loading Frame\r\n"
        self.capture_pub.publish('/ardrone/front/image_raw')
        
        
    def process_frames(self, empty):
        print "Processing Frames\r\n"
        self.process_pub.publish('/ardrone/front/image_raw,/ardrone/front/image_raw')
    
    def connect(self):
        self.capture_pub = rospy.Publisher('/feature_matcher/load', std_msgs.msg.String)
        self.process_pub = rospy.Publisher('/feature_matcher/process', std_msgs.msg.String)
        rospy.Subscriber('/xboxcontroller/button_a',Empty,self.get_frame)
        rospy.Subscriber('/xboxcontroller/button_b',Empty,self.process_frames)
        


def run():
    rospy.init_node('Scan_Controller')
    # Initialise controller
    s = ScanController()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
