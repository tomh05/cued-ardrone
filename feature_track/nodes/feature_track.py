#!/usr/bin/env python

#=====================Notes====================================================
# 
# It is unclear whether the translation matrix needs rescaling by camera proj
#
# It may be worth reading camera cal .yamls from disk if missing
#
# The various checks and filters carried out here should allow us to provide
# a quantified measure of confidence in the result
#
# Only need to store the keypoints and descriptors (kp# and desc#)
#==============================================================================

import roslib; roslib.load_manifest('feature_track')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
import cv
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg as gm
import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D
import tf
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path
import math
    
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
    
class FeatureTracker:
    def __init__(self):
        self.roll = 0.
        self.quaternion = tf.transformations.quaternion_from_euler(0.,0.,0., axes='sxyz')
        self.grey_previous = None
        self.frameskip = 0
        self.calibrated = False
        self.fd = cv2.FeatureDetector_create('ORB')
        self.de = cv2.DescriptorExtractor_create('ORB')
        self.dm = cv2.DescriptorMatcher_create('BruteForce')
        self.desc1 = None
        self.kp1 = None
        cv2.namedWindow("track")
        self.cloud_pub = rospy.Publisher('pointCloud', PointCloud)
        self.preload_template('/home/alex/cued-ardrone/feature_track/templates/board_undist.png')
        self.tf = tf.TransformListener()
        self.prev_position = None
        
    def preload_template(self, path):
        """Template features and descriptors need only be extracted once, so
        they are pre-calced here"""
        template = cv2.imread(path) # This should really use the ROS_PACKAGE_PATH
        self.grey_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template_pts = self.fd.detect(self.grey_template)
        self.template_kp, self.template_desc = self.de.compute(self.grey_template, template_pts)
        
    def compute_fundamental(self, x1,x2): # Not used
        n = x1.shape[1]
        # build matrix for equations
        A = np.zeros((n,9))
        for i in range(n):
            A[i] = [x1[0,i]*x2[0,i], x1[0,i]*x2[1,i], x1[0,i]*1.,
            x1[1,i]*x2[0,i], x1[1,i]*x2[1,i], x1[1,i]*1.,
            1.*x2[0,i], 1.*x2[1,i], 1. ]
        # compute linear least square solution
        U,S,V = np.linalg.svd(A)
        F = V[-1].reshape(3,3)
        # constrain F
        # make rank 2 by zeroing out last singular value
        U,S,V = np.linalg.svd(F)
        S[2] = 0
        F = np.dot(U,np.dot(np.diag(S),V))
        return F
        
    def compute_F_error(self, F, x1_32, x2_32):
        errs = []
        for i, p in enumerate(zip(x1_32.T, x2_32.T)):
            errs.append(np.r_[p[1], 1].T.dot(F).dot(np.r_[p[0], 1]))
        return np.mean(errs)
        
    def triangulate_point(self, x1,x2,P1,P2): # Not used
        """ Point pair triangulation from
        least squares solution. """
        M = np.zeros((6,6))
        M[:3,:4] = P1
        M[3:,:4] = P2
        M[:3,4] = -x1
        M[3:,5] = -x2
        U,S,V = np.linalg.svd(M)
        X = V[-1,:4]
        return X / X[3]
        
    def triangulate_points(self, x1,x2,P1,P2): # Not used
        """ Two-view triangulation of points in
        x1,x2 (3*n coordingates)"""
        n = x1.shape[1]
        if x2.shape[1] != n:
            raise ValueError("Number of points don't match.")
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        # Triangulate for each pair
        X = [ self.triangulate_point(x1[:,i],x2[:,i],P1,P2) for i in range(n)]
        return np.array(X).T

    def quaternion_multiply(self, quat1, quat2): # Not used
        v1 = np.array([quat1[0],quat1[1],quat1[2]])
        v2 = np.array([quat2[0],quat2[1],quat2[2]])
        cp = np.cross(v1, v2)
        x = quat1[0]+quat2[0]+cp[0]
        y = quat1[1]+quat2[1]+cp[1]
        z = quat1[2]+quat2[2]+cp[2]
        w = quat1[3]*quat2[3]-np.dot(v1, v2)
        return np.array([x,y,z,w])
        
    def find_and_match_points(self, grey_now):
        
        # Detect points
        pts2 = self.fd.detect(grey_now)
        self.pts2 = pts2

        # Describe points        
        kp1, desc1 = self.kp1, self.desc1
        kp2, desc2 = self.de.compute(grey_now, pts2)
        self.kp2, self.desc2 = kp2, desc2            
        
        # Bottom out if failed to get features
        if desc1 == None or desc2 == None or len(desc1) == 0 or len(desc2) == 0:
            self.grey_previous = grey_now
            self.kp1, self.desc1 = kp2, desc2
            print "No Features Found"
            return False, None, None
        
        # Match points
        i1_pts, i2_pts = self.match_points(kp1, kp2, desc1, desc2)
        
        # Bottom out if failed to get enough matches
        if len(i1_pts) < 8:
            self.grey_previous = grey_now
            self.kp1, self.desc1 = kp2, desc2
            print "Insufficient matches"
            return False, None, None
        
        return True, i1_pts, i2_pts
        
    def match_points(self, kp1, kp2, desc1, desc2):
        """Matches the two points. There appears to be no wat to specify match
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
        kp1_array = np.array(list(x.pt for x in kp1))
        kp2_array = np.array(list(x.pt for x in kp2))
        i1_pts = kp1_array[i1_indices,:]
        i2_pts = kp2_array[i2_indices,:]
        
        return i1_pts, i2_pts
        
    def make_homo(self, pts):
        pts = np.append(pts,np.array([np.ones(pts.shape[0])]).T, 1)
        return pts
        
        
    def extract_fundamental(self, i1_pts_undistorted, i2_pts_undistorted):
        """
        Extract fundamental matrix and then remove outliers
        FM_RANSAC should be good with lowish outliers
        FM_LMEDS may be more robust in some cases
        """
        F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC, param1 = 1, param2 = 0.99)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_masked = np.reshape(i1_pts_undistorted[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_undistorted[mask_prepped==1], (-1, 2))
        i1_pts_undistorted = np.array([i1_pts_masked])
        i2_pts_undistorted = np.array([i2_pts_masked])
        
        i1_pts_masked = np.reshape(self.i1_pts_draw[mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(self.i2_pts_draw[mask_prepped==1], (-1, 2))
        self.i1_pts_draw = np.array([i1_pts_masked])
        self.i2_pts_draw = np.array([i2_pts_masked])
        
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
        self.i1_pts_draw = np.reshape(self.i1_pts_draw[0][mask_nan == False], (-1, 2))
        self.i2_pts_draw = np.reshape(self.i2_pts_draw[0][mask_nan == False], (-1, 2))
        
        return F, i1_pts_corr, i2_pts_corr
        
    def extract_homography(self, i1_pts_undistorted, i2_pts_undistorted):
        """
        Extract homography and then remove outliers
        """
        H, mask = cv2.findHomography(i1_pts_undistorted, i2_pts_undistorted, cv2.RANSAC)
        # Expand mask for easy filtering
        mask_prepped = np.append(mask, mask, 1.)
        # Efficient np-style filtering, then reform
        i1_pts_masked = np.reshape(i1_pts_undistorted[0][mask_prepped==1], (-1, 2))
        i2_pts_masked = np.reshape(i2_pts_undistorted[0][mask_prepped==1], (-1, 2))
        i1_pts_undistorted = np.array([i1_pts_masked])
        i2_pts_undistorted = np.array([i2_pts_masked])              
        
        return H, i1_pts_undistorted[0], i2_pts_undistorted[0]
        
    def filter_correct_matches(self, i1_pts_undistorted, i2_pts_undistorted): # not used
        """
        Filter points that fit F using cv2.correctMatches
        This unhelpfully overwrites np.nan over rejected entried
        np.nan == np.nan returns false so have to use np.isnan(.)
        NB: This check appears redundant as F is calculated to match
        """        
        i1_pts_corr, i2_pts_corr = cv2.correctMatches(F, i1_pts_undistorted, i2_pts_undistorted)
        mask_nan = np.isnan(i1_pts_corr[0])
        i1_pts_corr = np.reshape(i1_pts_corr[0][mask_nan == False], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_corr[0][mask_nan == False], (-1, 2))
        return i1_pts_corr, i2_pts_corr
        
    def extract_projections(self, F, i1_pts_corr, i2_pts_corr):
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
        if np.linalg.det(U.dot(V))<0:
            V = -V
        # Contruct Diagonal
        SIGMA = np.diag(SIGMA)
        # Force third eig to zero
        SIGMA[2,2] = 0
        if SIGMA[0,0] < 0.7*SIGMA[1,1] or SIGMA[1,1] < 0.7*SIGMA[0,0]:
            print "WARNING: Disparate singular values"
        
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
        ind = 0
        maxfit = 0
        for i, P2 in enumerate(projections):
            # infront accepts only both dimensions
            # WARNING: cv2.tri produces unnormalised homo coords
            points4D = cv2.triangulatePoints(P1, P2, i1_pts_corr.transpose(), i2_pts_corr.transpose())
            # normalise homogenous coords
            points4D /= points4D[3]
            #points4D = self.triangulate_points(i1_pts_corr.transpose(), i2_pts_corr.transpose(), P1, P2)
            d1 = np.dot(P1,points4D)[2]
            d2 = np.dot(P2,points4D)[2]
            PI = sum((d1>0) & (d2>0))
            print "Performance index ", i, " : ", PI
            if PI > maxfit:
                maxfit = PI
                ind = i
                infront = (d1>0) & (d2>0)
        if (maxfit == 0):
            print "==================================================="
            print "P2 not extracted"
            print "==================================================="
            return False, None, None, None, None
        
        
        print projections[ind]
        
        # Filter points
        infront = np.array([infront]).transpose()
        infront = np.append(infront, infront, 1)
        i1_pts_corr = np.reshape(i1_pts_corr[infront==True], (-1, 2))
        i2_pts_corr = np.reshape(i2_pts_corr[infront==True], (-1, 2))
        self.i1_pts_draw = np.reshape(self.i1_pts_draw[infront==True], (-1, 2))
        self.i2_pts_draw = np.reshape(self.i2_pts_draw[infront==True], (-1, 2))
        
        return True, P1, projections[ind], i1_pts_corr, i2_pts_corr
    
    def update_quaternion(self, R):
        """Updates the cumulative local axis stored in self.quaternion"""
        
        # Make homogenous rotation matrix from R
        R4 = np.diag([0., 0., 0., 1.])
        R4[:3, :3] = R
        
        
        
        # Update Quaternion
        quat = tf.transformations.quaternion_from_matrix(R4)
        #print tf.transformations.euler_from_quaternion(quat)
        self.quaternion = tf.transformations.quaternion_multiply(quat,self.quaternion)
        
        '''
        br = tf.TransformBroadcaster() #create broadcaster
        br.sendTransform((0, 0, 0),
                         self.quaternion,
                         rospy.Time.now(), # NB: 'now' is not the same as time data was sent from drone
                         "image_rotation",
                         "world")
        '''
        
        # Output cumulative angles
        #angles = tf.transformations.euler_from_quaternion(self.quaternion, axes='sxyz')
        angles = tf.transformations.euler_from_quaternion(quat, axes='sxyz')
        self.image_angle_overlay = str(angles[0]*180/np.pi) + ", " + str(angles[1]*180/np.pi) + ", " + str(angles[2]*180/np.pi)
        
        # [0]=roll, [1]=pitch, [2]=yaw
        angles = tf.transformations.euler_from_quaternion(self.relative_quat, axes='sxyz')
        self.angle_overlay = str(angles[0]*180/np.pi) + ", " + str(angles[1]*180/np.pi) + ", " + str(angles[2]*180/np.pi)
        
        angles = tf.transformations.euler_from_quaternion(self.world_to_drone_quaternion, axes='sxyz')
        
        return angles
    
    def publish_cloud(self, points):
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time.now() # Should copy img header
        cloud.header.frame_id = "/ardrone_base_link" # Should be front camera really
        
        for i, p in enumerate(points): # Ideally done without a loop
            cloud.points.append(gm.Point32())
            cloud.points[i].x = p[0]
            cloud.points[i].y = p[1]
            cloud.points[i].z = p[2]
        
        self.cloud_pub.publish(cloud)

        
    def templateTrack(self, grey_now):
        """====================================================================
        Match points with template (reusing previously calculated data)
        ===================================================================="""
        t_i1_pts, t_i2_pts = self.match_points(self.template_kp, self.kp2, self.template_desc, self.desc2)
        if t_i1_pts == None or len(t_i1_pts) < 4:
            print "No template matches"
            return
            
        
        
        """====================================================================
        Undistort points using known camera calibration
        ===================================================================="""
        if self.calibrated:
            # Undistort points using calibration data
            t_i1_pts_undistorted = cv2.undistortPoints(np.array([t_i1_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)
            t_i2_pts_undistorted = cv2.undistortPoints(np.array([t_i2_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)
        else:
            print "WARNING: No calibration info. Cannot Continue"
            return
            
        t_i1_pts_undistorted = np.array([t_i1_pts,])
        t_i2_pts_undistorted = np.array([t_i2_pts,])
        
            
        """====================================================================
        Compute Planar Homography
        ===================================================================="""
        H, t_i1_pts_corr, t_i2_pts_corr = self.extract_homography(t_i1_pts_undistorted, t_i2_pts_undistorted)
        if t_i1_pts_corr == None or len(t_i1_pts_corr) < 4:
            print "Failed to extract homography"
            return        
        
        
        
        
        
        """====================================================================
        Plot extracted perspective projection
        ===================================================================="""
        
        # The corners of the template
        corners = np.array([[[0,0],
                           [self.grey_template.shape[1], 0],
                           [self.grey_template.shape[1], self.grey_template.shape[0]],
                           [0, self.grey_template.shape[0]]]], dtype=np.float32)
                           
        
        # Transform to actual view
        c = cv2.perspectiveTransform(corners, H)[0]
                           
        # Draw perspective projection
        img2 = stackImagesVertically(self.grey_template, grey_now)
        imh = self.grey_template.shape[0]
        cv2.line(img2,(int(c[0,0]), int(c[0,1])+imh), (int(c[1,0]), int(c[1,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])+imh), (int(c[2,0]), int(c[2,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])+imh), (int(c[3,0]), int(c[3,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])+imh), (int(c[0,0]), int(c[0,1])+imh), (255, 255 , 255), 2)
        

        
        
        """====================================================================
        # Calculate pose and translation to matched template
        ===================================================================="""
        
        real_size_x = 0.63 #0.57
        real_size_y = 0.44 #0.57
        
        t_x = real_size_x*((t_i1_pts_undistorted.T[0]/self.grey_template.shape[1])-0.5)
        t_y = real_size_y*((t_i1_pts_undistorted.T[1]/self.grey_template.shape[0])-0.5)
        t_z = np.zeros(t_x.shape)
        t_i1_pts_scaled = np.array(np.hstack((t_x, t_y, t_z)), dtype=np.float32)
        
        R, t, inliers = cv2.solvePnPRansac(t_i1_pts_scaled, np.array(t_i2_pts, dtype=np.float32), self.cameraMatrix, self.distCoeffs)
        R, J = cv2.Rodrigues(R)
        
        print R, t
        
        mag_dist_text = str(np.sqrt(t.T.dot(t)))
        
        imh = self.grey_template.shape[0]
        #img2 = stackImagesVertically(self.grey_template, grey_now)
        cv2.putText(img2, mag_dist_text, (25,imh+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        
        '''
        """====================================================================
        Plot perspective projection without homography
        ===================================================================="""
        
        # The corners of the template
        corners = np.array([[[-0.285,-0.285,0],
                           [-0.285, +0.285,0],
                           [+0.285, +0.285,0],
                           [+0.285, -0.285,0]]])
                           
        print np.hstack((R, t))
        
        # Transform to actual view
        c = cv2.perspectiveTransform(corners, np.hstack((R,t)))[0]
        c = 256*(c+0.5)
        
        print c
                           
        # Draw perspective projection
        
        
        cv2.line(img2,(int(c[0,0]), int(c[0,1])+imh), (int(c[1,0]), int(c[1,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])+imh), (int(c[2,0]), int(c[2,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])+imh), (int(c[3,0]), int(c[3,1])+imh), (255, 255 , 255), 2)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])+imh), (int(c[0,0]), int(c[0,1])+imh), (255, 255 , 255), 2)
        '''
        
        
        """====================================================================
        Draw Matches
        ===================================================================="""
        
        
        county = 0
        l = 35
        for p1, p2 in zip(t_i1_pts_corr, t_i2_pts_corr):
            #print p2[1]
            county += 1
            #cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
            cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
            cv2.imshow("template", img2)
        #print "No of drawn points : ", county


        
        
    def featureTrack(self, img):
        """Takes a cv2 numpy array image and compared to a previously
        buffered image. Features are extracted from each frame, 
        undistorted and matched.
        Multiple view geometry is then used to calculate the rotation and
        translation of the camera between frames and the position of observed
        points"""
        
        DEF_SET_DATA = False # Switches in fixed data
        DEF_TEMPLATE_MATCH = True # Switches template match - should be ROS param
        
        # Initialise previous image buffer
        if self.grey_previous == None:
            self.grey_previous = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            

        # Skip frames. Need to add ROS parameter to allow setting        
        self.frameskip += 1
        if self.frameskip < 11:
            return            
        self.frameskip = 0
        
            
        # Convert working images to monochrome
        grey_previous = self.grey_previous
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Initialise features and descriptor
        if self.kp1 == None:
            pts1 = self.fd.detect(grey_now)
            self.kp1, self.desc1 = self.de.compute(grey_now, pts1)
            return

        # Swap in artificial data is necessary
        if DEF_SET_DATA:
            img1 = cv2.imread("/home/alex/testData/1.jpg")
            img2 = cv2.imread("/home/alex/testData/4.jpg")
            grey_now = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            grey_previous = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            pts1 = self.fd.detect(grey_previous)
            self.kp1, self.desc1 = self.de.compute(grey_previous, pts1)

        """====================================================================
        Rotation and translation from navdata
        ===================================================================="""
        self.update_tf()
        #self.projection_from_navdata()
        

        """====================================================================
        Find matched points in both images
        ===================================================================="""
        success, i1_pts, i2_pts = self.find_and_match_points(grey_now)
        self.i1_pts_draw = i1_pts
        self.i2_pts_draw = i2_pts
        if not success:
            return
        #print "No of matched points : ", len(i1_pts)
        
        if DEF_TEMPLATE_MATCH:    
            # Carry out template match - Note this is the full procedure call and should really be threaded
            self.templateTrack(grey_now)
        
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
            
        '''
        """====================================================================
        Normalise with inverse K, this means computing F gives E
        This should avoid numerical problems in determining F
        ===================================================================="""
        i1_pts_norm = self.make_homo(i1_pts_undistorted[0])
        i2_pts_norm = self.make_homo(i2_pts_undistorted[0])
        i1_pts_norm = self.inverseCameraMatrix.dot(i1_pts_norm.T)[:2].T
        i2_pts_norm = self.inverseCameraMatrix.dot(i2_pts_norm.T)[:2].T
        print "No of undistorted points : ", len(i1_pts_norm)
        #print self.cameraMatrix.dot(i1_pts_norm)[:2].T
        '''
        
        """============================================================
        Extract F and filter outliers
        ============================================================"""
        E, i1_pts_corr, i2_pts_corr= self.extract_fundamental(i1_pts_undistorted, i2_pts_undistorted)
        if (i1_pts_corr == None or len(i1_pts_corr) < 1):
            print "No inliers consistent with F"
            self.grey_previous = grey_now
            self.kp1, self.desc1 = self.kp2, self.desc2
            return
        print "No of corrected points : ", len(i1_pts_corr)
                     

        """============================================================
        # Examine quality of F
        # Reject if error is too high and go to next frame
        ============================================================"""   
        avg_error = self.compute_F_error(E, i1_pts_corr.transpose(), i2_pts_corr.transpose())        
        print "Avg F error : ", avg_error
        if (abs(avg_error)>1): # Bottom out on high error
            print "===================="
            print "F Error too high"
            print "===================="
            self.grey_previous = grey_now
            self.kp1, self.desc1 = self.kp2, self.desc2
            return
        
        #============================================================================================================DOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOM
        self.tf_triangulate_points(i1_pts_corr, i2_pts_corr)
            
        """============================================================
        # Extract P1 and P2 via E
        ============================================================"""
        success, P1, P2, i1_pts_final, i2_pts_final = self.extract_projections(E, i1_pts_corr, i2_pts_corr)
        if not success: # Bottom out on fail
            self.grey_previous = grey_now
            self.kp1, self.desc1 = self.kp2, self.desc2
            return
        #print "No of projected points : ", len(i1_pts_final)
        
        R = P2[:,:3]
        #print "Rotation Matrix : ", R
        t = P2[:,3:4]
        
        """============================================================
        # Update cumulative orientation quaternion 
        ============================================================"""                
        angles = self.update_quaternion(R) 
        
        
        """====================================================================
        # Calculate 3D points
        ===================================================================="""
        points4D = cv2.triangulatePoints(P1, P2, i1_pts_final.transpose(), i2_pts_final.transpose())
        # normalise homogenous coords
        points4D /= points4D[3]
        
        # From camera one frame of ref
        X1 = P1.dot(points4D)
        med1 = 4*np.median(X1[2])
        # From camera two frame of ref
        X2 = P2.dot(points4D)
        med2 = 4*np.median(X2[2])
        
        # Remove unrealistically far back points
        reasonable = (X1[2]<med1) & (X2[2]<med2)
        reasonable = np.array([reasonable]).transpose()
        reasonable = np.append(reasonable, reasonable, 1)
        i1_pts_final = np.reshape(i1_pts_corr[reasonable==True], (-1, 2))
        i2_pts_final = np.reshape(i2_pts_corr[reasonable==True], (-1, 2))
        self.i1_pts_draw = np.reshape(self.i1_pts_draw[reasonable==True], (-1, 2))
        self.i2_pts_draw = np.reshape(self.i2_pts_draw[reasonable==True], (-1, 2))
        
        
        
        # Reform
        points3D1 = zip(*X1)
        points3D2 = zip(*X2)
        
        '''
        """============================================================
        # Publish point cloud
        ============================================================"""
        #self.cloud_from_navdata(i1_pts_final, i2_pts_final)
        self.publish_cloud(points3D1)
        

        #print "final : ", i1_pts_final
        '''
        '''
        """====================================================================
        # Convert back to pixel co-ords
        ===================================================================="""
        i1_pts_pixel = self.make_homo(i1_pts_final)
        i2_pts_pixel = self.make_homo(i2_pts_final)
        i1_pts_pixel = self.cameraMatrix.dot(i1_pts_pixel.T)[:2].transpose()
        i2_pts_pixel = self.cameraMatrix.dot(i2_pts_pixel.T)[:2].transpose()
        '''
        
        """====================================================================
        # Plot fully tracked points
        # Only that fit with the calculated geometry are plotted
        # Note: This plots undistorted points on the distorted image
        ===================================================================="""
        img2 = stackImagesVertically(grey_previous, grey_now)
        imh = grey_previous.shape[0]
        county = 0
        l = 120
        for p1, p2 in zip(self.i1_pts_draw, self.i2_pts_draw):
            county += 1
            cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
            
            
        # Draw artificial horizon for lower image
        h0 = grey_now.shape[0]
        w0 = grey_now.shape[1]
        h = int((3*h0/2)*(1-math.sin(angles[1])))
        cv2.line(img2, (w0/2, h), (int(w0/2+l*math.cos(angles[0])), int(h+l*math.sin(angles[0]))), (255, 255, 255), 1)
        cv2.line(img2, (w0/2, h), (int(w0/2-l*math.cos(angles[0])), int(h-l*math.sin(angles[0]))), (255, 255, 255), 1)
        
        
        # Overlay text (more readable than console)
        cv2.putText(img2, self.angle_overlay, (25,25), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
        cv2.putText(img2, self.image_angle_overlay, (25,50), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
        cv2.putText(img2, str(self.drone_coord_trans), (25,75), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
        cv2.putText(img2, str(self.temp_text), (25,100), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
        
        # Draw
        cv2.imshow("track", img2)
        cv2.waitKey(1)
                
            
        """====================================================================
        # Update previous image buffer
        ===================================================================="""
        self.grey_previous = grey_now
        self.pts1 = self.pts2
        self.kp1, self.desc1 = self.kp2, self.desc2
        
    def update_tf(self):
        """Updates the cache of most recent pose information"""
        t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        
        # This is world w.r.t ardrone_base_link but yields a result consistent with ardrone_base_link w.r.t world
        # Documentation, or more likely my understanding of it is wrong (the other way round gives world origin in drone coords)
        position, self.world_to_drone_quaternion = self.tf.lookupTransform("world", "ardrone_base_link", t)
        
        gamma = tf.transformations.euler_from_quaternion(self.world_to_drone_quaternion, axes='sxyz')[2]
        self.temp_text = position
        
        # Get change in position
        print "position", position
        print "prev_position", self.prev_position
        if self.prev_position != None:
            # Difference in position in world axis
            trans = np.array(([position[0] - self.prev_position[0]],
                                            [position[1] - self.prev_position[1]],
                                            [0.]))
            ''' This assumes flate drone
            # Convert to drone image-proc axis (z = drone forward , x = left to right horiz, y = down)
            self.drone_coord_trans = np.array([(trans[0]*math.cos(gamma)+trans[1]*math.sin(gamma)),
                                               [0.],
                                               (trans[0]*math.cos(gamma)-trans[1]*math.sin(gamma))])
            '''
            
            R = tf.transformations.quaternion_matrix(self.world_to_drone_quaternion)[:3, :3]
            # Into ardrone_base_link co-ords
            trans = R.dot(trans)
            # Re-order axis into image co-ords
            self.drone_coord_trans = np.array([-trans[1], -trans[2], trans[0]])
            
            
            
            
            '''
            trans = np.array([[position[0] - self.prev_position[0]],
                                            [position[1] - self.prev_position[1]],
                                            [position[2] - self.prev_position[2]],
                                            [1.]])
            # Rotate to drone axis
            rot = tf.transformations.quaternion_matrix(self.world_to_drone_quaternion)
            trans = rot.dot(trans)
            # Re-map to image axis convention
            self.tf_translation = trans[:3]
            trans[0] = -trans[1]
            trans[1] = -trans[2]
            trans[2] = trans[0]
            
            self.tf_translation = trans
            '''
            
            # Get relative quaternion
            # qmid = qbefore-1.qafter
            self.relative_quat = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(self.prev_quat), self.world_to_drone_quaternion)
        self.prev_position = position
        self.prev_quat = self.world_to_drone_quaternion
        
    def tf_triangulate_points(self, pts1, pts2):
        """ Triangulates 3D points from set of matches co-ords using relative
        camera position determined from tf"""
        
        print "Triangulation"
        # For ease of reading until names are made consistent and clear
        t = self.drone_coord_trans
        print "Translation : ", t
        R_cam1_to_cam2 = tf.transformations.quaternion_matrix(self.relative_quat)[:3, :3]
        R = R_cam1_to_cam2
        print "Rotation Matrix : ", R_cam1_to_cam2
        print "Eulers : ", tf.transformations.euler_from_quaternion(self.relative_quat)
        P_cam1_to_cam2 = np.hstack((R_cam1_to_cam2, self.drone_coord_trans))
        T = P_cam1_to_cam2
        print P_cam1_to_cam2
        
        '''
        P1 = self.P
        print P1.shape
        P2 = self.P.dot(P_cam1_to_cam2)
        print P2.shape
        '''
        
        
        empty = np.array([[0],[0],[0]])
        ab = np.reshape(self.inverseCameraMatrix.dot(self.make_homo(pts1).transpose()), (3, -1))
        a = ab[0]
        b = ab[1]
        cd = np.reshape(self.inverseCameraMatrix.dot(self.make_homo(pts2).transpose()), (3, -1))
        c = cd[0]
        d = cd[1]
        
        X = (-t[0] + c*t[2])/(R[0,0]-c*R[2,0]-(b/a)*(R[0,1]-c*R[2,1]+(1/b)*(R[0,2] - c*R[2,2])))
        Y = (a/b)*X
        Z = (1/a)*X
        
        print Z[1]
        
        print "x,y,z : ", X, ", ", Y, ", ", Z
        
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time.now() # Should copy img header
        cloud.header.frame_id = "ardrone_base_link"
        
        for i, ignore in enumerate(X): # Ideally done without a loop
            print i
            cloud.points.append(gm.Point32())
            cloud.points[i].x = Z[i]
            cloud.points[i].y = -X[i]
            cloud.points[i].z = -Y[i]
        
        print "cloud : ", cloud
        self.cloud_pub.publish(cloud)
        
        '''
        points3D = np.reshape(zip(*self.triangulate_points(pts1.transpose(), pts2.transpose(), P1, P2)), (-1, 4))[:, :3]
        '''
        #print "points3D : ", points3D
        
        
        
        
    
    '''    
    def cloud_from_navdata(self, i1_pts_final, i2_pts_final):
        """Produces P1 and P2 from deadreckon data"""
        # Use camera1 as origin viewpoint
        P1 = np.append(np.identity(3), [[0], [0], [0]], 1)        
        
        R = tf.transformations.quaternion_matrix(self.relative_quat)[:3, :3]
        t = self.tf_translation
        P2 = np.append(R, t, 1)
        
        """====================================================================
        # Calculate 3D points
        ===================================================================="""
        points4D = cv2.triangulatePoints(P1, P2, i1_pts_final.transpose(), i2_pts_final.transpose())
        # normalise homogenous coords
        points4D /= points4D[3]
        
        # From camera one frame of ref
        X1 = P1.dot(points4D)
        
        # From camera two frame of ref
        X2 = P2.dot(points4D)
        
        # Reform
        points3D1 = zip(*X1)
        points3D2 = zip(*X2)        
        
        """============================================================
        # Publish point cloud
        ============================================================"""
        self.publish_cloud(points3D1)
    '''
        
        
        
        


    def imgproc(self, d):
        """Converts the ROS published image to a cv2 numpy array
        and passes to FeatureTracker"""
        # ROS to cv image
        bridge = CvBridge()
        cvimg = bridge.imgmsg_to_cv(d,"bgr8")
        # cv to cv2 numpy array image
        npimg = np.asarray(cvimg)
        # Pass to FeatureTracker
        self.featureTrack(npimg)
        
    def setCameraInfo(self, ci):
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
  
def connect(m):
    rospy.init_node('Feature_Tracker')
    rospy.Subscriber('/ardrone/front/image_raw',Image,m.imgproc)
    rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, m.setCameraInfo)


def run():
    # Initialise tracker
    m = FeatureTracker()
    # Initialise ROS node
    connect(m)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
