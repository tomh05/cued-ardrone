#!/usr/bin/env python

import roslib; roslib.load_manifest('template_track')
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
from std_msgs.msg import Empty
from nav_msgs.msg import Path
import math
import time
import threading
import os
    
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
        self.frameskip = 0
        self.calibrated = False
        self.fd = cv2.FeatureDetector_create('SIFT')
        self.de = cv2.DescriptorExtractor_create('SIFT')
        self.dm = cv2.DescriptorMatcher_create('BruteForce')
        self.preload_template('/home/alex/cued-ardrone/feature_track/templates/boxTemplate.png')
        self.mag_dist = None
        self.corners = None
                
    def preload_template(self, path):
        """Template features and descriptors need only be extracted once, so
        they are pre-calced here"""
        template = cv2.imread(path) # This should really use the ROS_PACKAGE_PATH
        self.grey_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template_pts = self.fd.detect(self.grey_template)
        self.template_kp, self.template_desc = self.de.compute(self.grey_template, template_pts)
        
    def triangulate_point(self, x1,x2,P1,P2): 
        """ Point pair triangulation from
        least squares solution. """
        # Compose matrix representing simultaneous equations
        M = np.zeros((6,6))
        M[:3,:4] = P1
        M[3:,:4] = P2
        M[:3,4] = -x1
        M[3:,5] = -x2
        # Compute SVD
        U,S,V = np.linalg.svd(M)
        # numpy SVD is ordered from largest to smallest (for S)
        # so least squares solution will always lie in the last column of V
        # BUT since numpy SVD returns V transpose not V, it is the last row
        X = V[-1,:4]
        return X / X[3]
        
    def triangulate_points(self, x1,x2,P1,P2):
        """ Two-view triangulation of points in
        x1,x2 (2*n coordingates)"""
        n = x1.shape[1]
        # Make homogenous
        x1 = np.append(x1, np.array([np.ones(x1.shape[1])]), 0)
        x2 = np.append(x2, np.array([np.ones(x2.shape[1])]), 0)
        # Triangulate for each pair
        X = [ self.triangulate_point(x1[:,i],x2[:,i],P1,P2) for i in range(n)] # Looping here is probably unavoidable
        return np.array(X).T
        
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
    
    def rotation_to_euler(self, R):
        """Takes a 3x3 rotation matrix and return success, euler-angles.
        Angles are constrained to be the smallest possible for given R
        Returns success=false on gimbal lock"""
        """Based on 'Computing Euler angles from a rotation matrix' - Gregory
        G. Slabaugh"""
        
        # Check for gimbal lock (i.e. cos(theta) = 0)
        if ((R[2,0] == 1) or (R[2,0] == -1)):
            print "Gimbal lock. Rotation un-resolvable"
            return False, None
        
        theta = np.array([[-np.arcsin(R[2,0])],
                         [np.pi+np.arcsin(R[2,0])]])
                         
        psi = np.arctan2(R[2,1]/np.cos(theta),  R[2,2]/np.cos(theta))
        
        phi = np.arctan2(R[1,0]/np.cos(theta),  R[0,0]/np.cos(theta))
        
        angles = np.hstack((psi, theta, phi))
        
        if angles[0].T.dot(angles[0]) > angles[1].T.dot(angles[1]):
            #print "angles : ", angles[1]
            return True, angles[1]
            
        else:
            #print "angles : ", angles[0]
            return True, angles[0]
            
    def coord_image_to_drone_axis(self, angles):
        drone_angles = angles.copy()
        drone_angles[0] = -angles[2]
        drone_angles[1] = angles[0]
        drone_angles[2] = angles[1]
        return drone_angles
        
    def coord_drone_to_image_axis(self, angles):
        drone_angles = angles.copy()
        drone_angles[0] = angles[1]
        drone_angles[1] = angles[2]
        drone_angles[2] = -angles[0]
        return drone_angles
        
    def templateTrack(self):
        self.t_debug_text = []
        grey_now = self.grey_now
        
        """====================================================================
        Match points with template (reusing previously calculated data)
        ===================================================================="""
        t_i1_pts, t_i2_pts = self.match_points(self.template_kp, self.kp2, self.template_desc, self.desc2)
        if t_i1_pts == None or len(t_i1_pts) < 32:
            print "===================="
            print "Too few template matches"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts, t_i2_pts)
            return
            
        self.t_debug_text.append("No of matches " + str(len(t_i1_pts)))    
            
        """================================================================
        Undistort points using known camera calibration - Note PNPRansac undistorts
        ================================================================"""
        if self.calibrated:
            # Undistort points using calibration data
            t_i1_pts_undistorted = cv2.undistortPoints(np.array([t_i1_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)
            t_i2_pts_undistorted = cv2.undistortPoints(np.array([t_i2_pts]), self.cameraMatrix, self.distCoeffs, P=self.P)
        else:
            print "WARNING: No calibration info. Cannot Continue"
            return  
            
        t_i1_pts_undistorted = np.array([t_i1_pts,])
        t_i2_pts_undistorted = np.array([t_i2_pts,])
        

        """================================================================
        Implant world size in pixel co-ords
        ================================================================"""        
        temp_time = time.time()
        
        real_size_x = 0.57#0.63
        real_size_y = 0.57#0.44
        
        t_x = real_size_x*((t_i1_pts_undistorted.T[0]/self.grey_template.shape[1])-0.5)
        t_y = real_size_y*((t_i1_pts_undistorted.T[1]/self.grey_template.shape[0])-0.5)
        t_z = np.zeros(t_x.shape)
        t_i1_pts_scaled = np.array(np.hstack((t_x, t_y, t_z)), dtype=np.float32)
        
        
        
        """================================================================
        # Calculate pose and translation to matched template
        ================================================================"""
        
        R, t, inliers = cv2.solvePnPRansac(t_i1_pts_scaled, np.array(t_i2_pts, dtype=np.float32), self.cameraMatrix, self.distCoeffs)
        
        if inliers== None:
            print "===================="
            print "Template not found"
            print "===================="
            self.template_visualise(grey_now)
            return
        t_i1_pts_corr = t_i1_pts_undistorted[0][inliers[:,0]]
        t_i2_pts_corr = t_i2_pts_undistorted[0][inliers[:,0]]
        
        self.t_debug_text.append("No of fits " + str(len(t_i1_pts_corr)))
        if len(t_i1_pts_corr) < 20:
            print "===================="
            print "Too few fits"
            print "===================="
            self.template_visualise(grey_now)
            return
        
        R, J = cv2.Rodrigues(R)
        
        success, angles = self.rotation_to_euler(R)
        angles*=180/np.pi
        
        
        print "Template"
        print "R : ", R
        print "t : ", t
        print "angles : ", angles
        
        
        """================================================================
        # Checks on validity
        ================================================================"""
        # Check for reasonable distance
        if t[2] < 0.35:
            print "===================="
            print "False match - too close"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
        if t[2] > 4:
            print "===================="
            print "False match - too far"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
            
        
        # Check forward facing (potential faster solution exists)
        # Get plane of projected corners
        # Check if plane normal dot depth is negative
        length = 0.57
        width = length
        depth = 0.14
    
        # The corners of the template in anticlockwise convention        
        corners = np.array([[-length/2,-width/2,0,1],
                       [+length/2, -width/2,0,1],
                       [+length/2, +width/2,0,1],
                       [-length/2, +width/2,0,1]]).T
        # Rotate-translate                
        P =  np.diag([1.,1.,1.,1.])
        Pinner = np.hstack((R, t))
        P[:3, :4] =Pinner
        corners_rot= P.dot(corners)
        # Normalise
        corners_rot = corners_rot.T[:, :3]
        # Get plane normal
        AB = corners_rot[1]-corners_rot[0]
        AC = corners_rot[2]-corners_rot[0]
        print AC
        plane_normal = np.cross(AB,AC)
        print "plane_normal : ", plane_normal
        # Backward test
        if plane_normal[2] < 0:
            # Template backward facing
            print "===================="
            print "False match - backward facing"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
        # Acceptable angle test
        plane_normal_mag = np.sqrt(plane_normal.transpose().dot(plane_normal))
        theta = np.arccos(plane_normal[2]/plane_normal_mag)
        if theta > 1.308996939: # > 75deg:
            print "===================="
            print "False match - too much angle"
            print "===================="
            self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr)
            return
        
        
        """================================================================
        # Update distance to template
        ================================================================"""
        
        self.mag_dist = np.sqrt(t.T.dot(t)[0,0])
        self.t_debug_text.append("Dist " + str(self.mag_dist))
        
        # Make homogenous rotation matrix from R
        R4 = np.diag([0., 0., 0., 1.])
        R4[:3, :3] = R
        
        quaternion = tf.transformations.quaternion_from_matrix(R4)
        
        br = tf.TransformBroadcaster() #create broadcaster
        br.sendTransform((t[0],t[1],t[2]), 
                         # translation happens first, then rotation
                         quaternion,
                         self.time_now, # NB: 'now' is not the same as time data was sent from drone
                         "ardrone_base_link",
                         "template_match")
        
        #imh = self.grey_template.shape[0]
        #img2 = stackImagesVertically(self.grey_template, grey_now)
        #cv2.putText(img2, mag_dist_text, (25,imh+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
    
        
        
        '''
        
        """====================================================================
        Restore co-ordinates to pixel
        ===================================================================="""
        
        p_x = ((t_i1_corr_scaled.T[0]/real_size_x)+0.5)*self.grey_template.shape[1]
        p_y = ((t_i1_corr_scaled.T[1]/real_size_x)+0.5)*self.grey_template.shape[0]
        t_i1_pts_corr = np.array(np.hstack((p_x, p_y)), dtype=np.float32)
        '''
        
        """====================================================================
        Draw Matches
        ===================================================================="""
        self.template_visualise(grey_now, t_i1_pts_corr, t_i2_pts_corr, True, R, t)
    
    
    def template_overlay(self, R, t, img2):
        """====================================================================
        Plot perspective projection without homography
        ===================================================================="""
        length = 0.57
        width = length
        depth = 0.14
        
        square_side = 0.1
        
        # The corners of the template 
        if self.corners == None:       
            self.corners = np.array(  [[-length/2,-width/2,0,1],
                                       [-length/2, +width/2,0,1],
                                       [+length/2, +width/2,0,1],
                                       [+length/2, -width/2,0,1],
                                       [-length/2,-width/2,+depth,1],
                                       [-length/2, +width/2,+depth,1],
                                       [+length/2, +width/2,+depth,1],
                                       [+length/2, -width/2,+depth,1],
                                       [-square_side,-square_side,0,1],
                                       [-square_side, +square_side,0,1],
                                       [+square_side, +square_side,0,1],
                                       [+square_side, -square_side,0,1],
                                       [-square_side, -square_side,-2*square_side,1],
                                       [-square_side, +square_side,-2*square_side,1],
                                       [+square_side, +square_side,-2*square_side,1],
                                       [+square_side, -square_side,-2*square_side,1]]).T
        
        # Set up projection from extracted R and t                   
        P =  np.diag([1.,1.,1.,1.])
        Pinner = np.hstack((R, t))
        P[:3, :4] =Pinner
        
        # Reverse project corners to pixel space (effectively K.[R|t].X)
        # Instead of having inputting the real world corners with camera origin
        # We consider the template to be centred on origin xy plane at z = 0
        sub = P.dot(self.corners)[:3]
        c = self.cameraMatrix.dot(sub).T
        # Normalise
        for xy in c:
            xy/=xy[2]
        
        imh = self.grey_template.shape[0]
            
        # Draw perspective projection
        
        # Actual template square
        cv2.line(img2,(int(c[0,0]), int(c[0,1])+imh), (int(c[1,0]), int(c[1,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])+imh), (int(c[2,0]), int(c[2,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])+imh), (int(c[3,0]), int(c[3,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])+imh), (int(c[0,0]), int(c[0,1])+imh), (255, 255 , 0), 1)
        
        # Projected back face of box
        cv2.line(img2,(int(c[4,0]), int(c[4,1])+imh), (int(c[5,0]), int(c[5,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[5,0]), int(c[5,1])+imh), (int(c[6,0]), int(c[6,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[6,0]), int(c[6,1])+imh), (int(c[7,0]), int(c[7,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[7,0]), int(c[7,1])+imh), (int(c[4,0]), int(c[4,1])+imh), (255, 255 , 0), 1)
        
        # Links between front and back face
        cv2.line(img2,(int(c[0,0]), int(c[0,1])+imh), (int(c[4,0]), int(c[4,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])+imh), (int(c[5,0]), int(c[5,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])+imh), (int(c[6,0]), int(c[6,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])+imh), (int(c[7,0]), int(c[7,1])+imh), (255, 255 , 0), 1)
        
        '''
        cv2.line(img2,(int(c[8,0]), int(c[8,1])+imh), (int(c[9,0]), int(c[9,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[9,0]), int(c[9,1])+imh), (int(c[10,0]), int(c[10,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[10,0]), int(c[10,1])+imh), (int(c[11,0]), int(c[11,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[11,0]), int(c[11,1])+imh), (int(c[8,0]), int(c[8,1])+imh), (255, 255 , 0), 1)
        
        cv2.line(img2,(int(c[12,0]), int(c[12,1])+imh), (int(c[13,0]), int(c[13,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[13,0]), int(c[13,1])+imh), (int(c[14,0]), int(c[14,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[14,0]), int(c[14,1])+imh), (int(c[15,0]), int(c[15,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[15,0]), int(c[15,1])+imh), (int(c[12,0]), int(c[12,1])+imh), (255, 255 , 0), 1)
        
        cv2.line(img2,(int(c[8,0]), int(c[8,1])+imh), (int(c[12,0]), int(c[12,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[9,0]), int(c[9,1])+imh), (int(c[13,0]), int(c[13,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[10,0]), int(c[10,1])+imh), (int(c[14,0]), int(c[14,1])+imh), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[11,0]), int(c[11,1])+imh), (int(c[15,0]), int(c[15,1])+imh), (255, 255 , 0), 1)
        '''


    def template_visualise(self, img, pts1=None, pts2=None,isTemplate=False, R=None, t=None):
        """Carries out the visualisation routines for template tracking.
        Draws stacked images with tracked points and template overlay is found
        
        isTemplate is True on match and requires accompanying R and t
        """
        # Clone image for drawing
        img2 = stackImagesVertically(self.grey_template, img)
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        imh = self.grey_template.shape[0]
        
        # Draw template overlay
        if isTemplate:
            self.template_overlay(R, t, img2)
        
        # Draw matches
        if pts1 != None and pts2 != None:
            for p1, p2 in zip(pts1, pts2):
                cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
        
        # Write debug text
        for i, l in enumerate(self.t_debug_text):
            cv2.putText(img2, str(l), (img2.shape[1]/2,img2.shape[0]-25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        
        cv2.imshow("template", img2)
        
    def manager(self, img):
        """Feeds data to template match"""
        
        times = []
        time_offset = time.time()
        
        DEF_THREADING = True # Enables threading of template matching
        
        FRAMESKIP = 11 # No of frames between check
        
        # Skip frames. Need to add ROS parameter to allow setting        
        self.frameskip += 1
        if self.frameskip < 11:
            return            
        self.frameskip = 0
            
        # Convert working images to monochrome
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        self.grey_now = grey_now

        # Detect points
        pts2 = self.fd.detect(grey_now)
        self.pts2 = pts2

        # Describe points
        kp2, desc2 = self.de.compute(grey_now, pts2)
        self.kp2, self.desc2 = kp2, desc2            
        
        # Bottom out if failed to get features
        if desc2 == None or len(desc2) == 0:
            print "No Features Found"
            return
        
        # Run Template Tracking
        self.templateTrack()
        
        # Render Windows
        cv2.waitKey(3)
    
    def tf_triangulate_points(self, pts1, pts2):
        """ Triangulates 3D points from set of matches co-ords using relative
        camera position determined from tf"""
        
        print "Triangulation"
        # For ease of reading until names are made consistent and clear
        t = self.drone_coord_trans
        t = np.array([[-0.235],[0],[0]])
        print "Translation : ", t
        
        # Get rotation matrix for crone co-ord axis
        # Rebuild R
        # Flip eulers to image axis
        angles = np.array(tf.transformations.euler_from_quaternion(self.relative_quat))        
        angles = self.coord_drone_to_image_axis(angles)
        R_cam1_to_cam2 = tf.transformations.euler_matrix(angles[0],angles[1],angles[2], axes='sxyz')[:3, :3]        
        R_cam1_to_cam2 = np.diag([1,1,1])
        R = R_cam1_to_cam2
        #print "Rotation Matrix : ", R_cam1_to_cam2
        P_cam1_to_cam2 = np.hstack((R_cam1_to_cam2, t))
        T = P_cam1_to_cam2
        print P_cam1_to_cam2
        print self.P        
        
        #print self.cameraMatrix.shape
        PP1 = np.hstack((self.cameraMatrix, np.array([[0.],[0.],[0,]])))
        PP2 = self.cameraMatrix.dot(P_cam1_to_cam2)
        points3D_image = self.triangulate_points(pts1.transpose(), pts2.transpose(), PP1, PP2)[:3]
        
        infront = points3D_image[2] > 0
        #print infront
        # Filter points
        infront = np.array([infront]).transpose()
        infront = np.hstack((infront, infront, infront)).transpose()
        #print infront.shape
        #print points3D_image
        
        points3D_image= np.reshape(points3D_image[infront==True], (3, -1))
        #print points3D_image
        
        points3D= zip(*np.vstack((points3D_image[2], -points3D_image[0], -points3D_image[1])))
        #print len(points3D)
        
        self.publish_cloud(points3D)
    
    
    def speakDistance(self, d):
        print "bing-----------------------------------------------------------"
        """Audibly reads the template distance"""
        if self.mag_dist == None:
            text = "Marker not seen"
        else:
            text = "Distance to marker is "
        text+=str(self.mag_dist)
        os.system('espeak "'+text+'" --stdout | paplay')
    

    def imgproc(self, d):
        """Converts the ROS published image to a cv2 numpy array
        and passes to FeatureTracker"""
        self.time_now = d.header.stamp
        # ROS to cv image
        bridge = CvBridge()
        cvimg = bridge.imgmsg_to_cv(d,"bgr8")
        # cv to cv2 numpy array image
        npimg = np.asarray(cvimg)
        # Pass to FeatureTracker
        self.manager(npimg)
        
        
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
    rospy.init_node('Template_Tracker')
    rospy.Subscriber('/ardrone/front/image_raw',Image,m.imgproc)
    rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, m.setCameraInfo)
    rospy.Subscriber('/xboxcontroller/button_y',Empty,m.speakDistance)


def run():
    # Initialise tracker
    m = FeatureTracker()
    # Initialise ROS node
    connect(m)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
