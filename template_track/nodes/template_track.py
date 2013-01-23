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
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
import math
import time
import threading
import os

from copy import deepcopy

    
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
        self.directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.preload_template(self.directory+'/templates/boxTemplate.png')#/home/alex/cued-ardrone/template_track/templates/boxTemplate.png')
        self.template_toggle = False
        self.mag_dist = None
        self.corners = None
        
        self.avg_time = 0
        self.accepted = 0
        
        self.img_pub = rospy.Publisher("/template_track/img",Image)
        self.corner_pub = rospy.Publisher("/template_track/corners", Float32MultiArray)
        self.wall_trigger_pub = rospy.Publisher("/template_track/wall_trigger", Empty)
        
        #pyglet_thread = threading.Thread(target = self.pyglet)
        #pyglet_thread.start()
        
        self.published_no = 0
        
        self.connect()
        
    def connect(self):
        rospy.init_node('Template_Tracker')
        self.tf = tf.TransformListener()
        rospy.Subscriber('/ardrone/front/image_raw',Image,self.imgproc)
        rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, self.setCameraInfo)
        rospy.Subscriber('/xboxcontroller/button_y',Empty,self.speakDistance)
        rospy.Subscriber('/xboxcontroller/button_b',Empty,self.toggle_template)
        rospy.spin()
        

        
        
        
        
    def toggle_template(self, d):
        if self.template_toggle == False:
            self.preload_template(self.directory+'/templates/boxTemplate2.png')
            self.template_toggle = True
        else:
            self.preload_template(self.directory+'/templates/boxTemplate.png')
            self.template_toggle = False
        text = "Switched template"
        os.system('espeak "'+text+'" --stdout | paplay')
            
        
        newPose = gm.PoseStamped()
        newPose.header.stamp         = self.time_now
        newPose.header.frame_id      = 'template_match'
        dummy = Path()
        dummy.poses.append(newPose)
        pathPub = rospy.Publisher('/template_toggle_dummy',Path)
        pathPub.publish(dummy)
                
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
        self.corners_rot = corners_rot
        # Get plane normal
        AB = corners_rot[1]-corners_rot[0]
        AC = corners_rot[2]-corners_rot[0]
        #print AC
        plane_normal = np.cross(AB,AC)
        self.plane_normal = plane_normal
        #print "plane_normal : ", plane_normal
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
        
        self.quaternion = tf.transformations.quaternion_from_matrix(R4)
        
        print "Broadcasting tf"
        br = tf.TransformBroadcaster() #create broadcaster
        br.sendTransform((t[0],t[1],t[2]), 
                         # translation happens first, then rotation
                         self.quaternion,
                         self.time_now, # NB: 'now' is not the same as time data was sent from drone
                         "/template_match",
                         "/ardrone_base_frontcam"
                         )
        

        newPose = gm.PoseStamped()
        newPose.header.stamp         = self.time_now
        newPose.header.frame_id      = 'template_match'

        dummy = Path()
        dummy.poses.append(newPose)

        pathPub = rospy.Publisher('/template_dummy',Path)
        pathPub.publish(dummy)
        
        
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
        
        
        
        return True
    
    def template_wall(self, R, t, img2):
        
        
        # Get plane normal
        #AB = np.array([self.c[1]-self.c[0]])
        #AC = np.array([self.c[2]-self.c[0]])
        #print AC
        #plane_normal = np.cross(AB.T, AC.T)
        #print plane_normal
        
        print "\r\n\r\n"
        #print np.array([self.corners.T[0]]).T
        #print t
        
        # The corners of the template in anticlockwise convention        
        #print self.corners_rot
        #print self.plane_normal
        
        #d = -(self.plane_normal.dot(self.corners_rot[0]))
        #print d
        
        
        
        """
        # Determine scale of image
        # 
        # Use known template size in plane to determine metres/pixel in plane
        # Use this to determine in plane corner position
        # Project into true 3D using R and t to get world corner positions
        """
        origin = np.array([[0.,0.,0.,1.]]).T
        centre = self.world_to_pixel_distorted(origin, R, t)
        print "Centre : ", centre
        print "self.c :", self.c
        print "self.c :", self.c[0,0]
        print "self.c :", self.c[2,0]
        scale_x = (self.width)/(self.c[2,0]-self.c[0,0])
        scale_y = (self.length/2.)/(self.c[1,1]-self.c[0,1])
        print "Scale x :", scale_x
        print "Scale y :", scale_y
        
        scale = np.array([scale_x, scale_y])
        
        centre_x = centre[0,0]
        centre_y = centre[0,1]
        #print centre_x
        #print centre_y
        
        size_x = img2.shape[1]
        size_y = img2.shape[0]
        
        corners = np.array([[(0.-centre_x)*scale_x,(0.-centre_y)*scale_y,0.,1.],
                           [(size_x-centre_x)*scale_x, (0.-centre_y)*scale_x,0.,1.],
                           [(size_x-centre_x)*scale_x, (size_y-centre_y)*scale_x,0.,1.],
                           [(0.-centre_x)*scale_x, (size_y-centre_y)*scale_x,0.,1.]]).T
        
        print "Unprojected corners :\r\n", corners
        
        # Rotate-translate                
        P =  np.diag([1.,1.,1.,1.])
        Pinner = np.hstack((R, t))
        P[:3, :4] =Pinner
        corners_rot= P.dot(corners)
        # Normalise
        corners_rot = corners_rot.T[:, :3].T
        
        print "Corners in image coordinates w.r.t drone:\r\n", corners_rot
        
        self.tf.waitForTransform("ardrone_base_frontcam", "world", self.time_now, rospy.Duration(16))
        position, quat = self.tf.lookupTransform("ardrone_base_frontcam", "world", self.time_now)
        
        
        # Get position of drone in current image coordinates
        position = -np.array((position))
        print "Drone position :\r\n", position
        self.t_debug_text.append(position)
        
        # Form ready to add
        position = np.hstack((position, position))
        position = np.reshape(np.hstack((position, position)), (-1, 3))
        
        # Shift to absolute image co-ords
        corners_absolute_image = corners_rot.T + position
        
        corners_absolute_drone = tf.transformations.quaternion_matrix(quat)[:3, :3].dot(corners_absolute_image.T).T
        #corners_absolute_drone = tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(quat))[:3, :3].dot(corners_absolute_image.T).T
        
        print "Absolute corners :\r\n", corners_absolute_drone
        
        #corners_rot_drone_coords = np.array((corners_rot[2], -corners_rot[0], -corners_rot[1])).T
        
        #print corners_rot_drone_coords
        
        
        #print wall.c1
        float_array = []
        float_array.append(corners_absolute_drone[0,0])
        float_array.append(corners_absolute_drone[0,1])
        float_array.append(corners_absolute_drone[0,2])
        float_array.append(corners_absolute_drone[1,0])
        float_array.append(corners_absolute_drone[1,1])
        float_array.append(corners_absolute_drone[1,2])
        float_array.append(corners_absolute_drone[2,0])
        float_array.append(corners_absolute_drone[2,1])
        float_array.append(corners_absolute_drone[2,2])
        float_array.append(corners_absolute_drone[3,0])
        float_array.append(corners_absolute_drone[3,1])
        float_array.append(corners_absolute_drone[3,2])
        print "Arrayed absolute image coordinates :\r\n", float_array
       
        #pub = rospy.Publisher('/ardrone/walls',Wall);
        #pub.publish(wall)
        #self.world.walls.append(wall)
        #print self.world.walls
        
        '''
        float_array.append(0)
        float_array.append(0)
        float_array.append(1)
        float_array.append(0)
        float_array.append(1)
        float_array.append(1)
        float_array.append(0)
        float_array.append(1)
        
        
        # cv2 to cv to ROS image
        bridge = CvBridge()
        cvimg = cv.fromarray(img2)
        imgmsg = bridge.cv_to_imgmsg(cvimg, encoding="passthrough")
        imgmsg.header.stamp = self.time_now
        self.img_pub.publish(imgmsg)
        
        # publish corners
        corner_msg = Float32MultiArray()
        corner_msg.data = float_array
        self.corner_pub.publish(corner_msg)
        
        # publish trigger
        self.wall_trigger_pub.publish()
        
        
        cv2.waitKey(0)
        '''
        
        
        
        """
        # Undistort image
        """
        
        
        """
        
        We want the image as on the actual plane (object plane)
        _____
        ^^^^/
        |||/
        ||/  We observe the imageplane and need to project it onto object plane
        |/   (This could be done with projective texturing in pyglet)
        /    I *think* it should be more efficient to manually precalc here
        
        
        To do this projection, we use some information calculated earlier:
        
        1) The 3D planar positions (ie. as if observed plane was at z=0) of the
        image corners were found, assuming entire image is planar.
        
        2) Applying the actual R|t to these gives the actual 3D corners
        
        3) We then calculate the perspective transform from these to the camera
        image corners (ie. rectange of 640x360). 
        Note:This is a homography matrix
        
        4) The homography matrix is then translated using T^(-1).H.T
        This is necessary to keep the resultant image in the positive quadrant
        of opencv coordinate system so no image is clipped.
        
        5) The shifted homography is applied, producing an image, with the size
        specified by pre-calculated bounds. (OpenCV unhelpfully doesn't allow
        this to be simply specified as a flag, presumably as certain cases 
        could result in excessive image sizes. This is not the case for us as
        we can only make a match over a limited range of angles)
        
        """
        
        c2 = self.world_to_pixel_distorted(corners, R, t)
        print "c2 :\r\n", c2
        print c2[0]
        src = np.array([c2[0],c2[1],c2[2],c2[3]],np.float32)
        dst = np.array([[0.,0.],[img2.shape[1],0.],[img2.shape[1],img2.shape[0]],[0.,img2.shape[0]]],np.float32)
        print "src :\r\n", src
        
        print "dst :\r\n", dst
        
        #H =  cv2.getPerspectiveTransform(src, dst)
        H =  cv2.getPerspectiveTransform(dst, src)
        print "H :\r\n", H
        print c2[0]
        print np.array([[c2[0,0]],[c2[0,1]], [1]])
        testarray = np.array([[0],[0],[1]])
        #print "testarray: ", testarray
        partial = H.dot(testarray)
        #print partial
        cc1 = partial/partial[2]
        print cc1
        
        testarray = np.array([[img2.shape[1]],[0],[1]])
        #print "testarray: ", testarray
        partial = H.dot(testarray)
        #print partial
        cc2 = partial/partial[2]
        print cc2
        
        testarray = np.array([[img2.shape[1]],[img2.shape[0]],[1]])
        #print "testarray: ", testarray
        partial = H.dot(testarray)
        #print partial
        cc3 = partial/partial[2]
        print cc3
        
        testarray = np.array([[0],[img2.shape[0]],[1]])
        #print "testarray: ", testarray
        partial = H.dot(testarray)
        #print partial
        cc4 = partial/partial[2]
        print cc4
        
        cxmin = float(min(min(min(cc1[0], cc2[0]),cc3[0]),cc4[0]))
        cymin = float(min(min(min(cc1[1], cc2[1]),cc3[1]),cc4[1]))
        cxmax = float(max(max(max(cc1[0], cc2[0]),cc3[0]),cc4[0]))
        cymax = float(max(max(max(cc1[1], cc2[1]),cc3[1]),cc4[1]))
        
        print "x bounds : ", cxmin, ", ", cxmax
        print "y bounds : ", cymin, ", ", cymax
        
        
        
        #image = cv2.warpPerspective(img2, H, (2*img2.shape[1], 2*img2.shape[0]))
        
        #cv2.circle(image, (int(cc1[0]), int(cc1[1])), 5, (255, 0, 255), 1)
        #cv2.circle(image, (int(cc2[0]), int(cc2[1])), 5, (255, 0, 255), 1)
        #cv2.circle(image, (int(cc3[0]), int(cc3[1])), 5, (255, 0, 255), 1)
        #cv2.circle(image, (int(cc4[0]), int(cc4[1])), 5, (255, 0, 255), 1)
        
        #cv2.imshow('window', image)
        
        
        # Set up translation matrix
        T = np.array([[1., 0., -cxmin],[0., 1., -cymin], [0., 0., 1.]])
        print T
        
        # Shift homography by translation of [cxmin; cymin]
        Rshifted = np.linalg.inv(T).dot(H.dot(T))
        Rshifted = T.dot(H)
        print Rshifted
        
        # Project image plane onto object plane
        image = cv2.warpPerspective(img2, Rshifted, (int(np.exp2(np.ceil(np.log2((cxmax-cxmin))))),int(np.exp2(np.ceil(np.log2((cymax-cymin)))))))
        #image = cv2.warpPerspective(img2, Rshifted, (int(cxmax-cxmin),int(cymax-cymin)))
        
        # Draw corners to check correctly handled
        #cv2.circle(image, (int(cc1[0]-cxmin), int(cc1[1]-cymin)), 5, (255, 0, 255), 1)
        #cv2.circle(image, (int(cc2[0]-cxmin), int(cc2[1]-cymin)), 5, (255, 0, 255), 1)
        #cv2.circle(image, (int(cc3[0]-cxmin), int(cc3[1]-cymin)), 5, (255, 0, 255), 1)
        #cv2.circle(image, (int(cc4[0]-cxmin), int(cc4[1]-cymin)), 5, (255, 0, 255), 1)
        
        # Show for debug
        cv2.imshow('window', img2)
        cv2.imshow('window2', image)
        print "Pre-shift Corners"
        print cc1[0,0]
        print cc1[1,0]
        print cc2[0,0]
        print cc2[1,0]
        print cc3[0,0]
        print cc3[1,0]
        print cc4[0,0]
        print cc4[1,0] 
        print "Min bounds"       
        print cxmin
        print cymin
        print "Dims"
        print image.shape[1]
        print image.shape[0]
        corner1pos = (cc1[0,0]-cxmin)/image.shape[1]
        print corner1pos
        # min(.), max(.) are required to avoid numeric errors 
        float_array.append(max(0.,min(1.,(cc1[0,0]-cxmin)/image.shape[1])))
        float_array.append(1-max(0.,min(1.,(cc1[1,0]-cymin)/image.shape[0])))
        float_array.append(max(0.,min(1.,(cc2[0,0]-cxmin)/image.shape[1])))
        float_array.append(1-max(0.,min(1.,(cc2[1,0]-cymin)/image.shape[0])))
        float_array.append(max(0.,min(1.,(cc3[0,0]-cxmin)/image.shape[1])))
        float_array.append(1-max(0.,min(1.,(cc3[1,0]-cymin)/image.shape[0])))
        float_array.append(max(0.,min(1.,(cc4[0,0]-cxmin)/image.shape[1])))
        float_array.append(1-max(0.,min(1.,(cc4[1,0]-cymin)/image.shape[0])))
        print "float array :\r\n ", float_array
        
        self.published_no = self.published_no+1
        
        
        
        """
        # Publish data for renderer
        #
        # ROS doesn't support sending arbitrary messages, so data is passed as
        # two separate messages. A third trigger message is sent to trigger
        # combined proccesing of the other messages.
        """
        
        
        # cv2 to cv to ROS image
        bridge = CvBridge()
        cvimg = cv.fromarray(image)
        imgmsg = bridge.cv_to_imgmsg(cvimg, encoding="passthrough")
        imgmsg.header.stamp = self.time_now
        self.img_pub.publish(imgmsg)
        
        # publish corners
        corner_msg = Float32MultiArray()
        corner_msg.data = float_array
        self.corner_pub.publish(corner_msg)
        
        # publish trigger
        self.wall_trigger_pub.publish()
        
        
        
        
        
        '''
        c2 = self.world_to_pixel_distorted(corners, R, t)
        print "c2 :\r\n", c2
        print c2[0]
        src = np.array([c2[0],c2[1],c2[2],c2[3]],np.float32)
        dst = np.array([[0.,0.],[img2.shape[1],0.],[img2.shape[1],img2.shape[0]],[0.,img2.shape[0]]],np.float32)
        print "src :\r\n", src
        
        print self.c[:4]
                           
        H =  cv2.getPerspectiveTransform(src, dst)
        print "H :\r\n", H
        
        #image_bounds = dst.T
        #print "remapped corners :\r\n", H.dot(image_bounds)
        
        image = cv2.warpPerspective(img2, H, (2*img2.shape[1], 2*img2.shape[0]))
        
        
        cv2.imshow('window', image)

        c = -self.world_to_pixel_distorted(origin, R, t).T
        print c
        
        T = np.array([[1., 0., -c[0]],[0., 1., -c[1]], [0., 0., 1.]])
        print T
        
        Rshifted = np.linalg.inv(T).dot(H.dot(T))
        print Rshifted
        
        point1 =  Rshifted.dot(np.array([[0.,0.,0.]]).T)
        print point1
        
        image = cv2.warpPerspective(img2, Rshifted, (img2.shape[1], img2.shape[0]))
        
        cv2.circle(img2, (int(point1[0]), int(point1[1])), 5, (255, 0, 255), 1) 
        
        cv2.imshow('window2', image)
        '''
        
        '''
        src = np.array([self.c[0],self.c[1],self.c[2],self.c[3]],np.float32)
        dst = np.array([[0.,0.],[0.,128.],[128.,128.],[128.,0.]],np.float32)
        
        
        H =  cv2.getPerspectiveTransform(src, dst)
        
        image = cv2.warpPerspective(img2, H, (img2.shape[1], img2.shape[0]))
        
        
        cv2.imshow('window', image)
        
                                       
        c = -self.world_to_pixel_distorted(centre, R, t).T
        print c
        
        T = np.array([[1., 0., c[0]],[0., 1., c[1]], [0., 0., 1.]])
        print T
        
        Rshifted = np.linalg.inv(T).dot(H.dot(T))
        print Rshifted
        
        point1 =  Rshifted.dot(np.array([[0.,0.,0.]]).T)
        print point1
        
        image = cv2.warpPerspective(img2, Rshifted, (img2.shape[1], img2.shape[0]))
        
        cv2.circle(img2, (int(point1[0]), int(point1[1])), 5, (255, 0, 255), 1) 
        
        cv2.imshow('window', image)
        '''
        
        '''
        Rt = np.hstack((R, t))
        KRt = self.cameraMatrix.dot(Rt)
        KRtdashKRt = KRt.T.dot(KRt)
        psuedo_inverse = np.linalg.inv(KRtdashKRt).dot(KRt.T)
        #print psuedo_inverse
        
        corners = []
        c1 = np.array([0., 0.])
        c2 = np.array([0., img2.shape[0]])
        c3 = np.array([img2.shape[1], img2.shape[0]])
        c4 = np.array([img2.shape[1], 0.])
        corners = np.array((c1,c2,c3,c4))
        print corners
        
        corners = self.make_homo(cv2.undistortPoints(np.array([corners]), self.cameraMatrix, self.distCoeffs, P=self.P)[0])
        print corners
        
        print psuedo_inverse.dot(corners)
        
        #corners = KRtinv.dot(corners)
        
        #print corners
        '''
        
    
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
    
    def template_overlay(self, R, t, img2):
        """====================================================================
        Plot perspective projection without homography
        ===================================================================="""
        length = 0.57
        width = length
        depth = 0.14
        
        self.length = length
        self.width = width
        
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
                                       
        c = self.world_to_pixel_distorted(self.corners, R, t)
        self.c =c
        '''
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
        '''
        
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
        
        if isTemplate:
            self.template_wall(R, t, img)
        
        # Draw matches
        if pts1 != None and pts2 != None:
            for p1, p2 in zip(pts1, pts2):
                cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
        
        # Write debug text
        for i, l in enumerate(self.t_debug_text):
            cv2.putText(img2, str(l), (img2.shape[1]/2,25*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (63, 63, 255))
        
        cv2.imshow("template", img2)
        
    def manager(self, img):
        """Feeds data to template match"""
        
        launch = time.time()
        
        
        DEF_THREADING = True # Enables threading of template matching
        
        FRAMESKIP = 11 # No of frames between check
        
        # Skip frames. Need to add ROS parameter to allow setting        
        self.frameskip += 1
        if self.frameskip < 7:
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
        success = self.templateTrack()
        
        # Render Windows
        cv2.waitKey(3)
        
        if success:
            self.accepted += 1
            run_time = time.time()-launch
            self.avg_time = (self.avg_time*(self.accepted-1)+run_time)/self.accepted
        print "No of tracks (accepted): ", self.accepted
        print "Average track time (accepted): ", self.avg_time      
        
    
    
    def speakDistance(self, d):
        print "bing-----------------------------------------------------------"
        """Audibly reads the template distance"""
        if self.mag_dist == None:
            text = "Marker not seen"
        else:
            text = "Distance to marker is "
        text+=str(round(self.mag_dist,3))
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
            
    
    



            
def trunc(f, n=10):
    '''Truncates/pads a float f to a string of n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return str(f)[:slen]
    
def np_to_str(n):
    """Tidily converts a numpy array to a string"""
    s = ""
    for r in n:
        s+=trunc(r)+", "
    return s

  



def run():
    # Initialise tracker
    m = FeatureTracker()

if __name__ == '__main__':
    run()
