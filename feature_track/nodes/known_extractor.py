#!/usr/bin/env python

#========================== Feature Extractor =================================
# 
# This script extracts features whenever drone moves sufficiently (>0.3m mag)
#
#==============================================================================
#
# One advantage of using such a simple node is any bottlenecks are immediately
# obvious. 
#
# The only significant processing done here is two lines:
#   pts = self.fd.detect(frame)
#   kp, desc = self.de.compute(frame, pts)
# Which take 85+% of processing time with the remainder being shared by all 
# other tasks. There is no point in attempting to optimise this code.#
#
# SIFT should be capable of 15 fps
# ORB should be capable of 75 fps (and will typically produce more features)
#
#==============================================================================

import roslib; roslib.load_manifest('feature_track')
import rospy
from rospy.numpy_msg import numpy_msg
import ardrone_autonomy.msg
import cv2 
import cv
from cv_bridge import CvBridge
import numpy as np
import tf
from tf.msg import tfMessage
from std_msgs.msg import Empty
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PolygonStamped
from custom_msgs.msg import StampedFeaturesWithImage
from copy import deepcopy
import os

import time

class FeatureExtractor:
    def __init__(self, detector, descriptor):
        lookup = { 'ORB': 'BruteForce-Hamming', 'SIFT': 'BruteForce'}
        self.fd = cv2.FeatureDetector_create(detector)
        self.de = cv2.DescriptorExtractor_create(descriptor)
        self.bridge = CvBridge()
        self.matcher_type = lookup[descriptor]
        self.directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.load_known_data()
        self.index = 0
        
        self.connect()
        
        
    def connect(self):
        self.auto_scan_timer = None
        self.tf = tf.TransformListener()
        self.feature_pub = rospy.Publisher('/feature_extractor/features', StampedFeaturesWithImage)
        self.camera_info_pub = rospy.Publisher('/ardrone/front/camera_info', CameraInfo)
        self.poly_pub = rospy.Publisher('/test/boxoutline', PolygonStamped)
        # Tf
        self.frontcam_quat = tf.transformations.quaternion_from_euler(-90.0 * (np.pi/180.), 0.0, -90.0 * (np.pi/180.))
        self.frontcam_t = (0.19, 0.0, 0.0)
        self.br = tf.TransformBroadcaster()
        # Timers
        rospy.Timer(rospy.Duration(2), self.start_known_data, oneshot=True)
        rospy.Timer(rospy.Duration(0.2), self.publish_camera_info)
        rospy.Timer(rospy.Duration(0.2), self.publish_polygon)
        
    def start_known_data(self, d):
        rospy.Timer(rospy.Duration(2.), self.known_data)

    
    def load_known_data(self):
        self.positions = []
        self.positions.append((0.44, -0.125, 0.28+0.05))
        self.positions.append((0.44, -0.125, 0.14+0.05))
        self.positions.append((0.44, -0.125, 0.42+0.05))
        self.positions.append((0.44, -0.45, 0.42+0.05))
        self.positions.append((0.44, -0.45, 0.28+0.05))
        self.positions.append((0.44, -0.45, 0.14+0.05))
        self.positions.append((0.405, -0.414, 0.14+0.05))
        self.positions.append((0.405, -0.414, 0.28+0.05))
        self.positions.append((0.405, -0.414, 0.42+0.05))
        self.positions.append((0.44, -0.125, 0.2925+0.05))
        self.positions.append((0.44, -0.125, 0.1525+0.05))
        self.positions.append((0.44, -0.125, 0.4325+0.05))
        self.times = []
        self.quats = []
        quat_zero = tf.transformations.quaternion_from_euler(0.,0.,0.)
        self.quats.append(quat_zero)
        self.quats.append(quat_zero)
        self.quats.append(quat_zero)
        self.quats.append(quat_zero)
        self.quats.append(quat_zero)
        self.quats.append(quat_zero)
        quat_one = tf.transformations.quaternion_from_euler(0., 0., 27.6*np.pi/180.)
        self.quats.append(quat_one)
        self.quats.append(quat_one)
        self.quats.append(quat_one)
        quat_two = tf.transformations.quaternion_from_euler(2.862*np.pi/180.,-1.432*np.pi/180.,0.)
        self.quats.append(quat_two)
        self.quats.append(quat_two)
        self.quats.append(quat_two)
        self.times.append(rospy.Time.from_sec(1.))
        self.times.append(rospy.Time.from_sec(2.))
        self.times.append(rospy.Time.from_sec(3.))
        
    def box_overlay(self, img, index):
        corners = np.array([[1.45, 0.4, 0.],
                                 [1.36, 0.3, 0.],
                                 [1.79, -0.09, 0.],
                                 [1.5, -0.405, 0.],
                                 [1.6, -0.5, 0],
                                 [1.45, 0.4, 0.57],
                                 [1.36, 0.3, 0.57],
                                 [1.79, -0.09, 0.57],
                                 [1.5, -0.405, 0.57],
                                 [1.6, -0.5, 0.57]]).T
        drone_position = np.array([self.positions[index]]).T
        print "Drone pos: ", drone_position
        # Camera offset in world axis
        camera_offset = np.array([self.frontcam_t]).T
        # Rotate camera_offset to align with drone
        camera_offset = tf.transformations.quaternion_matrix(self.quats[index])[:3,:3].dot(camera_offset)
        camera_position = drone_position + camera_offset
        print "Camera pos: ", camera_position
        Rwi = np.array([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]])
        print  Rwi
        R = np.array(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(self.quats[index]))[:3,:3])
        print R
        R = Rwi.dot(R)
        Pinv = np.diag((1.,1.,1.,1.))        
        P = np.hstack((R, -R.dot(camera_position)))
        homo_corners = np.append(corners,np.array([np.ones(corners.shape[1])]), 0)
        undist_pts = self.cameraMatrix.dot(P.dot(homo_corners))
        undist_pts = (undist_pts/undist_pts[2])[:2]
        c = self.distort_points(undist_pts)
        
        
        img2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # Base Line
        cv2.line(img2,(int(c[0,0]), int(c[0,1])), (int(c[1,0]), int(c[1,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])), (int(c[2,0]), int(c[2,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])), (int(c[3,0]), int(c[3,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])), (int(c[4,0]), int(c[4,1])), (255, 255 , 0), 1)
        # Top Line Line
        cv2.line(img2,(int(c[5,0]), int(c[5,1])), (int(c[6,0]), int(c[6,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[6,0]), int(c[6,1])), (int(c[7,0]), int(c[7,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[7,0]), int(c[7,1])), (int(c[8,0]), int(c[8,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[8,0]), int(c[8,1])), (int(c[9,0]), int(c[9,1])), (255, 255 , 0), 1)
        # Base Line
        cv2.line(img2,(int(c[0,0]), int(c[0,1])), (int(c[5,0]), int(c[5,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[1,0]), int(c[1,1])), (int(c[6,0]), int(c[6,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[2,0]), int(c[2,1])), (int(c[7,0]), int(c[7,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[3,0]), int(c[3,1])), (int(c[8,0]), int(c[8,1])), (255, 255 , 0), 1)
        cv2.line(img2,(int(c[4,0]), int(c[4,1])), (int(c[9,0]), int(c[9,1])), (255, 255 , 0), 1)
        cv2.imshow('test',img2)
        cv2.waitKey(3)
        
    def distort_points(self, pts):
        
        k = self.distCoeffs
        k1 = k[0][0] # Radial coeff 1
        k2 = k[0][1] # Radial coeff 2
        p1 = k[0][2] # Tangential coeff 1
        p2 = k[0][3] # Tangential coeff 2
        
          
        K = self.cameraMatrix
        fx = K[0,0]
        fy = K[1,1]
        cx = K[0,2]                    
        cy = K[1,2]
        
        x_dash = (pts[0] - cx)/fx;   
        y_dash = (pts[1] - cy)/fy;
        
        # Precalc terms (Note: This is significantly simpler than if we had all 8 distCoeffs)
        r_2 = x_dash*x_dash + y_dash*y_dash
        r_4 = r_2*r_2
        terms = (1+k1*r_2+k2*r_4)
        
        # Distortion
        x_dash2 = x_dash*terms + 2*p1*x_dash*y_dash + p2*(r_2+2*x_dash*x_dash)
        y_dash2 = y_dash*terms + 2*p2*x_dash*y_dash + p1*(r_2+2*y_dash*y_dash)
        
        # To pixel
        u = fx*x_dash2+cx
        v = fy*y_dash2+cy
          
        return np.array([u,v]).T
        
    def known_data(self, d):
        self.index = np.random.randint(1,6)#len(self.positions))
        
        #if self.index == 1:
        #   self.index = 6
        #else:
        #    self.index = 1
        self.time_prev = time.time()
        img = cv2.imread(self.directory+'/images/frame'+str(5*self.index+1).zfill(4)+'.jpg', cv2.CV_LOAD_IMAGE_GRAYSCALE)
        self.box_overlay(img,self.index)
        
        # Extract features
        pts = self.fd.detect(img)
        # Describe features
        kp, desc = self.de.compute(img, pts)
        stamp = rospy.Time.now()
        self.publish_features(kp, desc, img, stamp)
        position = self.positions[self.index]
        quat = self.quats[self.index]
        self.publish_tf(position, quat, stamp)
        print "Test image 1 published (", np.around((time.time()-self.time_prev)*1000,1),"ms)"
        print stamp, ", ", position
        
    def publish_tf(self, position, quat, stamp):
        self.br.sendTransform(position, 
                         # translation happens first, then rotation
                         quat,
                         stamp,
                         "/ardrone_base_link",
                         "/world")                         
        # (values pulled from ardrone_drive.cpp)
        # We need to publish this when using high rate navdata to prevent
        # front camera tf from lagging base ardrone
        self.br.sendTransform(self.frontcam_t, self.frontcam_quat ,stamp, '/ardrone_base_frontcam', '/ardrone_base_link')
        # NOTE: child parent order is reversed w.r.t C++
        
    def publish_camera_info(self, d):
        ci = CameraInfo()
        ci.header.stamp = rospy.Time.now()
        ci.height = 360
        ci.width  = 640
        ci.distortion_model = 'plumb_bob'
        ci.D = (-0.51516392016931, 0.251510635789479, -0.00672699546473826, 0.00372825081052872, 0)
        ci.D = (-0.512826, 0.259694, 0.002571, 0.005383, 0.000000)
        self.distCoeffs = np.array([ci.D])
        ci.K = (562.798995593713, 0, 344.47005087653, 0, 561.569729771521, 168.041554274185, 0, 0, 1)
        ci.K = (570.864816, 0.000000, 340.823378, 0.000000, 568.598436, 146.522753, 0.000000, 0.000000, 1.000000)
        self.cameraMatrix = np.reshape(np.array([ci.K]),(3,3))
        ci.P = (449.539855957031, 0, 359.719213932665, 0, 0, 526.416748046875, 164.587378445052, 0, 0, 0, 1, 0)
        self.camera_info_pub.publish(ci)
    
    def publish_polygon(self, d):
        spoly = PolygonStamped()
        poly = Polygon()
        p = Point32()
        p.x = 1.36; p.y = 0.3
        poly.points.append(deepcopy(p))
        p.x = 1.45; p.y = 0.4
        poly.points.append(deepcopy(p))
        p.x = 1.98; p.y = -0.09
        poly.points.append(deepcopy(p))
        p.x = 1.6; p.y = -0.5
        poly.points.append(deepcopy(p))
        p.x = 1.5; p.y = -0.405
        poly.points.append(deepcopy(p))
        p.x = 1.79; p.y = -0.09
        poly.points.append(deepcopy(p))
        spoly.polygon = poly
        spoly.header.frame_id = '/world'
        self.poly_pub.publish(spoly)
        
    
    def publish_features(self, kp, desc, img, stamp):
        stwi = StampedFeaturesWithImage()
        stwi.header.stamp = stamp
        stwi.header.frame_id = '/ardrone_base_frontcam'
        stwi.points = []
        for x in kp:
            stwi.points.append(x.pt[0])
            stwi.points.append(x.pt[1])
        stwi.descriptors = desc.reshape(-1,).tolist()
        stwi.descriptors_stride = desc.shape[1]
        stwi.descriptors_matcher = self.matcher_type
        self.feature_pub.publish(stwi)
        
        
    def on_got_image(self, d):
        """Converts the ROS published image to a cv2 numpy array"""
        if self.image_lock:
            return
        self.ros_image = d


def run():
    # Init Node
    rospy.init_node('Feature_Extractor')
    
    # Get parameters
    
    descriptor = rospy.get_param('~desc', 'ORB')
    if descriptor != 'ORB' and descriptor != 'SIFT':
        print descriptor, " is not a valid descriptor (SIFT or ORB) - defaulting to ORB"
        descriptor = 'ORB'
    detector = descriptor
    
    # Print startup info
    print "\r\n"
    print "===================== Feature Extractor ==========================="
    print " Using ", descriptor, "descriptor - specify with _desc (ORB or SIFT)"
    print "==================================================================="
    print "\r\n"
    
    # Initialise controller
    #fe = FeatureExtractor(detector, descriptor)
    fe = FeatureExtractor(detector, descriptor)
    
    # Begin ROS loop
    rospy.spin()
    

if __name__ == '__main__':
    run()
