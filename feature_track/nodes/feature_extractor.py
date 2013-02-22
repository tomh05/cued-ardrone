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
from custom_msgs.msg import StampedFeaturesWithImage
from copy import deepcopy

import time

class FeatureExtractor:
    def __init__(self, detector, descriptor):
        lookup = { 'ORB': 'BruteForce-Hamming', 'SIFT': 'BruteForce'}
        self.fd = cv2.FeatureDetector_create(detector)
        self.de = cv2.DescriptorExtractor_create(descriptor)
        self.bridge = CvBridge()
        self.matcher_type = lookup[descriptor]
        self.ros_image = None
        self.image_lock = False
        self.connect()
        
        # Idle till there is a valid tf
        while (not self.tf.frameExists('/world')):
            rospy.sleep(1)
        self.start_scanning(Empty)
        
    def connect(self):
        self.auto_scan_timer = None
        self.tf = tf.TransformListener()
        self.image_sub = rospy.Subscriber('/ardrone/front/image_raw', Image, self.on_got_image)
        self.feature_pub = rospy.Publisher('/feature_extractor/features', StampedFeaturesWithImage)
        
    def auto_scan(self, event):
        # Get latest world co-ords of drone
        self.image_lock = True
        self.tf.waitForTransform("/world", "/ardrone_base_frontcam", self.ros_image.header.stamp, rospy.Duration(4))
        self.position_w2, self.quat_i_to_w2 = self.tf.lookupTransform( "/world", "/ardrone_base_frontcam", self.ros_image.header.stamp)
        self.position_w2 = np.array((self.position_w2))
        self.position_i2, self.quat_w_to_i2 = self.tf.lookupTransform( "/ardrone_base_frontcam", "/world", self.ros_image.header.stamp)
        self.position_i2 = -np.array((self.position_i2))
        # Fire if magnitude of motion above limit
        diff = self.position_w2 - self.position_w1
        mag = np.sqrt(diff[0]*diff[0] + diff[1]*diff[1]+diff[2]*diff[2])
        if (mag > 0.2 and self.ros_image != None):
            if (self.check_image_translation()):
                self.auto_scan_timer.shutdown()
                self.time_prev = time.time()
                self.feature_extract()
                self.image_lock = False
                self.continue_scanning(Empty)
        self.image_lock = False
            
    def check_image_translation(self):
        # Rotate frame2 position into frame1 image co-ordinates
        R = tf.transformations.quaternion_matrix(self.quat_w_to_i2)[:3, :3]
        position_i1_i2 = R.dot(self.position_w1)
        
        # Difference in position in image (frame1) co-ordinates
        trans = np.array(([(position_i1_i2[0] - self.position_i2[0])],
                          [(position_i1_i2[1] - self.position_i2[1])],
                          [(position_i1_i2[2] - self.position_i2[2])]))
        t = np.array([trans[0], trans[1], trans[2]])
        mag = np.sqrt(t[0]*t[0]+t[1]*t[1])
        if mag < 0.2:
            return False
        else:
            return True
            
    def start_scanning(self, empty):
        # Get latest world co-ord position
        self.position_w1, self.quat_i_to_w1 = self.tf.lookupTransform( "/world", "/ardrone_base_frontcam", rospy.Time(0))
        self.position_w1 = np.array((self.position_w1))
        self.position_i1, self.quat_w_to_i1 = self.tf.lookupTransform( "/ardrone_base_frontcam", "/world", rospy.Time(0))
        self.position_i1 = -np.array((self.position_i1))
        # Start polling for shift at 10Hz
        self.auto_scan_timer = rospy.Timer(rospy.Duration(0.1), self.auto_scan)
        
    def continue_scanning(self, empty):
        self.position_w1 = self.position_w2
        self.position_i1 = self.position_i2
        self.quat_i_to_w1 = self.quat_i_to_w2
        self.quat_w_to_i1 = self.quat_w_to_i2
        self.auto_scan_timer = rospy.Timer(rospy.Duration(0.1), self.auto_scan)
        
    def stop_scanning(self, empty):
        if self.auto_scan_timer != None:
            self.auto_scan_timer.shutdown()
    
    def feature_extract(self):
        """Loads a frame and extracts features"""
        # ROS to monochrome cv image
        cvimg = self.bridge.imgmsg_to_cv(self.ros_image,"mono8")
        # cv to cv2 numpy array image
        frame = np.asarray(cvimg)
        # Extract features
        pts = self.fd.detect(frame)
        # Describe features
        kp, desc = self.de.compute(frame, pts)
        # Publish
        self.publish_features(kp, desc, frame)
        
    def publish_features(self, kp, desc, img):
        stwi = StampedFeaturesWithImage()
        stwi.header = self.ros_image.header
        #stwi.image = self.ros_image
        prev = time.time()
        stwi.points = []
        for x in kp:
            stwi.points.append(x.pt[0])
            stwi.points.append(x.pt[1])
        stwi.descriptors = desc.reshape(-1,).tolist()
        stwi.descriptors_stride = desc.shape[1]
        stwi.descriptors_matcher = self.matcher_type
        stwi.image = self.ros_image
        self.feature_pub.publish(stwi)
        print "Features published (", np.around((time.time()-self.time_prev)*1000,1),"ms)"
        
    def on_got_image(self, d):
        """Converts the ROS published image to a cv2 numpy array"""
        if self.image_lock:
            return
        self.ros_image = d
        

class tf_resetter:
    # This allows replaying of bags without needing to kill node
    def __init__(self, detector, descriptor):
        self.timestamp = None
        self.detector = detector
        self.descriptor = descriptor
        self.fe = FeatureExtractor(detector, descriptor)
        rospy.Subscriber('/tf', tfMessage, self.tf_check)
    
    def tf_check(self, tfmsg):
        time_now = tfmsg.transforms[-1].header.stamp.to_sec()
        if self.timestamp != None and time_now < self.timestamp - 5.:
            print "\r\nResetting\r\n"
            self.timestamp = time_now
            self.fe.auto_scan_timer.shutdown()
            self.fe.tf.clear()
            self.fe.ros_image = None
            self.fe.image_lock = False
            rospy.sleep(0.5)
            self.fe.start_scanning(Empty)
        self.timestamp = time_now


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
    tr = tf_resetter(detector, descriptor)
    
    # Begin ROS loop
    rospy.spin()
    

if __name__ == '__main__':
    run()
