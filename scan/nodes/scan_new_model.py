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
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg as gm
import tf
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
import std_msgs.msg

    
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

def enum(**enums):
    return type('Enum', (), enums)

class ScanController:
    #States = enum(FLIGHT = 1, 
    #                  LOW_0 = 2, ASCEND_0 = 3, HIGH_0 = 4, ROT_0 = 5,
    #                  HIGH_90 = 6, DESCEND_90 = 7, LOW_90 = 8, ROT_90 = 9,
    #                  LOW_180 = 10, ASCEND_180 = 11, HIGH_180 = 12, ROT_180 = 13,
    #                  HIGH_270 = 14, DESCEND_270 = 15, LOW_270 = 16, ROT_270 = 17,
    #                  DONE = 18)
    
    def __init__(self):
        init_pub = rospy.Publisher('/feature_matcher/init', std_msgs.msg.String)
        init_pub.publish('/ardrone/front/image_raw')
        #self.state = States.FLIGHT
        self.image = None
        
        # Initialise tracker
        self.feature_tracker = FeatureTracker()
        
        # Initialise ROS node
        self.connect()
        
        self.frame_1_is_loaded = False
        self.frame_2_is_loaded = False
    

        
    def crude_auto_scan(self, event):
        """ Carries out a single scan
        
        At present is somewhat brute force controlled and WILL lock the thread
        whilst running
        """
        
        print "Pre\r\n"
        
        
        twist = gm.Twist()
        twist_pub = rospy.Publisher('cmd_vel',gm.Twist)

        #if self.state == States.DONE:
        #    return
        
        print "Flight\r\n"
        
        #if self.state == States.FLIGHT:
        
        """
        # Remove any inertia
        """
        twist = gm.Twist()
        twist_pub.publish(twist)
        rospy.sleep(1.0)
        #self.state = States.LOW_0
        
        """
        # Process the first frame
        """
        print "Frame1\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(1)
        self.feature_tracker.load_tf(1)
        
        """
        # Ascend 0.3m
        """
        deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
        position1, quaternion1 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)   
        
        twist = gm.Twist()
        twist.linear.z  = +0.25
        twist_pub.publish(twist)
        done = False
        
        while(not done):
            deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
            self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
            position2, quaternion2 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
            print "Height Change : ", position2[2]-position1[2]
            if (position2[2]-position1[2]> 0.3):
                done = True
                twist = gm.Twist()
                twist_pub.publish(twist)
                
        rospy.sleep(0.5)
        #self.state = States.HIGH_0
        
        """
        # Process the second frame
        """
        print "Frame2\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(2)
        self.feature_tracker.load_tf(2)
        
        self.feature_tracker.process_frames()
        
        """
        # Rotate 90 deg
        """
        print "Rotating\r\n"       
        
        
        deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
        position1, quaternion1 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
        quaternion1 = tf.transformations.quaternion_inverse(quaternion1)
        
        twist = gm.Twist()
        twist.angular.z  = +0.1
        twist_pub.publish(twist)
        done = False
        
        while(not done):
            deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
            self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
            position2, quaternion2 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
            quaternion2 = tf.transformations.quaternion_inverse(quaternion2)
            relative_quat = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(quaternion1), quaternion2)
            angles = tf.transformations.euler_from_quaternion(relative_quat)
            if (angles[2] > np.pi/2.):
                done = True
                twist = gm.Twist()
                twist_pub.publish(twist)
                
        rospy.sleep(0.5)
        
        """
        # Process the first frame
        """
        print "Frame1\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(1)
        self.feature_tracker.load_tf(1)
        
        deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
        position1, quaternion1 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)     
        
        """
        # Descend 0.3m
        """
        twist = gm.Twist()
        twist.linear.z  = -0.25
        twist_pub.publish(twist)
        done = False
        
        while(not done):
            deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
            self.tf.waitForTransform("ardrone_base_link","world", deadreckon_common_t, rospy.Duration(4))
            position2, quaternion2 = self.tf.lookupTransform("ardrone_base_link","world", deadreckon_common_t)
            if (position2[2]-position1[2]< -0.3):
                done = True
                twist = gm.Twist()
                twist_pub.publish(twist)
                
        rospy.sleep(0.5)
        #self.state = States.HIGH_0
        
        """
        # Process the second frame
        """
        print "Frame2\r\n"
        self.feature_tracker.image_flip()
        self.feature_tracker.load_frame(2)
        self.feature_tracker.load_tf(2)
        
        self.feature_tracker.process_frames()
        
    def get_frame(self, empty):
        print "Loading Frame\r\n"
        self.capture_pub('/ardrone/front/image_raw')
        
        
    def process_frames(self, empty):
        print "Processing Frames\r\n"
        self.process_pub('/ardrone/front/image_raw,/ardrone/front/image_raw')
    
    def connect(self):     
        
        self.feature_tracker.tf = tf.TransformListener()
        self.tf = self.feature_tracker.tf
        rospy.Subscriber('/ardrone/front/image_raw',Image,self.feature_tracker.imgproc)
        rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, self.feature_tracker.setCameraInfo)
        rospy.Subscriber('/feature_matcher/matches', StampedMatchesWithImages ,self.triangulate)
        self.capture_pub = rospy.Publisher('/feature_matcher/load', std_msgs.msg.String)
        self.process_pub = rospy.Publisher('/feature_matcher/process', std_msgs.msg.String)
        


def run():
    rospy.init_node('Scan_Controller')
    # Initialise controller
    s = ScanController()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
