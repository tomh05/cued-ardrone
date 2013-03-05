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
from std_msgs.msg import Empty
from std_msgs.msg import Header
from ardrone_autonomy.msg import navdata_altitude
import math
import time
from copy import deepcopy

class Controller:
    def __init__(self):
        self.connect()
        self.altitude = 0.
    
    def connect(self):
        self.tf = tf.TransformListener()
        rospy.Subscriber('/xboxcontroller/button_x', Empty, self.on_got_start_command)
        rospy.Subscriber('/ardrone/navdata_altitude', navdata_altitude, self.on_got_altitude)
        self.twist_pub = rospy.Publisher('cmd_vel',gm.Twist)
        self.trigger_pub = rospy.Publisher('/xboxcontroller/button_a',Empty);
        
    def on_got_altitude(self, d):
        self.altitude = (d.altitude_raw*1.) / 1000

    def on_got_start_command(self, empty):
        print "Beginning Scan"
        
        print "Stabilising..."
        """
        # Remove any inertia
        """
        twist = gm.Twist()
        self.twist_pub.publish(twist)
        rospy.sleep(0.5)
        print  "    Done"
        
        """
        # Process the first frame
        """
        print "Loading first frame"
        self.trigger_pub.publish()
        
        """
        # Ascend 0.3m
        """
        self.ascend_count = 0
        
        
        position1, quaternion1 = self.tf.lookupTransform("world","ardrone_base_link", rospy.Time(0))
        
        self.start_height = position1[2]
        self.start_altitude = self.altitude
        
        twist = gm.Twist()
        twist.linear.z  = +0.25
        self.twist_pub.publish(twist)
        
        self.ascend_timer = rospy.Timer(rospy.Duration(0.1), self.ascend_callback)
        
    def ascend_callback(self, timer):
        position1, quaternion1 = self.tf.lookupTransform("world","ardrone_base_link", rospy.Time(0))
        if position1[2] - self.start_height >= 0.3:
            self.ascend_timer.shutdown()
            twist = gm.Twist()
            self.twist_pub.publish(twist)
            print "Ascended"
            rospy.sleep(0.5)
            position1, quaternion1 = self.tf.lookupTransform("world","ardrone_base_link", rospy.Time(0))
            print "deadreckon dz: ", position1[2] - self.start_height
            print "altitude   dz: ", self.altitude - self.start_altitude
            print "Loading next frame"
            self.trigger_pub.publish()
        else:
            twist = gm.Twist()
            twist.linear.z  = +0.25
            self.twist_pub.publish(twist)
    


def run():
    rospy.init_node('Crude_Control')
    
    
    
    # Print startup info
    print "\r\n"
    print "========================= Crude Control ==========================="
    print "==================================================================="
    print "\r\n"
    
    
    # Initialise controller
    c = Controller()
    
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
