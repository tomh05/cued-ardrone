#!/usr/bin/env python

import roslib; roslib.load_manifest('crude_follow')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg as gm
import tf
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Empty
from nav_msgs.msg import Path
import math
import time
import os

    
class CrudeFollower:
    def __init__(self):
        loop_timer = rospy.Timer(rospy.Duration(1./10.), self.timed_callback)
        self.tf = tf.TransformListener()
        self.last_tf_time = None
        
    def timed_callback(self, event):
        self.deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "world")
        #self.template_common_t = self.tf.getLatestCommonTime("/template_match", "/ardrone_base_link")
        
        #if self.last_tf_time != None:
        #    if self.latest_common_t == self.template_common_t:
        #        print "No new data"
        #        return
        #    else:
        
        time = rospy.Time.now()
        
        self.tf.waitForTransform("template_match", "ardrone_base_link", self.deadreckon_common_t, rospy.Duration(16))
        position, self.world_to_drone_quaternion = self.tf.lookupTransform("template_match", "ardrone_base_link", time)        
        print "Processing new template data"
        
        t = gm.Twist()
        # 0.5 scaling factor for sensitivity
        

        
        
        if True:
        #if abs (position[0]) > abs(position[1]):
            if position[0] < 0:
                print "Go right"
                t.linear.y  = -0.05
            else:
                print "Go left"
                t.linear.y  = 0.05
        #else:
        #    if position[1] < 0:
        #        print "Go down"
        #    else:
        #        print "Go up"
        
        twist_pub = rospy.Publisher('cmd_vel',Twist)
        twist_pub.publish(t)
                
        
        #self.last_tf_time = self.template_common_t

    
    


def run():
    rospy.init_node('Crude_follower')
    # Initialise tracker
    m = CrudeFollower()
    # Initialise ROS node
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
