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
        #loop_timer = rospy.Timer(rospy.Duration(1./10.), self.timed_callback)
        self.tf = tf.TransformListener()
        self.last_tf_time = None
        self.enable = False
        
    def timed_callback(self, event):
        
        
        self.deadreckon_common_t = self.tf.getLatestCommonTime("ardrone_base_link", "template_match")
        #self.template_common_t = self.tf.getLatestCommonTime("/template_match", "/ardrone_base_link")
        
        #if self.last_tf_time != None:
        #    if self.latest_common_t == self.template_common_t:
        #        print "No new data"
        #        return
        #    else:
        
        time = rospy.Time.now()
        
        self.tf.waitForTransform("ardrone_base_frontcam","template_match", self.deadreckon_common_t, rospy.Duration(4))
        position, self.world_to_drone_quaternion = self.tf.lookupTransform("ardrone_base_frontcam","template_match", self.deadreckon_common_t)        
        print "Processing new template data"
        print position
        
        pos = np.array([position]).T
        print pos
        depth = np.sqrt(pos.T.dot(pos)[0,0])
        #print "mid : ", mid
        #print depth
        
        if self.enable == False:
            return
        
        t = gm.Twist()

        delta_yaw = np.arctan(-position[0]/position[2])
        print delta_yaw*180/np.pi
        
        if delta_yaw > 0.075:
            print "turning left"
            t.angular.z = 0.62*delta_yaw
        elif delta_yaw < - 0.075:
            print "turning right"
            t.angular.z = 0.62*delta_yaw
        
        '''    
        if position[0] < -0.05:
            print "Go left"
            t.linear.y  = +0.05
        elif position[0] > 0.05:
            print "Go right"
            t.linear.y  = -0.05
        '''        
        '''    
        if position[1] < -0.1:
            print "Go up"
            t.linear.z  = +0.05
        elif position[1] > 0.1:
            print "Go down"
            t.linear.z  = -0.05
        '''

        x_delta = depth-0.8 #position - target distance    
        if x_delta < 0.1:
            print "Go back"
            t.linear.x  = 0.2*x_delta
        elif x_delta > 0.1:
            print "Go forward"
            t.linear.x  = 0.2*x_delta
                
            
        
        
        twist_pub = rospy.Publisher('cmd_vel',gm.Twist)
        twist_pub.publish(t)
                
        
        #self.last_tf_time = self.template_common_t
        
    def toggle_enable(self, d):
        self.enable = not self.enable
        if self.enable:
            print "Enabling crude follow"
        else:
            print "Disabling crude follow"

    
    


def run():
    rospy.init_node('Crude_follower')
    # Initialise tracker
    m = CrudeFollower()
    rospy.Subscriber('/xboxcontroller/button_back', Empty, m.toggle_enable)
    rospy.Subscriber('/template_dummy', Path, m.timed_callback)
    t = gm.Twist()
    twist_pub = rospy.Publisher('cmd_vel',gm.Twist)
    twist_pub.publish(t)
    # Initialise ROS node
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
