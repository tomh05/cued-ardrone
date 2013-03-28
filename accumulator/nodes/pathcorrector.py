#!/usr/bin/env python

#============================ Pathcorrector ============================
# 
# This uses the accumulator refinements to correct the path.
# The output is purely for display purposes and billinearly 
# interpolated for corrections between aligned triangulations.
#
#=======================================================================

import roslib
roslib.load_manifest('accumulator')
import rospy
import tf
import math
import numpy as np
import message_filters
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import Described3DPoints
from copy import deepcopy



class PathCorrector:
    def __init__(self):
        self.connect()
        self.shifted_path = Path()
        self.path = Path()
        self.path_index = 0
        self.prev_correction = np.array([[0.],[0.],[0.]])
    
    def connect(self):
        rospy.init_node('deadreckon_broadcaster')
        rospy.Subscriber('deadreckon_path',Path, self.on_got_path)
        rospy.Subscriber('/accumulator/shifted_described_cloud',Described3DPoints, self.on_got_shift)
        self.path_pub = rospy.Publisher('/shifted_path',Path)
        self.br = tf.TransformBroadcaster()
        
    def on_got_shift(self, d):
        position_i = np.array([d.position_i]).T
        quat_i_to_w = d.quat_i_to_w
        position_w = tf.transformations.quaternion_matrix(quat_i_to_w)[:3,:3].dot(position_i)
        self.br.sendTransform((position_w[0],position_w[1],position_w[2]),quat_i_to_w,d.header.stamp,"/ardrone_aligned_cam2","/world") 
        latest = self.path.poses[-1].pose.position
        path_position_w = np.array([[latest.x],[latest.y],[latest.z]])
        print position_w
        # Correction per path entry
        delta = (position_w - path_position_w)
        shift = delta - self.prev_correction
        shift = shift/float(len(self.path.poses) - self.path_index)
        corr = deepcopy(shift) + self.prev_correction
        self.prev_correction = delta
        for i, p in enumerate(self.path.poses[self.path_index:]):
            p.pose.position.x = p.pose.position.x + corr[0,0]
            p.pose.position.y = p.pose.position.y + corr[1,0]
            p.pose.position.z = p.pose.position.z + corr[2,0]
            corr = corr + shift
            self.shifted_path.poses.append(deepcopy(p))
        pos = self.path.poses[-1].pose.position
        print (pos.x, pos.y, pos.z)
        pos = self.shifted_path.poses[-1].pose.position
        print (pos.x, pos.y, pos.z)
        self.path_pub.publish(self.shifted_path)
        self.path_index = len(self.shifted_path.poses)
    
    def on_got_path(self, d):
        self.path = d
        self.shifted_path.header = d.header

if __name__ == '__main__':
    pc = PathCorrector() # Initialise class to preserve vars
    
    
    print "\r\n"
    print "======================= Path Corrector ============================"
    print "==================================================================="
    print "\r\n"
    
    
    rospy.spin()
