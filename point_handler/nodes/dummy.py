#!/usr/bin/env python

import roslib; roslib.load_manifest('point_handler')
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud
import math
import os
import geometry_msgs.msg as gm


def create_dummy_cloud(event):
        cloud = PointCloud()
        cloud_pub = rospy.Publisher('pointCloud', PointCloud)
        
        i = 0
        while (i < 81):
            cloud.points.append(gm.Point32())
            cloud.points[i].x = int(i/9)*(2./8.)
            cloud.points[i].y = i % 9 *(2./8.)
            cloud.points[i].z = 2
            i = i+1
        
        print cloud    
        
        cloud_pub.publish(cloud)

def run():
    rospy.init_node('Point_Cloud_Dummy_Generator')
    # Initialise controller
    rospy.Timer(rospy.Duration(10), create_dummy_cloud)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
