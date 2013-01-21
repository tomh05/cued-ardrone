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
        cloud.header.frame_id = "/world"
        cloud_pub = rospy.Publisher('pointCloud', PointCloud)
        
        point_count = np.random.randint(20, 128)
        #point_count = 8
        i=0
        
        while(i < point_count):
            cloud.points.append(gm.Point32())
            wall = np.random.randint(1, 4+1)
            if wall == 1:
                cloud.points[-1].x = 2.*np.random.random()-1.
                cloud.points[-1].y = 2.*np.random.random()-1.
                cloud.points[-1].z = 1.
            elif wall == 2:
                cloud.points[-1].x = -1.
                cloud.points[-1].y = 2.*np.random.random()-1.
                cloud.points[-1].z = 2.*np.random.random()-1.
            elif wall == 3:
                cloud.points[-1].x = 2.*np.random.random()-1.
                cloud.points[-1].y = 2.*np.random.random()-1.
                cloud.points[-1].z = -1.
            elif wall == 4:
                cloud.points[-1].x = 1.
                cloud.points[-1].y = 2.*np.random.random()-1.
                cloud.points[-1].z = 2.*np.random.random()-1.
            i = i+1
        
        '''
        i = 0dw
            cloud.points.append(gm.Point32())
            cloud.points[i].x = int(i/9)*(2./8.)
            cloud.points[i].y = i % 9 *(2./8.)
            cloud.points[i].z = 2
            i = i+1
        
        print cloud    
        '''
        
        cloud_pub.publish(cloud)
        print "Done"
        
        

def run():
    rospy.init_node('Point_Cloud_Dummy_Generator')
    # Initialise controller
    rospy.Timer(rospy.Duration(10), create_dummy_cloud)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
