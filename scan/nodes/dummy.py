#!/usr/bin/env python

import roslib; roslib.load_manifest('scan')
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud
import math
import os
import geometry_msgs.msg as gm


class DummySampler():
    def __init__(self):
        
        self.cloud_pub = rospy.Publisher('/scan/absolute_cloud', PointCloud)
        self.walls = self.load_model_walls()
    
    @staticmethod
    def load_model_walls():
        walls = []
        walls.append(WallModel(2,2,-2,2,1,3))
        
        walls.append(WallModel(-2,2,2,2,1,3))
        walls.append(WallModel(-2,2,-2,-2,1,3))
        
        walls.append(WallModel(-2,-2,1,2,1,3))        
        walls.append(WallModel(-2,-2,-2,-1,1,3))
        
        
        
        '''
        walls.append(WallModel(2,2,2,6,1,3))
        walls.append(WallModel(-2,2,6,6,1,3))
        walls.append(WallModel(-2,-2,5,6,1,3))
        walls.append(WallModel(-2,-2,2,4,1,3))
        walls.append(WallModel(-4,-2,-2,-2,1,3))
        walls.append(WallModel(-4,-2,6,6,1,3))
        walls.append(WallModel(-4,-4,-2,6,1,3))
        '''
        return walls
        
    def create_dummy_cloud(self, event):
            cloud = PointCloud()
            cloud.header.frame_id = "/world"
            
            point_count = np.random.randint(20, 128)
            #point_count = 8
            i=0
            
            while(i < point_count):
                cloud.points.append(self.walls[np.random.randint(0, len(self.walls))].sample())
                i = i+1
                            
            self.cloud_pub.publish(cloud)
            print "Done"

class WallModel():
    def __init__(self, xmin, xmax, ymin, ymax, zmin, zmax):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.zmin = zmin
        self.zmax = zmax
    
    def sample(self):
        point = gm.Point32()
        point.x = np.random.sample()*(self.xmax-self.xmin)+self.xmin + (np.random.sample()-0.5)*0.2
        point.z = np.random.sample()*(self.ymax-self.ymin)+self.ymin + (np.random.sample()-0.5)*0.2
        point.y = np.random.sample()*(self.zmax-self.zmin)+self.zmin + (np.random.sample()-0.5)*0.2
        return point

def run():
    ds = DummySampler()
    rospy.init_node('Point_Cloud_Dummy_Generator')
    # Initialise controller
    rospy.Timer(rospy.Duration(10), ds.create_dummy_cloud)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
