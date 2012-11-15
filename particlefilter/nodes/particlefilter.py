#!/usr/bin/env python  
import roslib
roslib.load_manifest('particlefilter')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np
import random
import particle
import struct
import pickle # for importing map files
import time

from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from ar_pose.msg import ARMarkers
from particlefilter.srv import *

#from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
#from sensor_msgs.point_cloud2 import create_cloud_xyz32

#from sensor_msgs.point_cloud2 import create_cloud_xyz32

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#---------------------------
# Map Setup
mapfile = open("../maps/singlemarker.map","r")
worldmap = pickle.load(mapfile)

mapVisualisation = MarkerArray()
for i in range(len(worldmap.markers)):
    marker = Marker()
    marker.header.frame_id = '/world'
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.id=i
    marker.pose = worldmap.markers[i].pose.pose
    mapVisualisation.markers.append(marker)
 

class ParticleFilter:
    def __init__(self):
        self.N          = 100    # number of particles
        self.p          = []      # particle list
      
        # Callbacks may overlap: make sure they don't by having a 'teddy' that a callback takes and receives in turn

        self.busy = False 
        # service that handles accelerometer smoothing, integrating etc.
        print "waiting for IMU server..."
        rospy.wait_for_service('get_imu_movement') 
        self.getIMUMovement = rospy.ServiceProxy('get_imu_movement',IMUMovement)
        
        # tf
        #self.tfl = tf.TransformListener()

        # marker publisher for rviz 
        #self.markerPub = rospy.Publisher('particle_filter_markers',MarkerArray)
        self.markerPub = rospy.Publisher('map_markers',MarkerArray)
        self.pclPub = rospy.Publisher('point_cloud',PointCloud)
        self.pclBPub = rospy.Publisher('point_cloudB',PointCloud)

        # move particles each time navigation data arrives
        rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,self.navdataCallback)

        # and compute weights each time pose data arrives
    	rospy.Subscriber('/ar_pose_marker',ARMarkers,self.markerCallback)
    	rospy.Subscriber('/xboxcontroller/button_a',Empty,self.markerCallback) #debug

        self.createParticles()

    def createParticles(self):
        print "creating particles"
        for i in range(self.N):
            x = particle.Particle()
            self.p.append(x)

    def navdataCallback(self,navdata):
        while (self.busy):
            time.sleep(0.001)
        # reserve control of thread
        self.busy = True
        #find change in position
        t = self.getIMUMovement(navdata).transform
    	'''
        #markerArray = MarkerArray()
        # create cloud of markers
        marker = Marker()
        marker.header.frame_id = '/world'
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        '''
        '''
        # point cloud
        pc = PointCloud2(header       = navdata.header,
                         height       = 1,
                         width        = self.N,
                         is_dense     = False,
                         is_bigendian = False)
        pc.header.frame_id = "/world"
        pc.fields = [PointField('x', 0, 4, 1), # name, offset, bytes, count
	              PointField('y', 4, 4, 1),
	              PointField('z', 8, 4, 1)]
        pc.data=[]
        #pc = create_cloud_xyz32(header,points)
        '''
        pc = PointCloud(header	= navdata.header)
        pc.header.frame_id = "/world"
        

        for i in range(self.N):
            # update particle locations
            #self.p[i]=self.p[i].move(t)
            self.p[i].move(t)
            #print self.p
             
            newp = Point32()
            newp.x =  self.p[i].pos[0]
            newp.y =  self.p[i].pos[1] 
            newp.z =  self.p[i].pos[2] 
            #newp.z =  self.p[i].age 
                
            pc.points.append(newp)
            '''
            # markerArray
            marker = Marker()
            marker.header.frame_id = '/world'
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 6
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.id=i
            marker.pose.orientation.x = self.p[i].orientation[0]
            marker.pose.orientation.y = self.p[i].orientation[1]
            marker.pose.orientation.z = self.p[i].orientation[2]
            marker.pose.orientation.w = self.p[i].orientation[3]

            marker.pose.position.x =  self.p[i].pos[0]
            marker.pose.position.y =  self.p[i].pos[1] 
            marker.pose.position.z =  self.p[i].pos[2] 

            markerArray.markers.append(marker)
            '''

        #self.markerPub.publish(markerArray)
        self.pclPub.publish(pc)
        
        # return control
        self.busy = False

    def markerCallback(self,ar_markers):
        while (self.busy):
            time.sleep(0.001)
        # reserve control of thread
        self.busy = True
        if (len(ar_markers.markers)>0):

            # find weights
            w = []
            for i in range(self.N):
                w.append(self.p[i].likelihood(worldmap,ar_markers))
                #print w[i]
            
            # debug: if the markers were used...
            if w[0]!=1.0:
                # flash marker and draw estimates
                mapVisualisation.markers[0].color.g = 0.0
                self.markerPub.publish(mapVisualisation)

                pcB = PointCloud(header	= ar_markers.header)
                pcB.header.frame_id = "/world"
                for i in range(self.N):
                    newp = Point32()
                    newp.x =  self.p[i].markerPos[0]
                    newp.y =  self.p[i].markerPos[1] 
                    newp.z =  self.p[i].markerPos[2] 
                    pcB.points.append(newp)
                self.pclBPub.publish(pcB)
            
            # resample
            index = int(random.random() * self.N)
            beta = 0.0
            mw = max(w)
            new_p = []
            for i in range(self.N):
                beta += random.random() * mw * 2.0
                while w[index]< beta:
                    beta -= w[index]
                    index = (index+1) % self.N # increment and wrap

                clonedParticle = self.p[index].clone()
                new_p.append(clonedParticle)
                for j in new_p[0:-1]:
                    if j  is clonedParticle:
                        print "Help!"
            self.p = new_p
 
        mapVisualisation.markers[0].color.g = 1.0
        self.markerPub.publish(mapVisualisation)
       # return control of thread
        self.busy = False

if __name__ == '__main__':
    rospy.init_node('particle_filter')
    pf = ParticleFilter()
    rospy.spin()
