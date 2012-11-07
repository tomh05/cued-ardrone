#!/usr/bin/env python  
import roslib
roslib.load_manifest('particlefilter')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np

import particle

from   std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ar_pose.msg import ARMarkers
from particlefilter.srv import *


from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#---------------------------
# Map Setup
mapsize = 20



class ParticleFilter:
    def __init__(self):
        self.N          = 10    # number of particles
        self.p          = []      # particle list
        
        # this service handles the IMU smoothing, integrating etc.
        print "waiting for IMU server..."
        rospy.wait_for_service('get_imu_movement') 
        self.getIMUMovement = rospy.ServiceProxy('get_imu_movement',IMUMovement)
     
     
        # markers 
        self.markerPub = rospy.Publisher('particle_filter_markers',MarkerArray)

        # compute weights each time pose data arrives
    	rospy.Subscriber('/ar_pose_marker',ARMarkers,self.markerCallback)
    	rospy.Subscriber('/xboxcontroller/button_a',Empty,self.markerCallback)

        # move particles each time navigation data arrives
        rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,self.navdataCallback)

        print "hello"
        self.createParticles()

    def createParticles(self):
        print "creating particles"
        for i in range(self.N):
            x = particle.Particle(i,rospy.Time.now())
            self.p.append(x)

    def markerCallback(self,markers): 
        pass
        # find weights
        #w = []
        #for i in range(self.N):
        #    w[i] = p[i].likelihood(markers)

        # resample


    def navdataCallback(self,navdata):
        #find change in position
        t = self.getIMUMovement(navdata).transform


        markerArray = MarkerArray()





        for i in range(self.N):
            self.p[i]=self.p[i].move(t)
            
            
            
            marker = Marker()
            marker.header.frame_id = '/world'
            # verbitam from a help topic - edit this....
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 10
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
        #print markerArray
        #
        self.markerPub.publish(markerArray)

if __name__ == '__main__':
    rospy.init_node('particle_filter')
    pf = ParticleFilter()
    rospy.spin()
