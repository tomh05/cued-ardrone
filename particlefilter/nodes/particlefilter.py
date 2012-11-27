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
import threading # for locking

from std_msgs.msg import Empty
from nav_msgs.msg import Path

from ar_pose.msg import ARMarkers

from particlefilter.srv import *

from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import PoseStamped, Point32, Point

from visualization_msgs.msg import MarkerArray, Marker


#------------------------------------------------------
# debug setup

# draws pointcloud if false, or arrows if true. 
# use  less than 20 particles for arrows
useArrows = False
publishTf = False

#------------------------------------------------------
# Params setup
transNoise = rospy.get_param("/Particle_Filter/translation_noise",0.0)
rotNoise = rospy.get_param("/Particle_Filter/rotation_noise",0.0)
markerNoise = rospy.get_param("/Particle_Filter/marker_noise",0.0)

#------------------------------------------------------
# Map Setup
#mapfile = open("../maps/singlemarker.map","r")
#mapfile = open("../maps/multimarker.map","r")
map_path = rospy.get_param("/Particle_Filter/map_path")
mapfile = open(map_path,'r')

worldmap = pickle.load(mapfile)
gamma_offset = pickle.load(mapfile)
print gamma_offset

mapVisualisation = MarkerArray()
for i in range(len(worldmap.markers)):
    marker = Marker()
    marker.header.frame_id = '/world'
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.30
    marker.scale.y = 0.30
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.id=i
    marker.pose = worldmap.markers[i].pose.pose
    mapVisualisation.markers.append(marker)
 
#------------------------------------------------------

class ParticleFilter:
    def __init__(self):
        self.N          = 60     # number of particles
        self.p          = []      # particle list
        self.ar_markers = None 
        self.w = []
        self.gamma_offset = gamma_offset
        # Callbacks may overlap: make sure they don't by having a 'teddy' that a callback takes and receives in turn
        #self.busy = False 
        self.lock = threading.Lock()

        # service that handles accelerometer smoothing, integrating etc.
        rospy.loginfo("waiting for IMU server...")
        rospy.wait_for_service('get_imu_movement') 
        self.getIMUMovement = rospy.ServiceProxy('get_imu_movement',IMUMovement)
        
        if publishTf:
            self.br = tf.TransformBroadcaster() #create broadcaster

        # visualisation publishers
        if (useArrows):
            self.particleMarkerPub = rospy.Publisher('particle_markers',MarkerArray)
        else:
            self.pclPub = rospy.Publisher('point_cloud',PointCloud)
        self.pclBPub = rospy.Publisher('point_cloudB',PointCloud)
        self.mapMarkerPub = rospy.Publisher('map_markers',MarkerArray)

        # move particles each time navigation data arrives
        rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,self.navdataCallback)


        # reset gamma
        rospy.Subscriber('/xboxcontroller/button_a',Empty,self.resetgammaCallback)

        # and compute weights each time pose data arrives
    	rospy.Subscriber('/ar_pose_marker',ARMarkers,self.markerCallback,None,1) # Queue size 1

        self.createParticles()

    def resetgammaCallback(self,d):
        self.gamma_offset = math.radians(self.navdata.rotZ)
        for i in range(self.N):
            self.p[i].pos = np.array([0.0,0.0,0.0])
        rospy.loginfo("gamma offset is now " + str(self.gamma_offset))

    def createParticles(self):
        print "creating particles"
        for i in range(self.N):
            x = particle.Particle()
            x.setNoise(transNoise,rotNoise,markerNoise)
            self.p.append(x)

    def navdataCallback(self,navdata):
        self.lock.acquire()
        '''
        while (self.busy):
            time.sleep(0.001)
        # reserve control of thread
        self.busy = True
        '''
        #find change in position
        self.navdata = navdata
        movement = self.getIMUMovement(navdata=self.navdata, gamma_offset=self.gamma_offset)

        for i in range(self.N):
            # update particle locations
            self.p[i].move(movement)

        self.broadcastTf(navdata.header.stamp,0)
        self.visualiseParticles()

        # return control
        #self.busy = False
        if (self.ar_markers is not None):
            #weigh and resample
            if (len(self.ar_markers.markers)>0):
                self.findWeights(self.ar_markers)
         
                # debug: if the markers were used, draw estimates
                if self.w[0]!=1.0:
                    # flash marker red
                    mapVisualisation.markers[0].color.g = 0.0
                    self.mapMarkerPub.publish(mapVisualisation)
                    self.visualiseEstimates()
                    self.resample()
            self.ar_markers = None
        mapVisualisation.markers[0].color.g = 1.0
        self.mapMarkerPub.publish(mapVisualisation)
 
        self.lock.release()
       
    def visualiseParticles(self):
        if (useArrows):
            markerArray = MarkerArray()
            for i in range(self.N):
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
            self.particleMarkerPub.publish(markerArray)
        else:
            #pc = PointCloud(header=navdata.header)
            pc = PointCloud()
            pc.header.frame_id = "/world"
            weightChannel = ChannelFloat32(name="weight")
            pc.channels.append(weightChannel)
            for i in range(self.N):
                newp = Point32()
                newp.x =  self.p[i].pos[0]
                newp.y =  self.p[i].pos[1] 
                newp.z =  self.p[i].pos[2] 
                visualisationWeight = 1.0
                if self.ar_markers:
                    visualisationWeight =  self.p[i].likelihood(worldmap,self.ar_markers)
                pc.points.append(newp)
                pc.channels[0].values.append(visualisationWeight)

            self.pclPub.publish(pc)

    def markerCallback(self,ar_markers):
        self.lock.acquire()
        self.ar_markers = ar_markers
        self.lock.release()
        '''
        while (self.busy):
            time.sleep(0.001)
        # reserve control of thread
        self.busy = True
        if (len(ar_markers.markers)>0):
            self.findWeights(ar_markers)
         
            # debug: if the markers were used, draw estimates
            if self.w[0]!=1.0:
                # flash marker red
                mapVisualisation.markers[0].color.g = 0.0
                self.mapMarkerPub.publish(mapVisualisation)
                self.visualiseEstimates()
                
            self.resample()

        mapVisualisation.markers[0].color.g = 1.0
        self.mapMarkerPub.publish(mapVisualisation)
        # return control of thread
        self.busy = False
        '''

        
    def visualiseEstimates(self):
        pcB = PointCloud(header	= self.ar_markers.header)
        pcB.header.frame_id = "/world"
        for i in range(self.N):
            newp = Point32()
            newp.x =  self.p[i].markerPos[0]
            newp.y =  self.p[i].markerPos[1] 
            newp.z =  self.p[i].markerPos[2] 
            pcB.points.append(newp)
        self.pclBPub.publish(pcB)

    def findWeights(self,ar_markers):
        self.w = []
        for i in range(self.N):
            self.w.append(self.p[i].likelihood(worldmap,ar_markers))
           
    def resample(self):
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(self.w)
        if mw==0.0:
            print "all particles died"
        new_p = []
        for i in range(self.N):
            beta += random.random() * mw * 2.0
            while self.w[index]< beta:
                beta -= self.w[index]
                index = (index+1) % self.N # increment and wrap

            clonedParticle = self.p[index].clone()
            clonedParticle.setNoise(transNoise,rotNoise,markerNoise)
            new_p.append(clonedParticle)
        self.p = new_p

    def broadcastTf(self,time,i):
        if publishTf:
            self.br.sendTransform(self.p[i].pos,
                                  # translation happens first, then rotation
                                  self.p[i].orientation,
                                  time, 
                                  "/ardrone_base_link",
                                  "/world")

if __name__ == '__main__':
    rospy.init_node('particle_filter')
    pf = ParticleFilter()
    rospy.spin()
