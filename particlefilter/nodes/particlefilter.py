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
from geometry_msgs.msg import PoseStamped, Point32, Point, Pose

from visualization_msgs.msg import MarkerArray, Marker


#------------------------------------------------------
# debug setup

# draws pointcloud if false, or arrows if true. 
# use  less than 20 particles for arrows
useArrows = True
publishTf = True

#------------------------------------------------------
# Params setup
xyNoise     = rospy.get_param("Particle_Filter/xy_noise",0.0)
altNoise    = rospy.get_param("Particle_Filter/altitude_noise",0.0)
pitchNoise  = rospy.get_param("Particle_Filter/pitch_noise",0.0)
yawNoise    = rospy.get_param("Particle_Filter/yaw_noise",0.0)
rollNoise   = rospy.get_param("Particle_Filter/roll_noise",0.0)
markerNoiseP= rospy.get_param("Particle_Filter/marker_noise_p",0.0)
markerNoiseR= rospy.get_param("Particle_Filter/marker_noise_r",0.0)

#------------------------------------------------------
# Map Setup
#mapfile = open("../maps/singlemarker.map","r")
#mapfile = open("../maps/multimarker.map","r")
map_path = rospy.get_param("Particle_Filter/map_path")
mapfile = open(map_path,'r')

worldmap = pickle.load(mapfile)
gamma_offset = pickle.load(mapfile)
print gamma_offset

mapVisualisation = MarkerArray()
#for i in range(len(worldmap.markers)):
for i in range(len(worldmap.markers)):
    marker = Marker()
    marker.header.frame_id = '/world'
    marker.type = marker.CUBE
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

# draw room
marker = Marker()
marker.header.frame_id = '/world'
marker.type = marker.CUBE
marker.action = marker.ADD
marker.scale.x = 6.3
marker.scale.y = 5.25
marker.scale.z = 2.8
marker.color.a = 0.4
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 1.0
marker.id=99
marker.pose.position.x = 0.1
marker.pose.position.y = -0.175
marker.pose.position.z = 1.4
mapVisualisation.markers.append(marker)


#------------------------------------------------------

class ParticleFilter:
    def __init__(self):
        self.N            = 7      # number of particles 20 was best
        self.p            = []      # particle list
        self.ar_markers   = None 
        self.w            = [1.0]*self.N
        self.maxWparticle = 0
        self.gamma_offset = 0.0 # gamma_offset TODO set a convention
        self.gamma_offset = gamma_offset #TODO set a convention
        # Callbacks may overlap: make sure they don't by locking thread
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
            self.markerEstimatesPub = rospy.Publisher('particle_estimates',MarkerArray)
        else:
            self.pclPub = rospy.Publisher('point_cloud',PointCloud)
        self.pclBPub = rospy.Publisher('point_cloudB',PointCloud)
        self.mapMarkerPub = rospy.Publisher('map_markers',MarkerArray)

        # move particles each time navigation data arrives
        rospy.Subscriber('ardrone/navdata',ardrone_autonomy.msg.Navdata,self.navdataCallback)


        # reset gamma
        rospy.Subscriber('/xboxcontroller/button_a',Empty,self.resetgammaCallback)

        # and compute weights each time pose data arrives
    	rospy.Subscriber('ar_pose_marker',ARMarkers,self.markerCallback,None,1) # Queue size 1

        self.createParticles()

    def resetgammaCallback(self,d):
        self.gamma_offset = 0.0 #TODO fix convention
        self.gamma_offset = math.radians(self.navdata.rotZ) #TODO fix convention
        for i in range(self.N):
            #self.p[i].pos = np.array([0.0,0.0,0.0])
            self.p[i].pos = np.array([-5.0 + (random.random() * 10.0),-5.0 + (random.random() * 10.0),random.random()])
            self.p[i].orientation = tf.transformations.quaternion_from_euler(-3.14 + 6.28*random.random(), -3.14 + 6.28*random.random(), -3.14 + 6.28*random.random())
        rospy.loginfo("gamma offset is now " + str(self.gamma_offset))

    def createParticles(self):
        print "creating particles"
        for i in range(self.N):
            x = particle.Particle()
            x.setNoise(xyNoise,altNoise,rollNoise,pitchNoise,yawNoise,markerNoiseP,markerNoiseR)
            self.p.append(x)

    def navdataCallback(self,navdata):
        self.lock.acquire()
        #find change in position
        self.navdata = navdata
        movement = self.getIMUMovement(navdata=self.navdata, gamma_offset=self.gamma_offset)

        for i in range(self.N):
            # update particle locations
            self.p[i].move(movement)

        #######################self.visualiseParticles()
        if (self.ar_markers is not None ):
            #weigh and resample
            # only do this for marker data that is older than the current deadreckon data - if deadreckoning is behind, skip the
            # marker data and catch up on position
            if (len(self.ar_markers.markers)>0 and cmp(self.ar_markers.markers[0].header.stamp,navdata.header.stamp)<0):
                self.findWeights(self.ar_markers)
                #print 'now',rospy.Time.now() 
                #print 'mark',self.ar_markers.markers[0].header
                #print 'nav',navdata.header
                firstmarker = self.ar_markers.markers[0].id
                # debug: if the markers were used, draw estimates
                if self.w[0]!=1.0:
                    # flash marker red
                    mapVisualisation.markers[firstmarker].color.g = 0.0
                    self.mapMarkerPub.publish(mapVisualisation)
                    ######################self.visualiseEstimates()
                    self.resample()
                    mapVisualisation.markers[firstmarker].color.g = 1.0
        self.ar_markers = None
        self.mapMarkerPub.publish(mapVisualisation)
        self.broadcastTf(self.navdata.header.stamp,self.maxWparticle)
        self.lock.release()
       
    def visualiseParticles(self):
        if (useArrows):
            markerArray = MarkerArray()
            #for i in range(self.N):
            for i in range(4):

                #print self.w[i]

                marker = Marker()
                marker.header.frame_id = '/world'
                marker.type = marker.ARROW
                marker.action = marker.ADD
                marker.scale.x = 0.1
                marker.scale.y = 3
                marker.scale.z = 0.6
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = self.w[i]*10
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
                marker = Marker()
                marker.header.frame_id = '/world'
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.id=i+self.N+1
                marker.pose.orientation.x = self.p[i].morientation[0]
                marker.pose.orientation.y = self.p[i].morientation[1]
                marker.pose.orientation.z = self.p[i].morientation[2]
                marker.pose.orientation.w = self.p[i].morientation[3]
                marker.pose.position.x =  self.p[i].markerPos[0]
                marker.pose.position.y =  self.p[i].markerPos[1] 
                marker.pose.position.z =  self.p[i].markerPos[2] 
                markerArray.markers.append(marker)
                '''
                '''
                marker = Marker()
                marker.header.frame_id = '/world'
                marker.type = marker.ARROW
                marker.action = marker.ADD
                marker.scale.x = 0.5
                marker.scale.y = 6
                marker.scale.z = 1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.id=i+2*self.N+5
                marker.pose.position    = worldmap.markers[0].pose.pose.position
                marker.pose.orientation = worldmap.markers[0].pose.pose.orientation
                markerArray.markers.append(marker)
                '''
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
                #if self.ar_markers:
                #    visualisationWeight =  self.w[i] #self.p[i].likelihood(worldmap,self.ar_markers)
                if len(self.w)>0:
                    visualisationWeight = self.w[i]
                    pc.channels[0].values.append(visualisationWeight)
                pc.points.append(newp)

            self.pclPub.publish(pc)

    def markerCallback(self,ar_markers):
        self.lock.acquire()
        # feed in new markers for processing on next navdata
        self.ar_markers = ar_markers 
        self.lock.release()
      
        
    def visualiseEstimates(self):
        if (useArrows):
            markerArray = MarkerArray()
            #for i in range(self.N):
            for i in range(4):

                marker = Marker()
                marker.header.frame_id = '/world'
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.id=i+self.N+1
                marker.pose.orientation.x = self.p[i].morientation[0]
                marker.pose.orientation.y = self.p[i].morientation[1]
                marker.pose.orientation.z = self.p[i].morientation[2]
                marker.pose.orientation.w = self.p[i].morientation[3]
                marker.pose.position.x =  self.p[i].markerPos[0]
                marker.pose.position.y =  self.p[i].markerPos[1] 
                marker.pose.position.z =  self.p[i].markerPos[2] 
                markerArray.markers.append(marker)

            self.markerEstimatesPub.publish(markerArray)
        else:       
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
            clonedParticle.setNoise(xyNoise,altNoise,rollNoise,pitchNoise,yawNoise,markerNoiseP,markerNoiseR)
            clonedParticle.pos[0] += random.gauss(0.0,0.05)
            clonedParticle.pos[1] += random.gauss(0.0,0.05)
            clonedParticle.orientation = clonedParticle.addRotNoise(clonedParticle.orientation,0.00,0.00,0.1)
            new_p.append(clonedParticle)
        self.p = new_p
        self.maxWparticle = self.w.index(mw)

    def broadcastTf(self,time,i):
        if publishTf:
            #meanPos = (0.0,0.0,0.0)
            meanPx = meanPy = meanPz = 0.0
            meanRx = meanRy = meanRz = 0.0
            meanRw = 0.0
            for i in range(self.N):
                #meanPos += self.p[i].pos
                meanPx += self.p[i].pos[0]
                meanPy += self.p[i].pos[1]
                meanPz += self.p[i].pos[2]
                meanRx += self.p[i].orientation[0]
                meanRy += self.p[i].orientation[1]
                meanRz += self.p[i].orientation[2]
                meanRw += self.p[i].orientation[3]

            meanPx /= self.N
            meanPy /= self.N
            meanPz /= self.N
            meanRx /= self.N
            meanRy /= self.N
            meanRz /= self.N
            meanRw /= self.N
            #quatSize = np.sqrt(meanRx**2 + meanRy**2 + meanRz**2 + meanRw**2)
            #meanRx /= quatSize
            #meanRy /= quatSize
            #meanRz /= quatSize
            #meanRw /= quatSize


            #rospy.loginfo("Transmitting at x=")
            #rospy.loginfo(self.p[i].pos)
            #q = 2
            self.br.sendTransform((meanPx,meanPy,meanPz),
                                  (meanRx,meanRy,meanRz,meanRw),
                                  #self.p[i].pos,
                                  # translation happens first, then rotation
                                  #(worldmap.markers[q].pose.pose.orientation.x,
                                  #worldmap.markers[q].pose.pose.orientation.y,
                                  #worldmap.markers[q].pose.pose.orientation.z,
                                  #worldmap.markers[q].pose.pose.orientation.w),
                                  #self.p[i].morientation,
                                  #(0.5,0.5,0.5,0.49),
                                  time, 
                                  "/"+rospy.get_param('tf_prefix')+"/ardrone_base_link_particle_filter",
                                  "/world")
    #    def bestGuess(self):
    #    for i in range(self.N):
    #        pos = np.array([particle.pos for particle in self.p)
    #        print "x: "
    #        print np.mean(pos[0])

if __name__ == '__main__':
    rospy.init_node('particle_filter')
    pf = ParticleFilter()
    rospy.spin()
