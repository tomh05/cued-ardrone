#!/usr/bin/env python  
import roslib
roslib.load_manifest('particlefilter')
import rospy
import ardrone_autonomy.msg
import tf
import math
import random
import numpy as np
from   std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped


mapsize = 10

class Particle:
    def __init__(self):
        x=0.0
        y=0.0
        z=0.0
        self.pos = np.array([x,y,z])
        self.prevtime = None

        # for drawing debug pointcloud
        self.markerPos = np.array([0.0,0.0,0.0])

        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        self.orientation = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)

        self.translationNoise = 0.05
        self.rotationNoise = 0.01
        self.markerNoise = 10.0
        self.age = 0

    def move(self,ts):
        # rosbag: reset if time starts again
        time = ts.header.stamp
        if self.prevtime == None:
            self.prevtime = time 
        deltat = (time - self.prevtime).to_sec()
        self.prevtime = time
        if (deltat < 0): # rosbag looping
            self.pos = np.array([0.0,0.0,0.0])
            self.orientation = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)

        # convert translation to numpy format
        t = ts.transform.translation
        displacement = np.array([t.x,t.y,t.z])

        #convert rotation quaternion to numpy format
        q = ts.transform.rotation
        np_quaternion = np.array([q.x,q.y,q.z,q.w])

        # add noise
        nx = random.gauss(0.0,self.translationNoise)
        ny = random.gauss(0.0,self.translationNoise)
        nz = random.gauss(0.0,self.translationNoise)
        translationNoise = np.array([nx,ny,nz])

        nx = random.gauss(0.0,self.rotationNoise)
        ny = random.gauss(0.0,self.rotationNoise)
        nz = random.gauss(0.0,self.rotationNoise)
        nw = random.gauss(0.0,self.rotationNoise)
        np_quaternion = np.add(np_quaternion,[nx,ny,nz,nw])
        # quaternion_matrix() will renormalise np_quaternion for us

        # combination of current rotation and deadreckon        
        np_quaternionrot = tf.transformations.quaternion_multiply(np_quaternion,self.orientation)


        # perform position update
        rotationmat = tf.transformations.quaternion_matrix(np_quaternionrot)[:3,:3]
        self.pos = self.pos + np.dot(rotationmat,displacement) + translationNoise
        self.age+=1

    def clone(self):
        # return new particle at the current position
        newself = Particle()
        newself.pos = self.pos
        # this causes it to spread!

        '''
        for i in range(2):
            print "pos"
        print newself.pos
        print newself.pos
        print newself.pos
        print self.pos
        print self.pos
        print self.pos
        '''
        newself.orientation = self.orientation
        newself.prevtime = self.prevtime
        return newself

    def gauss(self, mu, sigma, x):
        return np.exp( - ((mu-x)**2) / (sigma ** 2) / 2.0) / np.sqrt( 2.0 * np.pi * (sigma **2))

    def likelihood(self,worldmap,ar_markers):
        #TODO use covariance data?
        weight = 1.0

        # markers are currently relative to base. Transform to particle location
        rotationmat = tf.transformations.quaternion_matrix(self.orientation)[:3,:3]

        for i in range(len(ar_markers.markers)):
            # convert marker to numpy format
            t = ar_markers.markers[i].pose.pose.position
            displacement = np.array([t.x,t.y,t.z])
            # adjust for front camera
            #displacement += np.array([0.0,0.0,0.21])

            q = ar_markers.markers[i].pose.pose.orientation
            np_quaternion = np.array([q.x,q.y,q.z,q.w])

            # compute marker position in world coordinates
            self.markerPos = self.pos + np.dot(rotationmat,displacement)

            #TODO subtract from camera to base_link manually! 
            marker = None
            conf = 0
            for j in range(len(worldmap.markers)):
                
                if (worldmap.markers[j].id == ar_markers.markers[i].id):
                    mapPosition = worldmap.markers[j].pose.pose.position
                    mapconf = worldmap.markers[j].confidence
                    break
            conf = ar_markers.markers[i].confidence
            error2=0.0
            if (conf>70):
                #error2 = (worldmap[i].x-self.markerPos.x)**2 + (worldmap[i].y-self.markerPos.y)**2 + (worldmap[i].z-self.markerPos.z)**2
                #error2 = (worldmark[0]-self.markerPos[0])**2 + (worldmark[1]-self.markerPos[1])**2 + (worldmark[2]-self.markerPos[2])**2
                error2 = (mapPosition.x-self.markerPos[0])**2 + (mapPosition.y-self.markerPos[1])**2 + (mapPosition.z-self.markerPos[2])**2
                #print error2
                #error2 = (self.markerPos[0])**2 + (self.markerPos[1])**2 + (self.markerPos[2])**2
                weight *= self.gauss(0,self.markerNoise,error2)
                #print str(error2) +" gave " +  str(weight)
                #weight = 1.0
        return weight

