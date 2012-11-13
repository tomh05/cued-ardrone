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
        #self.position.x = random.random() * mapsize
        #self.position.y = random.random() * mapsize
        #self.position.z = random.random() * mapsize


        #self.pos = Vector3Stamped()
        #self.pos.vector=(random.random() * mapsize,
        #x=random.random() * mapsize - mapsize/2
        #y=random.random() * mapsize - mapsize/2
        #z=random.random() * mapsize
        x=0.0
        y=0.0
        z=0.0
        self.pos = np.array([x,y,z])
        self.prevtime = None


        #self.alpha = random.random() * 2*math.pi
        #self.beta = random.random() * 2*math.pi
        self.alpha = 0.0
        self.beta = 0.0
        #self.gamma = random.random() * 2*math.pi
        self.gamma = 0.0
        self.orientation = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)
        self.translationNoise = 0.0
        self.rotationNoise = 0.0
        self.markerNoise = 10.0

    def set_noise(self,_noise):
        self.noise = _noise

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
        nx = random.gauss(0,self.translationNoise)
        ny = random.gauss(0,self.translationNoise)
        nz = random.gauss(0,self.translationNoise)
        translationNoise = np.array([nx,ny,nz])

        nx = random.gauss(0,self.rotationNoise)
        ny = random.gauss(0,self.rotationNoise)
        nz = random.gauss(0,self.rotationNoise)
        nw = random.gauss(0,self.rotationNoise)
        np_quaternion = np.add(np_quaternion,[nx,ny,nz,nw])
        # quaternion_matrix() will renormalise np_quaternion for us

        # combination of current rotation and deadreckon        
        np_quaternionrot = tf.transformations.quaternion_multiply(np_quaternion,self.orientation)


        # perform position update
        rotationmat = tf.transformations.quaternion_matrix(np_quaternionrot)[:3,:3]
        self.pos = self.pos + np.dot(rotationmat,displacement) + translationNoise
        self.orientation=np_quaternionrot

        # return new particle
        newself = Particle()
        newself.pos = self.pos
        newself.orientation = self.orientation
        newself.prevtime = self.prevtime
        return newself

    def gauss(self, mu, sigma, x):
        return np.exp( - ((mu-x)**2) / (sigma ** 2) / 2.0) / np.sqrt( 2.0 * np.pi * (sigma **2))

    def likelihood(self,worldmap,ar_markers):
        #TODO use covariance data?
        # markers are currently relative to base. Transform to particle location

        weight = 1.0
        rotationmat = tf.transformations.quaternion_matrix(self.orientation)[:3,:3]

        for i in range(len(ar_markers.markers)):
            #print ar_markers.markers[i]
            # convert marker to numpy format
            t = ar_markers.markers[i].pose.pose.position
            displacement = np.array([t.x,t.y,t.z])
 
            q = ar_markers.markers[i].pose.pose.orientation
            np_quaternion = np.array([q.x,q.y,q.z,q.w])

            # compute marker position in world coordinates
            markerPos = self.pos + np.dot(rotationmat,displacement)

           #TODO subtract from camera to base_link manually! TODO 
            worldmark = []
            worldmark.append(3.0)
            worldmark.append(4.0)
            worldmark.append(0.0)
            conf = ar_markers.markers[i].confidence
            error2=0.0
            if (conf>70):
                #error2 = (worldmap[i].x-markerPos.x)**2 + (worldmap[i].y-markerPos.y)**2 + (worldmap[i].z-markerPos.z)**2
                error2 = (worldmark[0]-markerPos[0])**2 + (worldmark[1]-markerPos[1])**2 + (worldmark[2]-markerPos[2])**2
                #print error2
                #error2 = (markerPos[0])**2 + (markerPos[1])**2 + (markerPos[2])**2
                weight *= self.gauss(0,self.markerNoise,error2)
                #print str(error2) +" gave " +  str(weight)
                #weight = 1.0
        return weight

