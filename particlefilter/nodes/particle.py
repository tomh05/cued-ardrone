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


class Particle:
    def __init__(self):
        x=0.0+ (random.random() * 10.0)
        y=0.0+ (random.random() * 10.0)
        z=0.0
        self.pos = np.array([x,y,z])
        self.prevtime = None

        # for drawing debug pointcloud
        self.markerPos = np.array([0.0,0.0,0.0])

        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        self.orientation = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)

        self.translationNoise       = 0.0 # 0.02
        self.rotationNoise          = 0.0 # 0.1
        self.markerNoise            = 0.0# 0.2 #10.0

    def setNoise(self,translationNoise,rotationNoise,markerNoise):
            self.translationNoise   = translationNoise
            self.rotationNoise      = rotationNoise
            self.markerNoise        = markerNoise

    def move(self,movement):
        # rosbag: reset if time starts again
        time = movement.absoluteTransform.header.stamp
        if self.prevtime == None:
            self.prevtime = time 
        deltat = (time - self.prevtime).to_sec()
        self.prevtime = time
        if (deltat < 0): # rosbag looping
            x=0.0+ (random.random() * 10.0)
            y=0.0+ (random.random() * 10.0)
            z=0.0
            self.pos = np.array([x,y,z])
            self.orientation = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)

        # convert translation to numpy format
        t = movement.relativeTranslation
        displacement = np.array([t.x,t.y,t.z])

        #convert rotation quaternion to numpy format
        q = movement.absoluteTransform.transform.rotation
        np_quaternion = np.array([q.x,q.y,q.z,q.w])
        self.orientation = np_quaternion
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
        #np_quaternionrot = tf.transformations.quaternion_multiply(np_quaternion,self.orientation)


        # perform position update
        rotationmat = tf.transformations.quaternion_matrix(np_quaternion)[:3,:3]
        self.pos = self.pos + np.dot(rotationmat,displacement) + translationNoise
        # absolute altitude
        self.pos[2] = movement.absoluteTransform.transform.translation.z
        self.pos = self.pos + translationNoise

    def clone(self):
        # return new particle at the current position
        newself = Particle()
        newself.pos = self.pos
        newself.orientation = self.orientation
        newself.prevtime = self.prevtime
        return newself

    def gauss(self, mu, sigma, x):
        return np.exp( - ((mu-x)**2) / (sigma ** 2) / 2.0) / np.sqrt( 2.0 * np.pi * (sigma **2))

    def likelihood(self,worldmap,ar_markers):
        #TODO use covariance data?
        weight = 1.0

        # markers are currently relative to base. Set up transform to particle base location...
        rot_mat_world_to_base = tf.transformations.quaternion_matrix(self.orientation)[:3,:3]
        # ...and from particle to front camera frame
        q_base_frontcam = np.array([-0.5,0.5,-0.5,0.5])
        rot_mat_base_to_frontcam = tf.transformations.quaternion_matrix(q_base_frontcam)[:3,:3]

        for i in range(len(ar_markers.markers)):
            # convert marker position to numpy format
            t = ar_markers.markers[i].pose.pose.position
            displacement = np.array([t.x,t.y,t.z])
            q = ar_markers.markers[i].pose.pose.orientation
            np_quaternion = np.array([q.x,q.y,q.z,q.w])

            # if marker is behind camera, throw it out
            if (displacement[2]<0):
                continue

            # transform marker from front_camera to base_link
            displacement += np.array([0.0,0.0,0.21])
            displacement = np.dot(rot_mat_base_to_frontcam,displacement) 

            # compute marker position in world coordinates (transform base_link to /world)
            self.markerPos = self.pos + np.dot(rot_mat_world_to_base,displacement)

            # search the map for a marker with matching ID  
            mapPosition = None
            for j in range(len(worldmap.markers)):
                if (worldmap.markers[j].id == ar_markers.markers[i].id):
                    mapPosition = worldmap.markers[j].pose.pose.position
                    #print "I'm looking at marker " + str(worldmap.markers[j].id)
                    break

            conf = ar_markers.markers[i].confidence

            if (mapPosition is not None and conf>70):
                error2 = (mapPosition.x-self.markerPos[0])**2 + (mapPosition.y-self.markerPos[1])**2 + (mapPosition.z-self.markerPos[2])**2
                #weight *= self.gauss(0,self.markerNoise,error2) #/ (1.01-0.01*conf)
                #print error2
                weight *= 1.0/error2
                #weight = 1.0
        return weight

