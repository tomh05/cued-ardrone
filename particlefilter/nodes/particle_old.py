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

mapsize = 20

class Particle:
    def __init__(self,_name,time):
        self.name=_name
        #self.position.x = random.random() * mapsize
        #self.position.y = random.random() * mapsize
        #self.position.z = random.random() * mapsize


        self.pos = Vector3Stamped()
        self.pos.vector=(random.random() * mapsize,
                         random.random() * mapsize,
                         random.random() * mapsize)
        self.alpha = random.random() * 2*math.pi
        self.beta = random.random() * 2*math.pi
        self.gamma = random.random() * 2*math.pi
        self.orientation = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)
        
        self.noise = 0.0
        self.tfb = tf.TransformBroadcaster()

        #self.tfl = tf.TransformListener()
        
        
        self.tfb.sendTransform(self.pos.vector,
                                self.orientation,
                                time,
                                "drone_particle_"+str(self.name),
                                "world")

        self.tfb.sendTransform(self.pos.vector,
                                self.orientation,
                                time,
                                "drone_particle_"+str(self.name),
                                "world")

        self.tfb.sendTransform(self.pos.vector,
                                self.orientation,
                                time,
                                "drone_particle_"+str(self.name),
                                "world")

    def set_noise(self,_noise):
        self.noise = _noise

    def move(self,trans,rot,time,listener):
        # move
        self.tfl = listener 
        deltapos = Vector3Stamped()
        deltapos_t = Vector3Stamped()
        deltapos.vector = trans
        #self.tfl.transformVector3("drone_particle_"+str(self.name),deltapos,deltapos_t)
        #listener.waitForTransform("/world", "/drone_particle_0", time, rospy.Duration(4.0))
        #listener.waitForTransform("/world","/drone_particle_"+str(self.name),time,rospy.Duration(0.5))
        #deltapos_t = listener.transformVector3("drone_particle_"+str(self.name),deltapos)

        rotmat = tf.transformations.euler_matrix(self.alpha,self.beta,self.gamma,'sxyz')
        print self.pos
        print rotmat
        self.pos = self.pos + rotmat*(trans + (1,))
        #self.pos = self.pos + deltapos_t
         
        self.orientation = rot


        self.tfb.sendTransform(self.pos.vector,
                                self.orientation,
                                time,
                                "drone_particle_"+str(self.name),
                                "world")
        return self

    def gauss(self, mu, sigma, x):
        return exp ( - ((mu-x)**2) / (sigma ** 2) / 2.0) / sqrt( 2.0 * pi * (sigma **2))

    def likelihood(self):
        pass
