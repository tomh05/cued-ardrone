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
    def __init__(self,_name,time):
        self.name=_name
        #self.position.x = random.random() * mapsize
        #self.position.y = random.random() * mapsize
        #self.position.z = random.random() * mapsize


        #self.pos = Vector3Stamped()
        #self.pos.vector=(random.random() * mapsize,
        x=random.random() * mapsize - mapsize/2
        y=random.random() * mapsize - mapsize/2
        z=random.random() * mapsize
        self.pos = np.array([x,y,z])

        #self.alpha = random.random() * 2*math.pi
        #self.beta = random.random() * 2*math.pi
        self.alpha = 0
        self.beta = 0
        self.gamma = random.random() * 2*math.pi
        self.orientation = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)
        self.noise = 0.0

    def set_noise(self,_noise):
        self.noise = _noise

    def move(self,ts):
        #convert to numpy format
        q = ts.transform.rotation
        np_quaternion = np.array([q.x,q.y,q.z,q.w])
        # combination of current rotation and deadreckon        
        np_quaternionrot = tf.transformations.quaternion_multiply(np_quaternion,
                                                               self.orientation)
        rotationmat = tf.transformations.quaternion_matrix(np_quaternion)[:3,:3]
        
        t = ts.transform.translation
        displacement = np.array([t.x,t.y,t.z])

        self.pos = self.pos + np.dot(rotationmat,displacement)
        self.orientation=np_quaternion

        return self

    def gauss(self, mu, sigma, x):
        return exp ( - ((mu-x)**2) / (sigma ** 2) / 2.0) / sqrt( 2.0 * pi * (sigma **2))

    def likelihood(self):
        pass
