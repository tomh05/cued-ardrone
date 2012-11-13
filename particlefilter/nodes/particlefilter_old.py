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

#---------------------------
# Map Setup
mapsize = 20



class ParticleFilter:
    def __init__(self):
        self.N          = 10    # number of particles
        self.p          = []      # particle list
        #transforms
        self.lastTime   = None
        self.listener = tf.TransformListener()
        


        self.br = tf.TransformBroadcaster()


        print 'sending'
        print self.br
        self.br.sendTransform((1.0,2.0,0.0),
                                tf.transformations.quaternion_from_euler(0.0,0.0,0.0),
                                rospy.Time(0),
                                "ardrone_base_link",
                                "world")

        print self.listener
        print self.listener.lookupTransform('/world','/ardrone_base_link',rospy.Time(0))

        self.createParticles()
        # compute weights each time pose data arrives
    	rospy.Subscriber('/ar_pose_marker',ARMarkers,self.markerCallback)
    	rospy.Subscriber('/xboxcontroller/button_a',Empty,self.markerCallback)


    def createParticles(self):
        for i in range(self.N):
            x = particle.Particle(i,rospy.Time.now())
            self.p.append(x)

    def markerCallback(self,markers):
        # update movement of particles
        # fetch delta deadreckon transform (movement relative to robot)

        (reltrans,relrot) = self.listener.lookupTransformFull("/dead_reckon",self.lastTime,
                                                   "/dead_reckon",markers.header.time)
        reltrans=(1,0,0)
        #(abstrans,absrot) = self.listener.lookupTransform("/world","/dead_reckon",markers.header.time)
        absrot=(0.5,0,0,0)
        for i in range(self.N):
            self.p[i]=self.p[i].move(reltrans,absrot,rospy.Time.now(),self.listener)
            #self.p[i]=self.p[i].move(reltrans,absrot,markers.header.time)

        # compute new weights
        print self.p[1].pos.vector

if __name__ == '__main__':
    rospy.init_node('particle_filter')
    pf = ParticleFilter()
    rospy.spin()
