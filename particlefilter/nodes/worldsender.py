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


if __name__ == '__main__':
    rospy.init_node('particle_filter')
    tfb = tf.TransformBroadcaster() #create broadcaster
    print "tf send"
    tfb.sendTransform((0,0,0), 
                      (0,0,0,0),
                       rospy.Time.now(),
                          "map",
                          "world")
        rospy.spin()
