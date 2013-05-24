#!/usr/bin/env python

'''
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import Empty
import tf
from time import time, sleep
import sys
import numpy as np

from positionControl import PositionController

class Maze:

    def __init__(self):
        self.covered = numpy.zeros((widthx,heighty,4))
        self.currentPos = numpy.zeros(3)
        self.tfListener = tf.TransformListener();

    def getCurrentGridPosition(self)
        (self.currentPos,self.currentRot) = self.tfListener.lookupTransform('/world','/ardrone_base_link_control',rospy.Time(0)) # get latest transform

    def logPoint(self):
        positionx = 
        positiony = 
        direction =
        self.covered(positionx,positiony,direction) += 1.0;


    def getNewDestination(self):
        weightedmap = self.covered
        for...
            weightedmap[i,j,k] += (i-self.currentposition) + (j-)
        newDest = min()



if __name__ == '__main__':
        rospy.init_node('maze')
        maze = Maze()
        rospy.sleep(1.0)
        droneMovement.positionController.pc_timer_init()
        print 'created'
        #rospy.Subscriber('/targetPose',Pose,droneMovement.goToWorldPose);
        rospy.Subscriber('/targetPose',Pose,droneMovement.setK);
        rospy.spin()
