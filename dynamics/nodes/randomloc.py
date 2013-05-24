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

class Positioner:

    def __init__(self):
        self.cmdpub = rospy.Publisher('cmd_vel', Twist)
        self.landpub = rospy.Publisher('ardrone/land', Empty)
        self.resetpub = rospy.Publisher('ardrone/reset', Empty)
        self.takeoffpub = rospy.Publisher('ardrone/takeoff', Empty)
        self.targetpub = rospy.Publisher('/move_base_simple/goal', PoseStamped)


    def newPosition(self,d):
        newPos = PoseStamped()
        rospy.sleep(3.0)
        newPos.pose.position.x = np.random.uniform(-1.5,1.0)
        newPos.pose.position.y = np.random.uniform(-1.5,1.0)
        newPos.pose.position.z = 1.0
        
        direction=3.14/2
        direction=np.pi/2 + np.random.uniform(-0.5,0.5)
        quat = tf.transformations.quaternion_from_euler(0.0,0.0,direction)
        newPos.pose.orientation.x = quat[0]
        newPos.pose.orientation.y = quat[1]
        newPos.pose.orientation.z = quat[2]
        newPos.pose.orientation.w = quat[3]
        print 'new pose', newPos
        self.targetpub.publish(newPos)


if __name__ == '__main__':
        rospy.init_node('maze')
        p = Positioner()
        rospy.sleep(1.0)

        p.arrivedsub = rospy.Subscriber('/pumba/targetreached',Empty,p.newPosition);        
        p.arrivedsub = rospy.Subscriber('/xboxcontroller/button_y',Empty,p.newPosition);        
        print 'created'
        #rospy.Subscriber('/targetPose',Pose,droneMovement.goToWorldPose);
        rospy.spin()
