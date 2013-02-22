#!/usr/bin/env python

'''
HoverController.py
This program performs high level control of marker hovering: 
sets up transform trees, defines world coordinate, works out current pose and sets desired pose, and sends these to position_controller
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
import tf
from time import time, sleep
import sys

from positionControl import PositionController

class PosRequester:

    def __init__(self):
        self.targetpub = rospy.Publisher('/targetPose', Pose)
         


    def resetTwist(self):
        # TODO decide if useful, and implement actual twist sending
        self.twist.linear  = {'x':0.0,'y':0.0,'z':0.0}
        self.twist.angular = {'x':0.0,'y':0.0,'z':0.0}

    def broadcastPose(self, targetPose):
        print 'Publishing...'
        print targetPose
        self.targetpub.publish(targetPose)

    def askPose(self):
        while (True):
            targetPose = Pose()
            targetPose.position.x = float(raw_input('x? '))
            targetPose.position.y = float(raw_input('y? '))
            targetPose.position.z = float(raw_input('z? '))
            self.broadcastPose(targetPose)


if __name__ == '__main__':
        rospy.init_node('posRequester')
        posRequester = PosRequester()
        #rospy.Subscriber('/targetPose',Pose,droneMovement.goToWorldPose);
        posRequester.askPose()
        #rospy.Subscriber('/tf',tf.msg.tfMessage,droneMovement.updatePose);
        rospy.spin()
