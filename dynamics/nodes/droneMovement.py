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

class DroneMovement:

    def __init__(self):
        self.twist = Twist()
        self.cmdpub = rospy.Publisher('cmd_vel', Twist)
        self.landpub = rospy.Publisher('/ardrone/land', Empty)
        self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
        self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
         
        self.currentPos = (0.0, 0.0, 1.5)
        self.currentYaw =0.0
        self.targetPos = (0.0,0.0,1.5) #self.currentPos
        self.targetYaw = self.currentYaw
        
        self.positionController = PositionController(self.targetPos,self.targetYaw,self.currentPos,self.currentYaw)
        self.positionController.refalon = True
        self.positionController.yawon = False # default settings from Rujian's code
        self.positionController.pc_timer_init()



    def resetTwist(self):
        # TODO decide if useful, and implement actual twist sending
        self.twist.linear  = {'x':0.0,'y':0.0,'z':0.0}
        self.twist.angular = {'x':0.0,'y':0.0,'z':0.0}

    def goToWorldPose(self, targetPose):
        self.targetPos = (targetPose.position.x, targetPose.position.y, targetPose.position.z)
        print 'setting pose to:'
        print targetPose.position
        self.positionController.dpw_handler(self.targetPos)
        #TODO rotation 
    
    def updatePose(self,data):

        if (data.transforms[0].child_frame_id == '/ardrone_base_link'):
            self.currentPos = (data.transforms[0].transform.translation.x,
                               data.transforms[0].transform.translation.y,
                               data.transforms[0].transform.translation.z)
            print self.currentPos
            #self.currentYaw = (data.transform.translation.x,
            #                   data.transform.translation.y,
            # TODO quaternion data.transform.translation.z)

            #self.positionController.cpw_cyw_handler(self.currentPos,self.currentYaw)

if __name__ == '__main__':
        rospy.init_node('movementHandler')
        droneMovement = DroneMovement()
        print 'created'
        rospy.Subscriber('/targetPose',Pose,droneMovement.goToWorldPose);
        rospy.Subscriber('/tf',tf.msg.tfMessage,droneMovement.updatePose);
        rospy.spin()
