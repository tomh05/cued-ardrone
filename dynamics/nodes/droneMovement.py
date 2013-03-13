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

from positionControl import PositionController

class DroneMovement:

    def __init__(self):
        self.twist = Twist()
        self.cmdpub = rospy.Publisher('cmd_vel', Twist)
        self.landpub = rospy.Publisher('/ardrone/land', Empty)
        self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
        self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
         
        self.currentPos = (0.0, 0.0, 0.0)
        self.currentYaw =0.0
        self.targetPos = (0.0,0.0,1.0) #self.currentPos
        self.targetYaw = self.currentYaw
        
        self.positionController = PositionController(self.targetPos,self.targetYaw,self.currentPos,self.currentYaw)
        self.positionController.refalon = False # compare current and target z. 
        self.positionController.yawon = True # TODO enable yaw control once gyros fixed



    def resetTwist(self):
        # TODO decide if useful, and implement actual twist sending
        self.twist.linear  = {'x':0.0,'y':0.0,'z':0.0}
        self.twist.angular = {'x':0.0,'y':0.0,'z':0.0}

    def goToWorldPose(self, targetPose):
        self.targetPos = (targetPose.position.x, targetPose.position.y, targetPose.position.z)
        targetQuat = (targetPose.orientation.x,
                      targetPose.orientation.y,
                      targetPose.orientation.z,
                      targetPose.orientation.w)
        targetEuler = tf.transformations.euler_from_quaternion(targetQuat)
        self.targetYaw = targetEuler[2]
        print 'setting pose to:'
        self.positionController.dpw_handler(self.targetPos)
        print 'yaw is now'
        print self.targetYaw
        self.positionController.dyw_handler(self.targetYaw)

        
    def goToPoseStamped(self,targetPoseStamped):
        targetPoseStamped.pose.position.z = 1.0 # it's zero by default
        self.goToWorldPose(targetPoseStamped.pose) 
    
    #crazy debug data format 
    def setK(self,targetPose):
        self.positionController.kParticle    = targetPose.position.x
        self.positionController.kParticleRot = targetPose.position.y

    def updatePose(self,data):
        ###### Currently un-used: position is determined by the positionController directly

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
        rospy.sleep(1.0)
        droneMovement.positionController.pc_timer_init()
        print 'created'
        #rospy.Subscriber('/targetPose',Pose,droneMovement.goToWorldPose);
        rospy.Subscriber('/targetPose',Pose,droneMovement.setK);
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,droneMovement.goToPoseStamped);
        rospy.Subscriber('/tf',tf.msg.tfMessage,droneMovement.updatePose);
        rospy.spin()
