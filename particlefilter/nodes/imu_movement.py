#!/usr/bin/env python  
import roslib
roslib.load_manifest('particlefilter')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np
from   std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from particlefilter.srv import *

class IMUHandle:
    def __init__(self):
        self.prevtime = None # time (float secs) of prev frame

        self.v_buffer = [] # buffer of last 10 frames [d.vx, d.vy]

        self.prev_z = 0.0
        self.prev_alpha = 0.0
        self.prev_beta = 0.0
        self.prev_gamma = 0.0

        self.rotZoffset = 0.0 # initial value of yaw to allow for offsets
        self.isFirstFrame= True # for resetting initial yaw value

        self.prevq = np.array([0.0,0.0,0.0,1.0])


    def handle_get_imu_movement(self,obj):
        navdata = obj.navdata
        self.rotZoffset = obj.gamma_offset
        # Running 10 sample median buffer (reduce noise on d.vx, d.vy)
        if len(self.v_buffer) >= 10:
            self.v_buffer.pop(0)
        self.v_buffer.append([navdata.vx, navdata.vy])
        vx = np.median(zip(*self.v_buffer)[0])
        vy = np.median(zip(*self.v_buffer)[1])

        #absT = TransformStamped()
        #absT.header=navdata.header
        absT = Vector3()
        relT = Vector3()
        
        response = IMUMovementResponse() 

        # get time difference between messages
        time = navdata.header.stamp
        if self.prevtime == None:
            self.prevtime = time
        deltat = (time - self.prevtime).to_sec()
        self.prevtime = time

        if (deltat<0): # rosbag looping
            self.isFirstFrame = True


        # translation
        relT.x = np.float64(vx*deltat / 1000.0)
        relT.y = np.float64(vy*deltat / 1000.0)
        relT.z = np.float64((navdata.altd / 1000.0) - self.prev_z)

        absT.y = 0.0
        absT.y = 0.0
        absT.z = np.float64(navdata.altd / 1000.0)

        # Euler angles in radians
        self.alpha = math.radians(navdata.rotX) # roll
        self.beta  = math.radians(navdata.rotY) # pitch
        self.gamma = math.radians(navdata.rotZ) - self.rotZoffset  # yaw

        if (self.isFirstFrame):
            self.prevYaw = self.gamma
            self.isFirstFrame = False 

        # produce quaternion
        q= tf.transformations.quaternion_from_euler(self.alpha,self.beta,self.gamma)
        qprime = tf.transformations.quaternion_multiply(q, tf.transformations.quaternion_inverse(self.prevq))

        self.prevq = q
        self.prev_z = navdata.altd / 1000.0


        response.absoluteTransform.header=navdata.header
        response.absoluteTransform.transform.translation = absT
        #response.absoluteTransform.transform.rotation = Quaternion(x=q[0], y=q[1], z = q[2], w = q[3])
        response.absoluteTransform.transform.rotation = Quaternion(x=qprime[0], y=qprime[1], z = qprime[2], w = qprime[3])

        response.relativeTranslation = relT
        

        self.prev_alpha = self.alpha
        self.prev_beta = self.beta
        self.prev_gamma = self.gamma
        return response

    def imu_movement_server(self):

        print "imu movement server running"
        rospy.init_node('imu_movement_server')
        s = rospy.Service('get_imu_movement', IMUMovement , self.handle_get_imu_movement)
        rospy.spin()

if __name__ == "__main__":
     imuh = IMUHandle()
     imuh.imu_movement_server()
