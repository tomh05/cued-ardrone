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
from geometry_msgs.msg import TransformStamped
from particlefilter.srv import *

class IMUHandle:
    def __init__(self):
        self.prevtime = None # time (float secs) of prev frame

        self.v_buffer = [] # buffer of last 10 frames [d.vx, d.vy]

        self.prev_z = 0.0
        self.prev_alpha = 0.0
        self.prev_beta = 0.0
        self.prev_gamma = 0.0
        #self.rotZoffset = 0 # initial value of yaw to allow for offsets
        self.isFirstFrame= True # for resetting initial yaw value


    def handle_get_imu_movement(self,obj):
        navdata = obj.navdata

        # Running 10 sample median buffer (reduce noise on d.vx, d.vy)
        if len(self.v_buffer) >= 10:
            self.v_buffer.pop(0)
        self.v_buffer.append([navdata.vx, navdata.vy])
        vx = np.median(zip(*self.v_buffer)[0])
        vy = np.median(zip(*self.v_buffer)[1])

        # create transform and copy over time data
        t = TransformStamped()
        t.header=navdata.header

        # get time difference between messages
        time = navdata.header.stamp
        if self.prevtime == None:
            self.prevtime = time
        deltat = (time - self.prevtime).to_sec()
        self.prevtime = time

        if (deltat<0): # rosbag looping
            self.isFirstFrame = True


        # translation
        t.transform.translation.x = np.float64( vx*deltat / 1000.0)
        t.transform.translation.y = np.float64( vy*deltat / 1000.0)
        t.transform.translation.z = np.float64( (navdata.altd - self.prev_z) / 1000.0)
        self.prev_z = navdata.altd

        # Euler angles in radians
        self.alpha = math.radians(navdata.rotX) # roll
        self.beta  = math.radians(navdata.rotY) # pitch
        self.gamma = math.radians(navdata.rotZ) # yaw

        if (self.isFirstFrame):
            self.prevYaw = self.gamma
            self.isFirstFrame = False 

        # produce quaternion
        da = self.alpha - self.prev_alpha
        db = self.beta - self.prev_beta
        dg = self.gamma - self.prev_gamma
        q= tf.transformations.quaternion_from_euler(da,db,dg)

        self.prev_alpha = self.alpha
        self.prev_beta = self.beta
        self.prev_gamma = self.gamma

        #t.transform.translation = (x,y,z)
        t.transform.rotation = q

        response = IMUMovementResponse() 

        #This is ugly, but cannot find nicer way to inject response.
        # maybe quaternion is in wrong format, should be float64?
        response.transform.header=t.header
        response.transform.transform.translation = t.transform.translation
        #convert numpyarray to xyzw object?
        response.transform.transform.rotation.x=q[0] 
        response.transform.transform.rotation.y=q[1] 
        response.transform.transform.rotation.z=q[2] 
        response.transform.transform.rotation.w=q[3] 
        #response.transform = t
        return response

    def imu_movement_server(self):

        print "imu movement server running"
        rospy.init_node('imu_movement_server')
        s = rospy.Service('get_imu_movement', IMUMovement , self.handle_get_imu_movement)
        rospy.spin()

if __name__ == "__main__":
     imuh = IMUHandle()
     imuh.imu_movement_server()
