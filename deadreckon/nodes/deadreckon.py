#!/usr/bin/env python  
import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class DeadReckoning:
    def __init__(self):
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world
        self.prevtime = None # time (float secs) of prev frame
        self.v_buffer = [] # buffer of last 10 frames [d.vx, d.vy]
        self.prev_vx_rot = 0. # previous x vel for trapezium rule
        self.prev_vy_rot = 0. # previous y vel for trapezium rule
        self.init_vx_rot = None # initial x vel for trapezium rule
        self.init_vy_rot = None # initial y vel for trapezium rule
        self.rotZoffset = 0 # initial value of yaw to allow for offsets
        self.trackPath = Path()
        self.frontcam_quat = tf.transformations.quaternion_from_euler(-90.0 * (np.pi/180.), 0.0, -90.0 * (np.pi/180.))
        self.frontcam_t = (0.21, 0.0, 0.0)
        self.br = tf.TransformBroadcaster() #create broadcaster
        self.pathPub = rospy.Publisher('deadreckon_path',Path)
        self.prev_tm = None
        
    def reset(self,d):
        print "resetting"
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world
        self.prevtime = None
        self.v_buffer = []
        self.prev_vx_rot = 0. # previous x vel for trapezium rule
        self.prev_vy_rot = 0. # previous y vel for trapezium rule
        self.init_vx_rot = None # initial x vel for trapezium rule
        self.init_vy_rot = None # initial y vel for trapezium rule
        self.rotZoffset = self.gamma 
        self.trackPath = Path()
        self.prev_tm = None

    def navdataCallback(self, d):
        
        if d.batteryPercent < 25:
            print "Warning: Battery at ", d.batteryPercent, "%"
            
        
       
       
        
        '''
        # Running 10 sample median buffer (reduce noise on d.vx, d.vy)
        '''
        #if len(self.v_buffer) < 10:
        #    self.v_buffer.append([d.vx, d.vy])
        #    vx = np.median(zip(*self.v_buffer)[0])
        #    vy = np.median(zip(*self.v_buffer)[1])
        #else:
        #    self.v_buffer.pop(0)
        #    self.v_buffer.append([d.vx, d.vy])
        #    vx = np.median(zip(*self.v_buffer)[0])
        #    vy = np.median(zip(*self.v_buffer)[1])
        vx = d.vx 
        vy = d.vy 
        #print str(d.vx) + "," + str(d.vy) + "," + str(d.vz)
        
        
        
        '''
        # Create rotation quaternion
        '''
        # Euler angles in radians
        self.alpha = math.radians(d.rotX) # roll
        self.beta  = math.radians(d.rotY) # pitch
        self.gamma = math.radians(d.rotZ) # yaw
        # produce quaternion
        quaternion = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma - self.rotZoffset)
        
        #print "bearing " + str(alpha) + ", " + str(beta)+", "+str(gamma)
        
        
        
        '''
        # Sort out frame timings
        '''
        time = d.header.stamp
        if self.prevtime == None:
            self.reset(self)
            self.prevtime = time
            self.prev_tm = d.tm
        #print "comparison"
        deltat = (time - self.prevtime).to_sec()
        #print deltat
        #deltat = (float(d.tm-self.prev_tm))/1000000.
        #if deltat > 0.1:
        #    deltat = self.prev_deltat
        deltat = (time - self.prevtime).to_sec()
        self.prev_deltat = deltat
        #print deltat
        self.prevtime = time
        self.prev_tm = d.tm
        # if playing rosbags and time jumps backwards, we want to reset
        if (deltat < -1.0): 
            self.reset(self)
        
        # NOTE: at 50Hz 10median filter is better than none and header times are better than tm
        
        '''
        # Resolve into co-ordinate system
        # vx and vy need to be resolved back by these to world axis
        '''
        
        
        # Old method: Produce reverse rotation matrix
        #origin, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
        #Rx = tf.transformations.rotation_matrix(-alpha, xaxis)
        #Ry = tf.transformations.rotation_matrix(-beta, yaxis)
        #Rz = tf.transformations.rotation_matrix(-gamma, zaxis)
        #R = tf.transformations.concatenate_matrices(Rx, Ry, Rz)
        #R = R[:3, :3] # Is homonogenous ???
        
        # vectorize
        #vel_vector = np.asmatrix([vx, vy, 0.]).btT
        #print vel_vector
        #quaternion_inverse = tf.transformations.quaternion_inverse(quaternion)
        #dir = tf.transformations.quaternion_matrix(quaternion_inverse)

        corr_angle = self.gamma - self.rotZoffset 
        vx_rot = math.cos(corr_angle)*vx - math.sin(corr_angle)*vy
        vy_rot = math.sin(corr_angle)*vx + math.cos(corr_angle)*vy
        
        """
        '''
        # Trapezium rule iterative implementation
        # I(n+1) = I(n) + (h/2)(y(n)+y(n+1))
        #
        # This is going to make ~no difference to overall
        # Do not bother using
        '''
        if (self.init_vx_rot == None):
            init_vx_rot = vx_rot
            init_vy_rot = vy_rot
            self.x += vx_rot*deltat / 1000
            self.y += vy_rot*deltat / 1000
        else:
            self.x = (deltat/2000)(self.x+self.prev_vx_rot+vx_rot) 
            self.y = (deltat/2000)(self.y+self.prev_vy_rot+vy_rot)
            self.prev_vx_rot = vx_rot
            self.prev_vy_rot = vy_rot
        """
        
        '''
        # Summation
        '''
        self.x += vx_rot*deltat / 1000
        self.y += vy_rot*deltat / 1000

        # altd is an >>>int<<< in mm for drone altitude relative to some initialisation (not elevation from ground)
        # *1. is to force float for division
        self.z = (d.altd*1.) / 1000

        '''
        # Publish tf
        '''
        
        self.br.sendTransform((self.x,self.y,self.z), 
                         # translation happens first, then rotation
                         quaternion,
                         time,
                         "/ardrone_base_link",
                         "/world")
                         
        # (values pulled from ardrone_drive.cpp)
        # We need to publish this when using high rate navdata to prevent
        # front camera tf from lagging base ardrone
        self.br.sendTransform(self.frontcam_t, self.frontcam_quat ,time, '/ardrone_base_frontcam', '/ardrone_base_link')
        # NOTE: child parent order is reversed w.r.t C++


        '''
        # Publish path visualisation data
        '''

        self.trackPath.header        = d.header

        # copy headers over from Navdata
        self.trackPath.header        = d.header
        newPose = PoseStamped()
        newPose.header               = d.header
        newPose.header.frame_id      = '/world'

        newPose.pose.position.x      = self.x
        newPose.pose.position.y      = self.y
        newPose.pose.position.z      = self.z
        newPose.pose.orientation.x   = self.alpha  
        newPose.pose.orientation.y   = self.beta  
        newPose.pose.orientation.z   = self.gamma
        
        self.trackPath.poses.append(newPose)

        
        self.pathPub.publish(self.trackPath)


if __name__ == '__main__':
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    rospy.init_node('deadreckon_broadcaster')
    rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,dead_reckon.navdataCallback)
    rospy.Subscriber('/xboxcontroller/button_start',Empty,dead_reckon.reset)
    rospy.spin()
