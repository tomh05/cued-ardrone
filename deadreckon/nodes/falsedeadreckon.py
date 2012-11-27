#!/usr/bin/env python  
import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np
from   std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class DeadReckoning:
    def __init__(self):
        self.state = 0
        self.frameskip = 0
        self.prevtime = None



    def navdataCallback(self, d):
        # Skip frames. Need to add ROS parameter to allow setting        
        self.frameskip += 1
        if self.frameskip < 11:
            return            
        self.frameskip = 0
        '''
        # Create rotation quaternion
        '''
        # Euler angles in radians
        self.alpha = 0.
        self.beta  = 0.
        if self.state == 0:
            self.gamma = 0.
        else:
            self.gamma = -np.pi/2.
            
        # produce quaternion
        quaternion = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)
        
        #print "bearing " + str(alpha) + ", " + str(beta)+", "+str(gamma)
        
        
        
        '''
        # Sort out frame timings
        '''
        time = d.header.stamp
        if self.prevtime == None:
            self.prevtime = time
        deltat = (time - self.prevtime).to_sec()
        self.prevtime = time
 
        
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
        
        if self.state == 0:
            self.x = 0.
            self.y = 1.
            self.state = 1
        else:
            self.x = 1.
            self.y = 1.
            self.state = 0
            
        
        self.z = 1.

        print (self.x, self.y, self.z)
        print tf.transformations.euler_from_quaternion(quaternion)
        '''
        # Publish tf
        '''
        br = tf.TransformBroadcaster() #create broadcaster
        br.sendTransform((self.x,self.y,self.z), 
                         # translation happens first, then rotation
                         quaternion,
                         time,
                         "ardrone_base_link",
                         "world")


if __name__ == '__main__':
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    rospy.init_node('deadreckon_broadcaster')
    rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,dead_reckon.navdataCallback)
    rospy.spin()
