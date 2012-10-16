#!/usr/bin/env python  
import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np

class DeadReckoning:
    def __init__(self):
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z locarion in world
        self.prevtime = None # time (float secs) of prev frame
        self.v_buffer = [] # buffer of last 10 frames [d.vx, d.vy]

    def navdataCallback(self, d):
        
        
        '''
        # Sort out frame timings
        '''
        time = rospy.get_time() # should actually pull drone timings instead
        if self.prevtime == None:
            self.prevtime = time
        deltat = time - self.prevtime
        self.prevtime = time
        
        
        
        
        '''
        # Running 10 sample median buffer (reduce noise on d.vx, d.vy)
        '''
        if len(self.v_buffer) < 10:
            self.v_buffer.append([d.vx, d.vy])
            vx = np.median(zip(*self.v_buffer)[0])
            vy = np.median(zip(*self.v_buffer)[1])
        else:
            self.v_buffer.pop(0)
            self.v_buffer.append([d.vx, d.vy])
            vx = np.median(zip(*self.v_buffer)[0])
            vy = np.median(zip(*self.v_buffer)[1])
    
        #print d.rotX
        #print str(d.vx) + "," + str(d.vy) + "," + str(d.vz)
        
        
        
        '''
        # Create rotation quaternion
        '''
        # Euler angles in radians
        alpha = d.rotX * math.pi / 180 # roll
        beta  = d.rotY * math.pi / 180 # pitch
        gamma = d.rotZ * math.pi / 180 # yaw
        # produce quaternion
        quaternion = tf.transformations.quaternion_from_euler(alpha, beta, gamma)
        
        #print "bearing " + str(alpha) + ", " + str(beta)+", "+str(gamma)
        
        
        
        
        '''
        # Resolve into co-ordinate system
        # The euler angles effective give quad rotations from the world
        # vx and vy need to be resolved back by these to world axis
        '''
        
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!! Current code appears wrong !!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        # Produce reverse rotation matrix
        origin, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
        Rx = tf.transformations.rotation_matrix(-alpha, xaxis)
        Ry = tf.transformations.rotation_matrix(-beta, yaxis)
        Rz = tf.transformations.rotation_matrix(-gamma, zaxis)
        R = tf.transformations.concatenate_matrices(Rx, Ry, Rz)
        R = R[:3, :3] # Is homonogenous ???
        
        # vectorize
        vel_vector = np.asmatrix([vx, vy, 0.]).T
        print vel_vector
        
        # Apply rotayion matrix
        vel_vector = R.dot(vel_vector)
        
        
        '''
        # Update location
        '''
        vel_list = list(vel_vector.flat)
        #print vel_list
        self.x += vel_list[0]*deltat
        self.y += vel_list[1]*deltat
        
        print str(self.x) + "," + str(self.y) + "," + str(self.z)
        
        
        
        '''
        # Publish tf
        '''
        br = tf.TransformBroadcaster() #create broadcaster
        br.sendTransform((self.x,self.y,self.z),
                         quaternion,
                         rospy.Time.now(), # NB: 'now' is not the same as time data was sent from drone
                         "ardrone_base_link",
                         "world")

if __name__ == '__main__':
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    rospy.init_node('deadreckon_broadcaster')
    #turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,dead_reckon.navdataCallback)
    rospy.spin()
