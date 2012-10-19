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
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world
        self.prevtime = None # time (float secs) of prev frame
        self.v_buffer = [] # buffer of last 10 frames [d.vx, d.vy]
        self.rotZoffset = 0 # initial value of yaw to allow for offsets
        self.trackPath = Path()

    def reset(self,d):
        print "resetting"
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world
        self.rotZoffset = self.gamma 
        self.trackPath = Path()


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
        # Resolve into co-ordinate system
        # The euler angles effectively give quad rotations from the world
        # vx and vy need to be resolved back by these to world axis
        '''
        
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!! Current code appears wrong !!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        # Produce reverse rotation matrix
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

        # radians to rotate from camera to world frame
        cam_yaw = self.gamma - self.rotZoffset 
        
        vx_rot = math.cos(cam_yaw)*vx - math.sin(cam_yaw)*vy
        vy_rot = math.sin(cam_yaw)*vx + math.cos(cam_yaw)*vy

        self.x += vx_rot*deltat / 1000 # convert mm/s to ROS standard unit of m/s
        self.y += vy_rot*deltat / 1000
        self.z = d.altd / 1000



        # Apply rotation matrix
        #vel_vector = R.dot(vel_vector)
        
        
        '''
        # Update location
        '''
        #vel_list = list(vel_vector.flat)
        #print vel_list
        #self.x += vel_list[0]*deltat
        #self.y += vel_list[1]*deltat
        
        #print str(self.x) + "," + str(self.y) + "," + str(self.z)
        
        
        
        '''
        # Publish tf
        '''
        br = tf.TransformBroadcaster() #create broadcaster
        br.sendTransform((self.x,self.y,self.z), # translation happens first, then rotation
                         quaternion,
                         rospy.Time.now(), # NB: 'now' is not the same as time data was sent from drone
                         "ardrone_base_link",
                         "world")

        '''
        # Publish path visualisation data
        '''
        # copy headers over from Navdata
        self.trackPath.header        = d.header
        newPose = PoseStamped()
        newPose.header               = d.header
        newPose.header.frame_id      = 'world'

        newPose.pose.position.x      = self.x
        newPose.pose.position.y      = self.y
        newPose.pose.position.z      = self.z
        newPose.pose.orientation.x   = self.alpha  
        newPose.pose.orientation.y   = self.beta  
        newPose.pose.orientation.z   = self.gamma
        
        self.trackPath.poses.append(newPose)

        pathPub = rospy.Publisher('deadreckon_path',Path)
        pathPub.publish(self.trackPath)


if __name__ == '__main__':
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    rospy.init_node('deadreckon_broadcaster')
    #turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,dead_reckon.navdataCallback)
    rospy.Subscriber('/xboxcontroller/button_start',Empty,dead_reckon.reset)
    rospy.spin()
