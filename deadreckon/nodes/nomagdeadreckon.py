#!/usr/bin/env python  
import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np
import message_filters
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

DEG2RAD = np.pi/180.

FILTERENABLE = True
FILTERSIZE = 10
PATHDIV = 20



class DeadReckoning:
    def __init__(self):
        # Clear accumulating variables
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world
        self.rotZoffset = 0 # initial value of yaw to allow for offsets
        self.trackPath = Path
        self.path_div = 0
        
        # Declare publishers and constant tf values
        self.frontcam_quat = tf.transformations.quaternion_from_euler(-90.0 * (np.pi/180.), 0.0, -90.0 * (np.pi/180.))
        self.frontcam_t = (0.21, 0.0, 0.0)
        self.br = tf.TransformBroadcaster()        
        self.pathPub = rospy.Publisher('deadreckon_path',Path)
        
        # Clear time values
        self.prevtime = None # time (float secs) of prev frame
        
        # Clear median buffer
        self.v_buffer = None # buffer of last FILTERSIZE frames [d.vx, d.vy]
        self.buffer_index = -FILTERSIZE
        
        self.rotZ = 0
        self.gamma = 0
      
    def complementary_filter(self, angle1, angle2, rate2):

        tau=0.5; # default 1.0

        accel_delta = (angle2 - angle1) * tau * tau;
        filterTerm2 = accel_delta * dt + filterTerm2;
        filterTerm1 = filterTerm2 + ((angle2 - angle1) * 2 * tau) + rate2;
        angle1 = (filterTerm1 * dt) + angle1;

        return previousAngle; # This is actually the current angle, but is stored for the next iteration
        
    def first_order_completmentary_filter(self, angle, accel, gyro, dt):
        tau = 0.5 # Tuneable time constant
        alpha = tau/(tau+dt);
        angle = alpha *(angle+gyro*dt) + (1-alpha)*accel
        return angle

        
    def reset(self,d):
        print "resetting"
        # Clear accumulating variables
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world        
        self.rotZoffset = self.gamma        
        self.trackPath = Path()
        self.path_div = 0
        
        # Clear time values
        self.prevtime = None
        
        # Clear median buffer
        self.v_buffer = None # buffer of last 10 frames [d.vx, d.vy]
        self.buffer_index = -FILTERSIZE + 1

    def navdata_callback(self, d, phys):
        if d.batteryPercent < 25:
            print "Warning: Battery at ", d.batteryPercent, "%"
            
        '''
        # Sort out frame timings
        '''
        time = d.header.stamp
        if self.prevtime == None:
            self.reset(self)
            self.prevtime = time
        deltat = (time - self.prevtime).to_sec()
        self.prev_deltat = deltat
        self.prevtime = time
        # if playing rosbags and time jumps backwards, we want to reset
        if (deltat < -1.0): 
            self.reset(self)
            
        if FILTERENABLE:
            '''
            # Running FILTERSIZE sample median buffer 
            # (reduce noise on d.vx, d.vy)
            #
            # This version uses a fixed size np buffer (after initially filled)
            # so avoids reallocating memory
            '''
            if self.v_buffer == None:
                self.v_buffer = np.array([[d.vx],[d.vy]])
            elif self.buffer_index < 0:
                self.v_buffer = np.hstack((self.v_buffer, np.array([[d.vx],[d.vy]])))
                self.buffer_index = self.buffer_index + 1
            else:
                self.v_buffer[0, self.buffer_index] = d.vx
                self.v_buffer[1, self.buffer_index] = d.vy
                self.buffer_index = self.buffer_index + 1
                if self.buffer_index == FILTERSIZE:
                    self.buffer_index = 0
            vx = np.median(self.v_buffer[0])
            vy = np.median(self.v_buffer[1])
        else:
            vx = d.vx
            vy = d.vy
        
        # Calculate gyro rotation and accelerometer angle
        accX = +phys.phys_accs[0]
        accY = -phys.phys_accs[1]
        accZ = -phys.phys_accs[2]
        gyrX = +phys.phys_gyros[0]
        gyrY = -phys.phys_gyros[1]
        gyrZ = -phys.phys_gyros[2]
        pitch = math.atan2(accY, accZ)+np.pi
        roll = math.atan2(accX, accZ)+np.pi
        
        
        
        
        '''
        # Create rotation quaternion
        '''
        # Euler angles in radians        
        self.alpha = DEG2RAD*d.rotX # roll
        self.beta  = DEG2RAD*d.rotY # pitch
        self.gamma = DEG2RAD*d.rotZ # yaw
        vector = np.array([[gyrX],[gyrY],[gyrZ]])
        #self.rotZ = self.rotZ + (gyrZ*math.cos(self.alpha)*math.cos(self.beta) + gyrY*math.sin(self.alpha) - gyrX*math.cos(self.alpha)*math.sin(self.beta) )*deltat*DEG2RAD
        #self.rotZ = self.rotZ + R.dot(vector)[2]*deltat*DEG2RAD
        
        # Resolve body gyro rates into euler rates
        rate_of_yaw = gyrY*math.sin(self.alpha)/math.cos(self.beta) + gyrZ*math.cos(self.alpha)/math.cos(self.beta)
        self.rotZ = self.rotZ + rate_of_yaw*deltat*DEG2RAD
        self.gamma = self.rotZ
        # produce quaternion
        quaternion = tf.transformations.quaternion_from_euler(self.alpha,self.beta,self.gamma-self.rotZoffset,   axes='sxyz')
        
        
        
        
        
        
        
        
        
        
        '''
        # Resolve into co-ordinate system
        # vx and vy need to be resolved back by these to world axis
        '''
        corr_angle = self.gamma - self.rotZoffset 
        vx_rot = np.cos(corr_angle)*vx - np.sin(corr_angle)*vy
        vy_rot = np.sin(corr_angle)*vx + np.cos(corr_angle)*vy
        
        
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
        # Publish path visualisation data every 10 navdatas
        '''
        if (self.path_div % PATHDIV == 0):
            self.trackPath.header        = d.header

            # copy headers over from Navdata
            self.trackPath.header        = d.header
            newPose = PoseStamped()
            newPose.header               = d.header
            newPose.header.frame_id      = '/world'
            newPose.pose.position.x      = self.x
            newPose.pose.position.y      = self.y
            newPose.pose.position.z      = self.z
            self.trackPath.poses.append(newPose)        
            self.pathPub.publish(self.trackPath)
        self.path_div = self.path_div + 1

if __name__ == '__main__':
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    rospy.init_node('deadreckon_broadcaster')
    rospy.Subscriber('/xboxcontroller/button_start',Empty,dead_reckon.reset)
    
    
    navdata_sub = message_filters.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata)
    phys_sub = message_filters.Subscriber('/ardrone/navdata_phys_measures',ardrone_autonomy.msg.navdata_phys_measures)

    ts = message_filters.TimeSynchronizer([navdata_sub, phys_sub], 50) # buffer for 1/4 secs @ 200Hz
    ts.registerCallback(dead_reckon.navdata_callback)
    
    # Get parameters
    FILTERSIZE = rospy.get_param('~filtersize', 10)
    FILTERENABLE = rospy.get_param('~filterenable', False)
    if FILTERSIZE < 2:
        FILTERENABLE = False
    print "\r\n"
    print "========================== Dead Reckon ============================"
    print "Use median filter (", FILTERSIZE, ") : ", FILTERENABLE, " - (set with _filtersize and _filterenable)"
    PATHDIV = rospy.get_param('~pathdiv', 20)
    print "Downsample path by factor of: ", PATHDIV, " - (set with _pathdiv)"
    print "==================================================================="
    print "\r\n"
    
    
    rospy.spin()
