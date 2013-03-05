#!/usr/bin/env python

#=================== Dead Reckon (no magnetometer) =====================
# 
# This script caccumulates position based on gyro and accelerometer data
# The yaw is calculated without the magnetometer for use where the 
# magnetometer fails.
#
# The roll and pitch euler angles do not appear to drift as the onboard
# calculated yaw does. As such, these are presumably produced using 
# gyros and accelerometers for rate and absolute respectively, combined
# in either a Kalman or complentary filter. Complementary filter (1st &
# 2nd order) were added here but are not used.
#
# The navdata_phys_measures provides post processed gyro results with
# biases removed.
#
# Note: As the yaw is resolved in euler angles, it IS vulnerable to 
#       gimbal lock. This is not an issue though as in the frame used
#       gimbal lock occurs at a pitch of 90deg (drone vertical), which
#       should not happen in flight and triggers a safety cut-out.
#=======================================================================

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
        self.trackPath = Path
        self.path_div = 0
        self.filterTerm2 = 0.
        self.gamma = 0.
        
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
        
    def reset(self,d):
        print "resetting"
        # Clear accumulating variables
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world   
        self.trackPath = Path()
        self.path_div = 0
        self.filterTerm2 = 0.
        self.gamma = 0.        
        
        # Clear time values
        self.prevtime = None
        
        # Clear median buffer
        self.v_buffer = None # buffer of last 10 frames [d.vx, d.vy]
        self.buffer_index = -FILTERSIZE + 1
      
    def second_order_complementary_filter(self, angle1, angle2, rate2): # Not used        
        tau=0.5; # default 1.0
        accel_delta = (angle2 - angle1) * tau * tau;
        self.filterTerm2 = accel_delta * dt + self.filterTerm2;
        filterTerm1 = fself.ilterTerm2 + ((angle2 - angle1) * 2 * tau) + rate2;
        angle_now = (filterTerm1 * dt) + angle1;

        return angle_now; # This is actually the current angle, but is stored for the next iteration
        
    def first_order_complementary_filter(self, angle, accel, gyro, dt): # Not used
        tau = 0.5 # Tuneable time constant
        alpha = tau/(tau+dt);
        angle = alpha *(angle+gyro*dt) + (1-alpha)*accel
        return angle

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
        
        # Seperate reading and switch to consistent axis
        accX = +phys.phys_accs[0]
        accY = -phys.phys_accs[1]
        accZ = -phys.phys_accs[2]
        gyrX = +phys.phys_gyros[0]
        gyrY = -phys.phys_gyros[1]
        gyrZ = -phys.phys_gyros[2]      
        
        '''
        # Create rotation quaternion
        '''
        # Euler angles in radians        
        self.alpha = DEG2RAD*d.rotX # roll
        self.beta  = DEG2RAD*d.rotY # pitch
        
        # Resolve body gyro rates into euler rates
        rate_of_yaw = gyrY*math.sin(self.alpha)/math.cos(self.beta) + gyrZ*math.cos(self.alpha)/math.cos(self.beta)
        # Integrate to euler angle
        self.gamma = self.gamma + rate_of_yaw*deltat*DEG2RAD # yaw
        
        # produce quaternion
        quaternion = tf.transformations.quaternion_from_euler(self.alpha,self.beta,self.gamma,axes='sxyz')
        
        
        '''
        # Resolve into co-ordinate system
        # vx and vy need to be resolved back by these to world axis
        '''
        corr_angle = self.gamma 
        vx_rot = np.cos(corr_angle)*vx - np.sin(corr_angle)*vy
        vy_rot = np.sin(corr_angle)*vx + np.cos(corr_angle)*vy
        
        
        '''
        # Summation
        '''
        self.x += vx_rot*deltat / 1000.
        self.y += vy_rot*deltat / 1000.
        # altd is an >>>int<<< in mm for drone altitude
        self.z = float(d.altd) / 1000.

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
    print "===================== Dead Reckon (No Mag) ======================="
    print "Use median filter (", FILTERSIZE, ") : ", FILTERENABLE, " - (set with _filtersize and _filterenable)"
    PATHDIV = rospy.get_param('~pathdiv', 20)
    print "Downsample path by factor of: ", PATHDIV, " - (set with _pathdiv)"
    print "==================================================================="
    print "\r\n"
    
    
    rospy.spin()
