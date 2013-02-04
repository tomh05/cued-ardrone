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

DEG2RAD = np.pi/180.

FILTERENABLE = True
FILTERSIZE = 10
TMDELTAT = False
TMSTAMP = False



class DeadReckoning:
    def __init__(self):
        # Clear accumulating variables
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world
        self.rotZoffset = 0 # initial value of yaw to allow for offsets
        self.trackPath = Path()
        
        # Declare publishers and constant tf values
        self.frontcam_quat = tf.transformations.quaternion_from_euler(-90.0 * (np.pi/180.), 0.0, -90.0 * (np.pi/180.))
        self.frontcam_t = (0.21, 0.0, 0.0)
        self.br = tf.TransformBroadcaster()        
        self.pathPub = rospy.Publisher('deadreckon_path',Path)
        
        # Clear time values
        self.prevtime = None # time (float secs) of prev frame
        self.prev_tm = None
        self.offset_tm = None # This is the tm based time in seconds
        
        # Clear median buffer
        self.v_buffer = None # buffer of last FILTERSIZE frames [d.vx, d.vy]
        self.buffer_index = -FILTERSIZE
        
    def reset(self,d):
        print "resetting"
        # Clear accumulating variables
        self.x = 0.0 # x location in world
        self.y = 0.0 # y location in world
        self.z = 0.0 # z location in world        
        self.rotZoffset = self.gamma        
        self.trackPath = Path()
        
        # Clear time values
        self.prevtime = None
        self.prev_tm = None
        self.offset_tm = None
        
        # Clear median buffer
        self.v_buffer = None # buffer of last 10 frames [d.vx, d.vy]
        self.buffer_index = -FILTERSIZE + 1

    def navdataCallback(self, d):        
        if d.batteryPercent < 25:
            print "Warning: Battery at ", d.batteryPercent, "%"
            
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
        
        
        
        '''
        # Create rotation quaternion
        '''
        # Euler angles in radians        
        self.alpha = DEG2RAD*d.rotX # roll
        self.beta  = DEG2RAD*d.rotY # pitch
        self.gamma = DEG2RAD*d.rotZ # yaw
        # produce quaternion
        quaternion = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma - self.rotZoffset)
        
        
        
        
        '''
        # Sort out frame timings
        '''
        time = d.header.stamp
        if self.prevtime == None:
            self.reset(self)
            self.prevtime = time
            self.prev_tm = d.tm
            self.offset_tm = time.to_sec()
            self.init_time = time.to_sec()
        if TMDELTAT:
            deltat = (float(d.tm-self.prev_tm))/1000000.
            # If not using realtime navdata, tm has occasional large values
            # These are incorrect and crudely removed by assuming same as prev
            if deltat > 0.2:
                print "De-blipping"
                deltat = self.prev_deltat
            self.offset_tm = self.offset_tm + deltat
        else:
            deltat = (time - self.prevtime).to_sec()
            
        #print deltat
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
        if TMSTAMP:
            self.br.sendTransform((self.x,self.y,self.z), 
                         # translation happens first, then rotation
                         quaternion,
                         rospy.Time.from_sec(self.offset_tm),
                         "/ardrone_base_link",
                         "/world")                         
            # (values pulled from ardrone_drive.cpp)
            # We need to publish this when using high rate navdata to prevent
            # front camera tf from lagging base ardrone
            self.br.sendTransform(self.frontcam_t, self.frontcam_quat ,rospy.Time.from_sec(self.offset_tm), '/ardrone_base_frontcam', '/ardrone_base_link')
            # NOTE: child parent order is reversed w.r.t C++
        else:
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
        newPose.pose.orientation.z   = corr_angle
        self.trackPath.poses.append(newPose)        
        self.pathPub.publish(self.trackPath)


if __name__ == '__main__':
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    rospy.init_node('deadreckon_broadcaster')
    rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,dead_reckon.navdataCallback)
    rospy.Subscriber('/xboxcontroller/button_start',Empty,dead_reckon.reset)
    
    # Get parameters
    FILTERSIZE = rospy.get_param('~filtersize', 10)
    FILTERENABLE = rospy.get_param('~filterenable', False)
    if FILTERSIZE < 2:
        FILTERENABLE = False
    
    print "\r\n----------------------------------DEAD RECKON--------------------------------------"
    print "Use median filter (", FILTERSIZE, ") : ", FILTERENABLE, "        (set with _filtersize and _filterenable)"
    
    TMDELTAT = rospy.get_param('~usetmdeltat', True)
    print "Use tm for deltat        : ", TMDELTAT, "         (set with _usetmdeltat)"
    TMSTAMP = rospy.get_param('~usetmstamp', True)
    print "Use tm for timestamps    : ", TMSTAMP, "         (set with _usetmstamp)"
    print "-----------------------------------------------------------------------------------\r\n"
    
    
    rospy.spin()
