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

FILTERENABLE = False
FILTERSIZE = 10
TMDELTAT = False
TMSTAMP = False
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
        self.path_div = 0
        
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
                self.v_buffer = np.array([[d.magX],[d.magY],[d.magZ]])
            elif self.buffer_index < 0:
                self.v_buffer = np.hstack((self.v_buffer, np.array([[d.magX],[d.magY],[d.magZ]])))
                self.buffer_index = self.buffer_index + 1
            else:
                self.v_buffer[0, self.buffer_index] = d.magX
                self.v_buffer[1, self.buffer_index] = d.magY
                self.v_buffer[2, self.buffer_index] = d.magZ
                self.buffer_index = self.buffer_index + 1
                if self.buffer_index == FILTERSIZE:
                    self.buffer_index = 0
            magX = np.median(self.v_buffer[0])
            magY = np.median(self.v_buffer[1])
            magZ = np.median(self.v_buffer[2])
        else:
            vx = d.vx
            vy = d.vy
            magX = d.magX
            magY = d.magY
            magZ = d.magZ
        
        print (magX, magY, magZ)
        
        


if __name__ == '__main__':
    
    rospy.init_node('deadreckon_broadcaster')
    
    # Get parameters
    FILTERSIZE = rospy.get_param('~filtersize', 10)
    FILTERENABLE = rospy.get_param('~filterenable', False)
    if FILTERSIZE < 2:
        FILTERENABLE = False
    
    print "\r\n----------------------------------DEAD RECKON--------------------------------------"
    print "Use median filter (", FILTERSIZE, ") : ", FILTERENABLE, "        (set with _filtersize and _filterenable)"
    
    TMDELTAT = rospy.get_param('~usetm', True)
    print "Use tm instead of headers : ", TMDELTAT, "         (set with _usetm)"
    TMSTAMP = TMDELTAT
    
    PATHDIV = rospy.get_param('~pathdiv', 20)
    print "Downsample path by factor of: ", PATHDIV, "         (set with _pathdiv)"
    
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    
    rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,dead_reckon.navdataCallback)
    
    rospy.spin()
