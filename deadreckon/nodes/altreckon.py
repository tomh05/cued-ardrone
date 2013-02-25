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

    def navdataCallback(self, d):
        
        
        self.z = (d.altitude_raw*1.) / 1000

        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        self.time = d.header.stamp
        '''
        # Publish tf
        '''
        self.br.sendTransform((self.x,self.y,self.z), 
                         # translation happens first, then rotation
                         self.quaternion,
                         self.time,
                         "/ardrone_base_link",
                         "/world")                         
        # (values pulled from ardrone_drive.cpp)
        # We need to publish this when using high rate navdata to prevent
        # front camera tf from lagging base ardrone
        self.br.sendTransform(self.frontcam_t, self.frontcam_quat ,self.time, '/ardrone_base_frontcam', '/ardrone_base_link')
        # NOTE: child parent order is reversed w.r.t C++

if __name__ == '__main__':
    dead_reckon = DeadReckoning() # Initialise class to preserve vars
    rospy.init_node('deadreckon_broadcaster')
    rospy.Subscriber('/ardrone/navdata_altitude',ardrone_autonomy.msg.navdata_altitude,dead_reckon.navdataCallback)
    rospy.Subscriber('/xboxcontroller/button_start',Empty,dead_reckon.reset)
    
    # Get parameters
    FILTERSIZE = rospy.get_param('~filtersize', 10)
    FILTERENABLE = rospy.get_param('~filterenable', False)
    if FILTERSIZE < 2:
        FILTERENABLE = False
    print "\r\n"
    print "========================== Alt Reckon ============================"
    print " Postions purely on altitude regardless of if drone is flying"
    print "==================================================================="
    print "\r\n"
    
    
    rospy.spin()
