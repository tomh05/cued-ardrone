#!/usr/bin/env python  
import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
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
        self.frameno = None
        self.bingno = -1
        self.bongno = 0
        self.bingtime = None 
        self.bongtime = None

    def videostream(self, d):
        if self.frameno == None:
            self.frameno = d.frame_number
            self.bongtime = d.drone_time
            return
            
        if d.frame_number != self.frameno:
            self.frameno = d.frame_number
            self.bongno = self.bongno+1
            self.latest = d.drone_time-self.bongtime
            print 'a', (self.bongno, d.drone_time-self.bongtime)
            
    def image_raw(self, d):
        if self.bingno == -1:
            self.bingno = 0
            self.bingtime = d.header.stamp.to_sec()
            return
        print 'b', (self.bingno, d.header.stamp.to_sec()-self.bingtime)
        
        print "diff: ", d.header.stamp.to_sec()-self.bingtime - self.latest
        self.bingno = self.bingno+1
   
        
        


if __name__ == '__main__':
    
    rospy.init_node('deadreckon_broadcaster')
    
    dead_reckon = DeadReckoning()
    
    rospy.Subscriber('/ardrone/navdata_video_stream',ardrone_autonomy.msg.navdata_video_stream,dead_reckon.videostream)
    rospy.Subscriber('/ardrone/front/image_raw',Image,dead_reckon.image_raw)
    
    rospy.spin()
