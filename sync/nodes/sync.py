#!/bin/sh
''''exec python -u -- "$0" ${1+"$@"} # '''

#========================== Feature Extractor =================================
# 
# This node sets navdata and image header stamps to actual measurement time
#
#==============================================================================
#
# ardrone_driver publishes navdata and image_raw with header stamp
# The navdata header corresponds to the time ardrone_driver receives navdata
# The image_raw header corresponds to the time ardrone_driver sends image_raw
#
# navdata.tm is the drone time in milliseconds
# image_raw does not have a drone time (it is stripped in the parrot pipeline)
# 
# navdata_video_stream provides info about parrot encoding including drone
# times.
#
# This node first syncronises the navdata_video_stream frame timings and the 
# image_raw frames. It uses this to establish a drone time to ROS time offset
# and republishes the navdata and image_raw feeds with header stamps that are
# in the same time frame and reflect measurement time.
#
#
#==============================================================================



import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import math
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
import sys




class Syncer:
    def __init__(self):
        self.locked = False
        self.locked2 = False
        self.locked2count = 0
        
        self.locked_stamp = None
        
        self.ros_time_offset = None
        self.frame_offset = None
        self.frame_prev = -1
        self.frame_count = 0
        
        self.time_buffer = []
        self.image_buffer = []
        
        self.dnav = 0
        self.dimage = 0
        
        sys.stdout.write("Unlocked")
        self.connect()
        
        
    def connect(self):
        rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,self.on_got_navdata)
        rospy.Subscriber('/ardrone/navdata_video_stream',ardrone_autonomy.msg.navdata_video_stream,self.on_got_navdata_video_stream)
        rospy.Subscriber('/ardrone/front/image_raw',Image, self.on_got_image)
        self.image_pub = rospy.Publisher('/sync/image_raw', Image)
        self.nav_pub = rospy.Publisher('/sync/navdata', ardrone_autonomy.msg.Navdata)
        
    def on_got_image(self, d):
        self.frame_count = self.frame_count + 1
        
        if not self.locked:
            
            if len(self.time_buffer) < 10:
                return
            i = d.header
            fc = self.frame_count
            
            best = None            
            self.bests = []
            
            for index, t in enumerate(self.time_buffer):
                if t.header.stamp.to_sec() <= i.stamp.to_sec():
                    best = (t, i, index)
                    self.bests.append(best)
                elif best != None:
                    self.frame_offset = best[0].frame_number - fc
                    self.ros_time_offset = best[0].header.stamp.to_sec()-best[0].drone_time
                    self.dnav = best[0].frame_number - self.frame_offset
                    self.locked = True
                    sys.stdout.write("\rUnlocked - > Locked                                          ")
                    self.best_index = index
                    self.purge_time_buffer(best[2])
                    self.locked2count = self.frame_count
                    return
        elif not self.locked2:
            if self.frame_count - self.locked2count < 50:
                if self.frame_count - self.locked2count == 1:
                    return
                if self.dnav < self.frame_count:
                    self.locked2count = self.frame_count
                    self.best_index = self.best_index - 1
                    if self.best_index < 0:
                        sys.stdout.write("\rUnlocked < - Locked")
                        self.locked = False
                        return
                    self.frame_offset = self.frame_offset - 1
                    self.ros_time_offset = self.bests[self.best_index][0].header.stamp.to_sec()-self.bests[self.best_index][0].drone_time
                    self.dnav = self.dnav + 1
            else:
                self.locked2 = True
                self.runin = 0
                sys.stdout.write("\rUnlocked - > Locked - > Double Locked")
        else:
            if len(self.time_buffer) > 0 and self.frame_count == self.time_buffer[0][1]:
                self.publish_image(d, self.time_buffer.pop(0)[0])
            else:
                sys.stdout.write(' #')
                self.runin = self.runin + 1
                if self.runin > 10:
                    self.locked = False
                    self.locked2 = False
                    self.time_buffer = []
                    sys.stdout.write("\rUnlocked < - Locked < - Double Locked                       ")
    
    def purge_time_buffer(self, index):
        for i, t in enumerate(self.time_buffer):
            if i > index:
                self.dnav = t.frame_number-self.frame_offset
        self.time_buffer = []
    
    def on_got_navdata_video_stream(self, d):
        if not self.locked:
            if self.frame_prev != d.frame_number:
                self.frame_prev = d.frame_number
                self.time_buffer.append(d)
        elif not self.locked2:
            if self.frame_prev != d.frame_number:
                self.frame_prev = d.frame_number
                self.dnav = d.frame_number-self.frame_offset
        else:
            if self.frame_prev != d.frame_number:
                self.frame_prev = d.frame_number
                self.dnav = d.frame_number-self.frame_offset
                self.time_buffer.append((d.drone_time, self.dnav))
        
    def on_got_navdata(self, d):
        if self.locked2:
            d.header.stamp = rospy.Time.from_sec(self.ros_time_offset+d.tm*0.000001)
            self.nav_pub.publish(d)
        
    def publish_image(self, image, tm):
        image.header.stamp = rospy.Time.from_sec(self.ros_time_offset+tm)
        self.image_pub.publish(image)
        


if __name__ == '__main__':
    
    rospy.init_node('sync')
    
    print "\r\n"
    print "============================== Sync ==============================="
    print " Wait until stable on Double Locked state"
    print "==================================================================="
    print "\r\n"
    s = Syncer() # Initialise class to preserve vars
    rospy.spin()
