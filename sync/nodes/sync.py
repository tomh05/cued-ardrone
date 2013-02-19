#!/usr/bin/env python  
import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import math
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Image




class Syncer:
    def __init__(self):
        self.locked = False
        self.locked_stamp = None
        
        self.time_offset = None
        self.frame_offset = None
        self.frame_prev = -1
        self.frame_count = 0
        
        self.time_buffer = []
        self.image_buffer = []
        
        self.connect()
        
        
    def connect(self):
        rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,self.on_got_navdata)
        rospy.Subscriber('/ardrone/navdata_video_stream',ardrone_autonomy.msg.navdata_video_stream,self.on_got_navdata_video_stream)
        rospy.Subscriber('/ardrone/front/image_raw',Image, self.on_got_image)
        
    def on_got_image(self, d):
        self.frame_count = self.frame_count + 1
        
        if not locked:
            if len(self.image_buffer) < 5:
                self.image_buffer.append((d.header, self.frame_count))
                
            best = None
            for i, fc in self.image_buffer:
                for t in self.time_buffer:
                    #print best
                    if t.header.stamp.to_sec() < i.stamp.to_sec():
                        best = (t, i)
                    elif best != None:
                        self.frame_offset = best[0].frame_number - fc
                        self.time_offset = best[1].stamp.to_sec()
                        self.locked == True
                        self.image_buffer = []
                        return
        else:
            self.image_buffer.append(d, self.frame_count)
                        
    
    def on_got_navdata_video_stream(self, d):
        if not locked:
            if self.frame_prev != d.frame_number:
                self.frame_prev = d.frame_number
                self.time_buffer.append(d)
        else:
            if self.frame_prev != d.frame_number:
                self.frame_prev = d.frame_number
                
                for i, image, fc in enumerate(self.image_buffer):
                    if fc + self.frame_buffer
            
            
            
                        
                        
            
        
            
        
    def on_got_navdata(self, d):
        self.image_buffer.append(d)


if __name__ == '__main__':
    
    rospy.init_node('sync')
    s = Syncer() # Initialise class to preserve vars
    
    
    rospy.spin()
