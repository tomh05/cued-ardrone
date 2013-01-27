#!/usr/bin/env python

import roslib
roslib.load_manifest('floor_gridder')
import rospy
import math
import cv2
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension
from custom_msgs.msg import RendererPolyLineTri

class FloorGridder():
    def __init__(self):        
        self.connect()
        self.clear_maps()
        
        
        
    
    def connect(self):
        rospy.init_node('Floor_Gridder')
        rospy.Subscriber('/point_hander/renderer_data', RendererPolyLineTri, self.on_got_renderer_data)
        self.gridded_map_pub = rospy.Publisher('/floor_gridder/gridded_map', Int16MultiArray)
        self.nav_map_pub = rospy.Publisher('/floor_gridder/nav_map', Int16MultiArray)
        
    def clear_maps(self):
        # Gridded map is simply a display purpose map
        self.gridded_map = np.ones(shape=(1024,1024), dtype = np.uint8)
        self.gridded_map = 255*self.gridded_map
        # Nav map is a map of valid drone centre positions
        self.nav_map = np.ones(shape=(1024,1024), dtype = np.uint8)
        self.nav_map = 255*self.gridded_map
        # Centre offset gives the [x; y] distance from top left to world origin
        self.centre_offset = np.array([self.gridded_map.shape[1]/2, 
                                       self.gridded_map.shape[0]/2])
        
    def on_got_renderer_data(self, renderer_poly_line_tri):
        # end_xyxy is an array in the pattern {x1, y1, x2, y2, x3, y3 ... }
        # where each pair of x1y1 and x2y2 are the end points of a line
        end_xyxy = renderer_poly_line_tri.floorlines
        
        # Working with 10cm grid size
        i = 0
        while (i < len(end_xyxy)):
            x1 = int(10*end_xyxy[i])
            y1 = int(10*end_xyxy[i+1])
            x2 = int(10*end_xyxy[i+2])
            y2 = int(10*end_xyxy[i+3])
            cv2.line(self.gridded_map, (x1+self.centre_offset[0],y1+self.centre_offset[1]), (x2+self.centre_offset[0], y2+self.centre_offset[1]), (0, 0, 0))
            cv2.line(self.nav_map, (x1+self.centre_offset[0],y1+self.centre_offset[1]), (x2+self.centre_offset[0], y2+self.centre_offset[1]), (0, 0, 0), thickness=6)
            i = i+4
            print i
        print "Done"
        cv2.imwrite('test.png', self.gridded_map)
        cv2.waitKey(2)
        
        send_gridded_map = Int16MultiArray()
        send_gridded_map.data=self.gridded_map.reshape(-1,).tolist()
        send_gridded_map.layout.dim = [MultiArrayDimension('width', self.gridded_map.shape[1]*self.gridded_map.shape[0], self.gridded_map.shape[1])]
        print send_gridded_map.data[0]
        
        
        self.gridded_map_pub.publish(send_gridded_map)

def run():    
    fg = FloorGridder()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
