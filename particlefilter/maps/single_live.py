#!/usr/bin/env python  
import roslib
roslib.load_manifest('particlefilter')
import rospy
import ardrone_autonomy.msg
import math
import numpy as np
import pickle

from   std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from ar_pose.msg import ARMarkers, ARMarker

markermap = ARMarkers()
markermap.header.frame_id = "\world"



# define a marker

m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 0  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = 3.0
m.pose.pose.position.y      = 0.0
m.pose.pose.position.z      = 1.0

m.pose.pose.orientation.x   = 0.0
m.pose.pose.orientation.y   = 0.0
m.pose.pose.orientation.z   = 0.0
m.pose.pose.orientation.w   = 0.0

markermap.markers.append(m)

f = open("single_live.map",'w')
pickle.dump(markermap,f)

# gamma orientation offset for map
gamma = 3.141
pickle.dump(gamma,f)

print "Created map"
