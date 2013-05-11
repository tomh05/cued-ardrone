#!/usr/bin/env python  
import roslib
roslib.load_manifest('particlefilter')
import rospy
import ardrone_autonomy.msg
import math
import numpy as np
import pickle

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from ar_pose.msg import ARMarkers, ARMarker

markermap = ARMarkers()
markermap.header.frame_id = "\world"

# going clockwise around room


#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 0  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = 3.250
m.pose.pose.position.y      = 1.050
m.pose.pose.position.z      = 1.650

m.pose.pose.orientation.x   = 0.5
m.pose.pose.orientation.y   = -0.5
m.pose.pose.orientation.z   = -0.5
m.pose.pose.orientation.w   = 0.5

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 1  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = 3.250
m.pose.pose.position.y      = -1.850
m.pose.pose.position.z      = 1.510

m.pose.pose.orientation.x   = 0.5
m.pose.pose.orientation.y   = -0.5
m.pose.pose.orientation.z   = -0.5
m.pose.pose.orientation.w   = 0.5

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 2  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = 0.180
m.pose.pose.position.y      = -2.800
m.pose.pose.position.z      = 1.580

m.pose.pose.orientation.x   = 0.0
m.pose.pose.orientation.y   = 0.707
m.pose.pose.orientation.z   = 0.707
m.pose.pose.orientation.w   = 0.0

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 3  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = -3.050
m.pose.pose.position.y      = -0.83
m.pose.pose.position.z      = 1.430

m.pose.pose.orientation.x   = 0.5
m.pose.pose.orientation.y   = 0.5
m.pose.pose.orientation.z   = 0.5
m.pose.pose.orientation.w   = 0.5

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 4  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = -3.050
m.pose.pose.position.y      = 1.540
m.pose.pose.position.z      = 1.570

m.pose.pose.orientation.x   = 0.5
m.pose.pose.orientation.y   = 0.5
m.pose.pose.orientation.z   = 0.5
m.pose.pose.orientation.w   = 0.5

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 5  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = -0.450
m.pose.pose.position.y      = 2.450
m.pose.pose.position.z      = 1.280

m.pose.pose.orientation.x   = 0.707
m.pose.pose.orientation.y   = 0.0
m.pose.pose.orientation.z   = 0.0
m.pose.pose.orientation.w   = 0.707

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 6  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = 2.200
m.pose.pose.position.y      = 2.450
m.pose.pose.position.z      = 1.460

m.pose.pose.orientation.x   = 0.707
m.pose.pose.orientation.y   = 0.0
m.pose.pose.orientation.z   = 0.0
m.pose.pose.orientation.w   = 0.707

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 7  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = -0.450
m.pose.pose.position.y      = 2.450
m.pose.pose.position.z      = 0.470

m.pose.pose.orientation.x   = 0.707
m.pose.pose.orientation.y   = 0.0
m.pose.pose.orientation.z   = 0.0
m.pose.pose.orientation.w   = 0.707

markermap.markers.append(m)
# copy above section for more markers

#----------------------------------------------------------
#define a marker
m = ARMarker()
m.header.frame_id           = "/world"
m.id                        = 8  # this refers to the order in the ar_posemarker configuration file, not the filename of the marker

m.pose.pose.position.x      = 0.620
m.pose.pose.position.y      = 2.450
m.pose.pose.position.z      = 0.810

m.pose.pose.orientation.x   = 0.707
m.pose.pose.orientation.y   = 0.0
m.pose.pose.orientation.z   = 0.0
m.pose.pose.orientation.w   = 0.707

markermap.markers.append(m)
# copy above section for more markers




f = open("lab1.map",'w')
pickle.dump(markermap,f)

# gamma orientation offset for map
gamma = 0
pickle.dump(gamma,f)

print "Created map"
