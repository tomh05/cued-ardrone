#!/usr/bin/env python

'''
quickstart.py
Tests the ease of use of PositionController.py
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8

from time import time, sleep
from PositionController import PositionController

def run():
	cmdpub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('quickcontroller')
	landpub = rospy.Publisher('/ardrone/land', Empty)
	resetpub = rospy.Publisher('/ardrone/reset', Empty)
	takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
	camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
	sleep(1)
	resetpub.publish(Empty())
	sleep(1)
	resetpub.publish(Empty())
	cmdpub.publish(Twist())
	camselectclient(1); print 'select bottom camera'
	dpw=(0,0,1)
	dyw=0
	cpw=dpw
	cyw=dyw
	takeoffpub.publish(Empty()); print 'takeoff'
	sleep(8)
	pc=PositionController(dpw,dyw,cpw,cyw); print 'construct position controller'
	sleep(1)
	pc.pc_timer_init(); print 'start position controller timer'
	sleep(1)
	dpw=(-1,0,1); pc.dpw_handler(dpw);
	sleep(5)
	dpw=(0,0,1); pc.dpw_handler(dpw);
	sleep(18)
	landpub.publish(Empty())
	rospy.spin()
	
	
	

if __name__=='__main__':
	run()
