#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ardrone_autonomy.msg import Navdata

last_navdata = Navdata()
old_buttons = [0,] * 10

takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty)
def takeoff():
    global takeoff_pub
    rospy.loginfo('Taking off')
    takeoff_pub.publish(Empty())

land_pub = rospy.Publisher('ardrone/land', Empty)
def land():
    global land_pub
    rospy.loginfo('Landing')
    land_pub.publish(Empty())

def navdata_cb(data):
    global last_navdata
    last_navdata = data

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
def joy_cb(data):
    global last_navdata, old_buttons

    axes = data.axes
    buttons = data.buttons

    # Did the 'XBox' button change state to 'on'?
    if buttons[8] and not old_buttons[8]:
        if last_navdata.state <= 2:
            # Drone is initialised or landed so take off
            takeoff()
        else:
            # If in doubt: land
            land()

    # Request a new twist based on the controller values
    t = Twist()

    t.angular.z = axes[3]   # right analogue stick X
    t.linear.x = axes[4]    # right analogue stick Y

    cmd_vel_pub.publish(t)

    old_buttons = buttons

def main():
    rospy.init_node('joystick_controller', anonymous=True)

    rospy.Subscriber('joy', Joy, joy_cb)
    rospy.Subscriber('ardrone/navdata', Navdata, navdata_cb)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

# vim:sw=4:sts=4:et
