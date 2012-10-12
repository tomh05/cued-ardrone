#!/usr/bin/env python

import roslib; roslib.load_manifest('xboxcontroller')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

button_state = [0] * 11; # store previous button state


def processJoy(d): # callback function
    scanButtons(d)
    scanAxes(d) 

def scanButtons(d): # debounce buttons and fire events
    for i in range(11):
        if d.buttons[i] != button_state[i]:
            button_state[i] = d.buttons[i] 
            if button_state[i]:
                print i 
                if i >= 0 and i < len(actions):
                    actions[i]()

def takeoff():
    print "Taking off"
    pub = rospy.Publisher('/ardrone/takeoff',Empty);
    pub.publish()

def land():
    print "landing"
    pub = rospy.Publisher('/ardrone/land',Empty);
    pub.publish()

def reset():
    print "resetting"
    pub = rospy.Publisher('/ardrone/reset',Empty);
    pub.publish()
def nowt():
    pass


actions = {
    0: nowt,
    1: reset,
    2: takeoff,
    3: land,
}

def scanAxes(d):
#    d.axes[0] # left left right
#    d.axes[1] # left up down
#    d.axes[2] # left trigger
#    d.axes[3] # right X
#    d.axes[4] # right Y
#    d.axes[5] # right trigger
#    d.axes[6] # D-pad X
#    d.axes[7] # D-pad Y
    t = Twist()
    t.linear.x  = d.axes[1]
    t.linear.y  = d.axes[0]
    t.linear.z  = 0.5 * ( d.axes[2] - d.axes[5] )
    t.angular.z = d.axes[3]

    twist_pub = rospy.Publisher('cmd_vel',Twist)
    twist_pub.publish(t)


def init():
    rospy.init_node('xbox_controller')
    rospy.Subscriber('/joy',Joy,processJoy)



def run():
    init()
    rospy.spin()

if __name__ == '__main__':
    run()
