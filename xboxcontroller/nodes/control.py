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
                rospy.loginfo('Button press: %s' % (i,))
                if i >= 0 and i < len(actions):
                    actions[i]()

def takeoff():
    rospy.loginfo("Taking off")
    pub = rospy.Publisher('ardrone/takeoff',Empty);
    pub.publish()

def land():
    rospy.loginfo("Landing")
    pub = rospy.Publisher('ardrone/land',Empty);
    pub.publish()

def reset():
    rospy.loginfo("Resetting")
    pub = rospy.Publisher('ardrone/reset',Empty);
    pub.publish()

def fire_a():
    pub = rospy.Publisher('/xboxcontroller/button_a',Empty);
    pub.publish()

def fire_b():
    pub = rospy.Publisher('/xboxcontroller/button_b',Empty);
    pub.publish()
    
def fire_x():
    pub = rospy.Publisher('/xboxcontroller/button_x',Empty);
    pub.publish()

def fire_y():
    pub = rospy.Publisher('/xboxcontroller/button_y',Empty);
    pub.publish()

def fire_back():
    pub = rospy.Publisher('/xboxcontroller/button_back',Empty);
    pub.publish()  

def fire_start():
    pub = rospy.Publisher('/xboxcontroller/button_start',Empty);
    pub.publish()  

def fire_left_pad_button():
    pub = rospy.Publisher('/xboxcontroller/button_left_pad',Empty);
    pub.publish()  

def fire_right_pad_button():
    pub = rospy.Publisher('/xboxcontroller/button_right_pad',Empty);
    pub.publish()  
def nowt():
    pass


actions = {
    0: fire_a,
    1: fire_b,
    2: fire_x,
    3: fire_y,
    4: land,
    5: takeoff,
    6: fire_back,
    7: fire_start,
    8: reset,
    9: fire_left_pad_button,
    10: fire_right_pad_button
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
    # 0.5 scaling factor for sensitivity
    t.linear.x  = 0.5 * d.axes[1]
    t.linear.y  = 0.5 * d.axes[0]
    t.linear.z  = 0.5 * 0.5 * ( d.axes[2] - d.axes[5] )
    t.angular.z = 0.8 * d.axes[3]

    twist_pub = rospy.Publisher('cmd_vel',Twist)
    twist_pub.publish(t)


def init():
    rospy.init_node('xbox_controller')
    rospy.Subscriber('joy',Joy,processJoy)



def run():
    init()
    rospy.spin()

if __name__ == '__main__':
    run()
