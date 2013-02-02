#!/usr/bin/env python

import roslib; roslib.load_manifest('xboxcontroller')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def scanButtons():
    exiting = False
    while (not exiting):        
        key = raw_input('Enter your input:')
        print key
        if key == 'a':
            print "Firing a"
            fire_a()
        elif key == 'b':
            print "Firing b"
            fire_b()
        elif key == 'x':
            print "Firing x"
            fire_x()
        elif key == 'y':
            print "Firing y"
            fire_y()
        elif key == '':
            exiting = True
            


def takeoff():
    rospy.loginfo("Taking off")
    pub = rospy.Publisher('/ardrone/takeoff',Empty);
    pub.publish()

def land():
    rospy.loginfo("Landing")
    pub = rospy.Publisher('/ardrone/land',Empty);
    pub.publish()

def reset():
    rospy.loginfo("Resetting")
    pub = rospy.Publisher('/ardrone/reset',Empty);
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



def init():
    rospy.init_node('xbox_controller')
    scanButtons()


def run():
    init()
    

if __name__ == '__main__':
    run()
    

