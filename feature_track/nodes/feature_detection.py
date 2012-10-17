#!/usr/bin/env python

import roslib; roslib.load_manifest('test_node')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
import numpy

fileno = 0


def navdata(d): 
    #print d.rotX
    pass
    
    
    
def features(cimg):
    global fileno
    '''
    grey = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
    max_corners = 25
    quality_level = 0.01
    min_distance = 10
    
    
    corners = cv2.goodFeaturesToTrack(grey, max_corners, quality_level, min_distance)
    cv2.imwrite("/home/alex/bags/test.jpg", cimg)
    
    
    #for c in corners:
    #    print c
    #    print c[0]
    
    
    cv2.namedWindow('corners')
    
    cimg2=cimg.copy()
    if len(corners) > 0:
        for c in corners:
            cv2.circle(cimg2, (c[0][0],c[0][1]) , 5, (255, 0, 255))
        
    #cv2.imshow('corners', cimg2)
    cv2.imshow('corners', cimg2)
    cv2.waitKey(1)
    '''
    cv2.imwrite('/home/alex/bags/tempbags/frame_'+str(fileno).zfill(5)+'.jpg', cimg)
    fileno+=1
    
    
    
    

def imgproc(d):
    bridge = CvBridge()
    cvimg = bridge.imgmsg_to_cv(d,"bgr8")
    npimg = numpy.asarray(cvimg)
    features(npimg)

def connect():
    rospy.init_node('dronecontroller')
    #rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,navdata)
    rospy.Subscriber('/ardrone/front/image_raw',Image,imgproc)


def run():
    connect()
    cv2.namedWindow("Output")
    rospy.spin()

if __name__ == '__main__':
    run()
