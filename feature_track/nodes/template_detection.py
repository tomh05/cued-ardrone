#!/usr/bin/env python

import roslib; roslib.load_manifest('test_node')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
import numpy



def navdata(d): 
    #print d.rotX
    pass
    
def template_detect(cimg):
    global template
    
    grey = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
    
    results = cv2.matchTemplate(grey, template, cv2.TM_CCORR)
    
    (min_x, max_y, minloc, maxloc) = cv2.minMaxLoc(results)
    
    (x, y) = minloc
    
    print minloc
    print min_x
    print max_y
    
    #print template
    cimg2 = cimg.copy()
    cv2.namedWindow('template')
    #if results[minloc[0]][minloc[1]] < 1:
    #    cv2.circle(cimg2, minloc , 5, (255, 0, 255))
    #cv2.imshow('template', cimg2)

    cv2.waitKey(1)
    
    print results
    
    

def imgproc(d):
    bridge = CvBridge()
    cvimg = bridge.imgmsg_to_cv(d,"bgr8")
    npimg = numpy.asarray(cvimg)
    template_detect(npimg)

def connect():
    rospy.init_node('dronecontroller')
    #rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,navdata)
    rospy.Subscriber('/ardrone/front/image_rect_color',Image,imgproc)


def run():
    global template
    connect()
    template = cv2.imread('/home/alex/ImageDump/rect/template.jpg')
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow("Output")
    rospy.spin()

if __name__ == '__main__':
    run()
