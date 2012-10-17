#!/usr/bin/env python

import roslib; roslib.load_manifest('test_node')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
import numpy as np
import math

fileno = 0  

def process_frame(cimg):
    
    cv2.namedWindow("test")
    grey = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

    #imhist, bins = np.histogram(grey, 512, normed=True)
    #imhist_length = sum(imhist)
    #samples_probability3 = [float(h) / imhist_length for h in imhist]
    #test3 = -sum([p * math.log(p, 2) for p in samples_probability3 if p!= 0])
    grey2 = cv2.resize(grey,(64,36), interpolation = cv2.INTER_NEAREST)

    # Divide in x and calculate entropy for each band

    entropies = []
    max = 0


    for i in range(0,16):
        step = grey2.shape[1]/16
        imhist, bins = np.histogram(grey2[0:grey2.shape[0], step*i:step*(i+1)], 256, normed=True)
        #hist = cv2.calcHist(grey[0:grey.shape[0], step*i:step*(i+1)], [0], None, [256], [0,255])
        imhist_length = sum(imhist)
        samples_probability = [float(h) / imhist_length for h in imhist]
        entropy = -sum([p * math.log(p, 2) for p in samples_probability if p!= 0])
        #print entropy
        if entropy > max:
            max = entropy
            mid = int(10*step*(i+0.5))
        #entropies.append(entropy)

    print mid

    cv2.line(cimg, (mid, 0), (mid, cimg.shape[0]), [255, 0, 255], 1)
    cv2.imshow("test", cimg)
    cv2.waitKey(1)


    




 


    '''


    cv2.namedWindow('display3')
    cv2.imshow('display3', cimg2)
    #cv2.imshow('display3', 1)

    ch = cv2.waitKey(5)
    '''


def imgproc(d):
    global fileno
    global time
    global prevtime
    global fpsavg
    
    bridge = CvBridge()
    cvimg = bridge.imgmsg_to_cv(d,"bgr8")
    npimg = np.asarray(cvimg)
    #print "got image"
    #filepath = 'img'+str(fileno).zfill(5)+'.jpg'
    #print "Writing to '" + filepath  + "'"
    #cv2.imwrite(filepath, npimg)
    #cv2.imshow("Output",npimg)

    #time = rospy.Time.now()
    #fps = 1.0/(time.secs+time.nsecs/1E9-prevtime.secs-prevtime.nsecs/1E9)
    #fpsavg = (fpsavg*fileno + fps)/(fileno+1)
    #prevtime = time
    #fileno+=1 
    #print str(fpsavg)
    process_frame(npimg)
    #cv2.waitKey(1)

def connect():
    rospy.init_node('dronecontroller')
    #rospy.Subscriber('/image_/navdata',ardrone_autonomy.msg.Navdata,navdata)
    rospy.Subscriber('/ardrone/front/image_raw',Image,imgproc)
    #rospy.Subscriber('/ardrone/front/image_rect_color',Image,imgproc)

def nothing(*arg):
        pass
    

def initialise():
    global prevtime
    global time
    
    connect()
    prevtime = rospy.Time.now()
    time = rospy.Time.now()
    

def run():
    initialise()    
    prevtime = rospy.Time.now()
    time = rospy.Time.now()     
    rospy.spin()

if __name__ == '__main__':
    run()










