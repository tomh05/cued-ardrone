#!/usr/bin/env python

import roslib; roslib.load_manifest('test_node')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
import numpy
import math

expected_zone_inner = [300,160,40,40]
expected_zone_outer = [280,140,80, 80]
fileno = 0
fpsavg = 0
prevtime = 0
time = 0

def navdata(d): 
    #print d.rotX
    pass
    
def grow_square(square, factor):
    prevW = square[2]
    prevH = square[3]
    square[2] *= factor
    square[3] *= factor
    square[0] -= (square[2] - prevW)/2
    square[1] -= (square[3] -prevH)/2

def process_frame(cimg):
    global expected_zone_inner
    global expected_zone_outer
    grey = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

    b_enb = cv2.getTrackbarPos('B enable', 'sliders')
    thresh = cv2.getTrackbarPos('B thresh', 'sliders')
    thrs1 = cv2.getTrackbarPos('C thrs1', 'sliders')
    thrs2 = cv2.getTrackbarPos('C thrs2', 'sliders')
    g_enb = cv2.getTrackbarPos('G enable', 'sliders')
    sigma = cv2.getTrackbarPos('G sigma', 'sliders')/10.0
    if sigma == 0:
        cv2.setTrackbarPos('G sigma', 'sliders', 1)
        sigma = 0.1
    minlength = cv2.getTrackbarPos('H minlen', 'sliders')
    maxgap = cv2.getTrackbarPos('H maxgap', 'sliders')


    if b_enb == 1:
        retval, binary = cv2.threshold(grey, thresh, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, thrs1, thrs2)
    else:
        edges = cv2.Canny(grey, thrs1, thrs2)
        
    cv2.namedWindow('edges')
    cv2.imshow('edges', edges)

    if g_enb == 1:
        edges = cv2.GaussianBlur(edges, (0,0), sigma)

    cv2.namedWindow('edgesBlur')
    cv2.imshow('edgesBlur', edges)
    cv2.waitKey(1)

    seq, heirarchy=cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cimg2 = cimg.copy()
    cv2.drawContours(cimg2, seq, -1, (0, 255, 0))
    #cv2.namedWindow('display')
    #cv2.imshow('display', cimg2)l
    #cv2.waitKey(5)

    #cimg2 = cimg.copy()
    seq_ext, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cimg2, seq_ext, -1, (255, 0, 0))
    cv2.namedWindow('display2')
    cv2.imshow('display2', cimg2)

    sqrs = []

    cimg2 = cimg.copy()
    for s in seq:
        #do not take into account external countours
        if not (s==seq_ext):                               # This is dodgy
            perim = cv2.arcLength(s, True) #contour perimeter
            area =cv2.contourArea(s) #contour area      
            polygon=cv2.approxPolyDP(s,perim*0.02, True)
            sqr=cv2.boundingRect(polygon)
            sqrs.append(sqr)
        else:
            #move on to the next outer contour      
            seq_ext=seq_ext.h_next()   
            #h_next: points to sequences on the same level
            seq=seq.h_next()



    # Get Lines at all angles
    edges = cv2.Canny(grey, thrs1, thrs2)
    
    '''
    lines = cv2.HoughLinesP(edges, 1, math.pi/320, 2, None, minlength, maxgap)
    linesa = []
    linesb = []
    linesc = []
    '''
    stags = []
    sqrs2 = []
    sqrs3 = []
    sqrs4 = []



    # Filter squares
    '''
    for i, s in enumerate(sqrs):
        # Wrong proportions
        if s[2]<s[3] and 2*s[2]>s[3] and s[2] > 64 and s[2] < 640 - 16 and s[3]>64 and s[3] < 360 - 16:
            if expected_zone[0]!=-1:
                if s[0] > expected_zone[0]-64 and s[0] < expected_zone[0]+64 and s[1] > expected_zone[1]-64 and s[1] < expected_zone[1]+64 and s[0]+s[2] > expected_zone[0]+expected_zone[2]-64 and s[0]+s[2] < expected_zone[0]+expected_zone[2]+64 and s[1]+s[3] > expected_zone[1]+expected_zone[3]-64 and s[1]+s[3] < expected_zone[1]+expected_zone[3]+64:
                    sqrs2.append(s)
            else:
                sqrs2.append(s)
        # Moved too much
    '''
    
    # wider than tall
    sqrs2 = [s for s in sqrs if s[2] < s[3]]
    # excessively thin
    sqrs2 = [s for s in sqrs2 if 2*s[2] > s[3]]
    # too small
    sqrs2 = [s for s in sqrs2 if s[2] > 64 or s[3] > 64]
    
    # Within Expected Region
    if expected_zone_inner != None:
        sqrs3 = [s for s in sqrs2 if s[0] < expected_zone_inner[0]]
        sqrs3 = [s for s in sqrs3 if s[0] > expected_zone_outer[0]]
        sqrs3 = [s for s in sqrs3 if s[1] < expected_zone_inner[1]]
        sqrs3 = [s for s in sqrs3 if s[1] > expected_zone_outer[1]]
        sqrs3 = [s for s in sqrs3 if (s[0] + s[2] > expected_zone_inner[0]+expected_zone_inner[2] and s[0] + s[2] < expected_zone_outer[0] + expected_zone_outer[2] and s[1] + s[3] > expected_zone_inner[1] +expected_zone_inner[3] and s[1] + s[3] < expected_zone_outer[1]+expected_zone_outer[3])]

    '''
    for i, s1 in enumerate(sqrs3):
        for j, s2 in enumerate(sqrs3):
            if i != j:
                if s1[0] > s2[0] and s1[1] > s2[1] and s1[0]+s1[2] < s2[0]+s2[2] and s1[1]+s1[3] < s2[1]+s2[3]:
                    sqrs4.append(s2)
    sqrs4 = [x for x in sqrs2 if x not in sqrs4]
    '''
    sqrs4 = sqrs3


    best = 999
    selected_square_temp = None
    
    if len(sqrs4) > 0:
        selected_square_temp = sqrs4[0]
    
    '''
    for s in sqrs3:
        i = ((s[2]/expected_zone[2])**2+(s[3]/expected_zone[3])**2 -2)**2
        if i < best:
            best = i
            #if (i < 2.5) or expected_zone[0]==-1:
            expected_zonetemp = s
    '''
            
    # Relax expected_zone
    if selected_square_temp == None:
        grow_square(expected_zone_outer, 1.01)
        grow_square(expected_zone_inner, 1/1.01)
        '''
        prevW = expected_zone_outer[2]
        prevH = expected_zone_outer[3]
        expected_zone_outer[2] = min(expected_zone_outer[2] * 1.01, 640)
        expected_zone_outer[3] = min(expected_zone_outer[3] * 1.01, 360)
        expected_zone_outer[0] -= (expected_zone_outer[2] - prevW)/2
        expected_zone_outer[1] -= (expected_zone_outer[3] -prevH)/2
        prevW = expected_zone_inner[2]
        prevH = expected_zone_inner[3]
        expected_zone_inner[2] /= 1.01
        expected_zone_inner[3] /= 1.01
        expected_zone_inner[0] -= (expected_zone_inner[2] - prevW)/2
        expected_zone_inner[1] -= (expected_zone_inner[3] -prevH)/2
        '''


    '''
    for i, s in enumerate(sqrs2):
    stags.append(0)
    if lines!= None:
        #Corner a
        for l in lines[0]:
        # Check aligned in x
        if l[0] < s[0] + 4:
            if l[0] > s[0] - 4:
                # Check aligned in y
                if l[1] < s[1]+s[3] + 4:
                    if l[1] > s[1]+s[3] - 4:
                        linesa.append(l)
                        stags[i]+=stags[i]
        #Corner b
        for l in lines[0]:
        # Check aligned in x
        if l[0] < s[0]+s[2] + 4:
            if l[0] > s[0]+s[2] - 4:
                # Check aligned in y
                if l[1] < s[1]+s[3] + 4:
                    if l[1] > s[1]+s[3] - 4:
                        linesb.append(l)
                        stags[i]+=stags[i]              
    '''


    cimg2 = cimg.copy()
    
    if expected_zone_inner != None:
        # Plot expected region
        pt1 = (int(expected_zone_inner[0]),int(expected_zone_inner[1]))
        pt2 = (int(expected_zone_inner[0]+expected_zone_inner[2]),int(expected_zone_inner[1]+expected_zone_inner[3]))
        cv2.rectangle(cimg2, pt1, pt2, (180,0,180), 1)
        pt1 = (int(expected_zone_outer[0]),int(expected_zone_outer[1]))
        pt2 = (int(expected_zone_outer[0]+expected_zone_outer[2]),int(expected_zone_outer[1]+expected_zone_outer[3]))
        cv2.rectangle(cimg2, pt1, pt2, (200,0,200), 1)
    
    for sqr in sqrs:
        pt1 = (sqr[0],sqr[1])
        pt2 = (sqr[0]+sqr[2],sqr[1]+sqr[3])
        cv2.rectangle(cimg2, pt1, pt2, (0,0,0), 1)


    for sqr in sqrs2:
        pt1 = (sqr[0],sqr[1])
        pt2 = (sqr[0]+sqr[2],sqr[1]+sqr[3])
        cv2.rectangle(cimg2, pt1, pt2, (127,127,127), 1)

    for sqr in sqrs3:
        pt1 = (sqr[0],sqr[1])
        pt2 = (sqr[0]+sqr[2],sqr[1]+sqr[3])
        cv2.rectangle(cimg2, pt1, pt2, (200,200,200), 1)

    if selected_square_temp != None:
        pt1 = (selected_square_temp[0],selected_square_temp[1])
        pt2 = (selected_square_temp[0]+selected_square_temp[2],selected_square_temp[1]+selected_square_temp[3])
        cv2.rectangle(cimg2, pt1, pt2, (0,255,255), 1)
        selected_square = selected_square_temp
        expected_zone_inner[0] = selected_square[0]
        expected_zone_inner[1] = selected_square[1]
        expected_zone_inner[2] = selected_square[2]
        expected_zone_inner[3] = selected_square[3]
        expected_zone_outer[0] = selected_square[0]
        expected_zone_outer[1] = selected_square[1]
        expected_zone_outer[2] = selected_square[2]
        expected_zone_outer[3] = selected_square[3]
        grow_square(expected_zone_inner, 1/2.0)
        grow_square(expected_zone_outer, 2.0)      
    
    '''    
    pt1 = (expected_zone[0],expected_zone[1])
    pt2 = (expected_zone[0]+expected_zone[2],expected_zone[1]+expected_zone[3])
    cv2.rectangle(cimg2, pt1, pt2, (200,200,200), 1)
    '''    

    #if lines!= None:
    #    for line in lines[0]:
    #        pt1 = (line[0],line[1])
    #        pt2 = (line[2],line[3])
    #        cv2.line(cimg2, pt1, pt2, (0,255,255), 1)
    '''
    if len(linesa)!= None:
    for line in linesa:
        pt1 = (line[0],line[1])
        pt2 = (line[2],line[3])
        cv2.line(cimg2, pt1, pt2, (255,0,255), 1)

    if len(linesb)!= None:
    for line in linesb:
        pt1 = (line[0],line[1])
        pt2 = (line[2],line[3])
        cv2.line(cimg2, pt1, pt2, (0,255,255), 1)
    '''


    cv2.namedWindow('display3')
    cv2.imshow('display3', cimg2)
    #cv2.imshow('display3', 1)

    ch = cv2.waitKey(5)


def imgproc(d):
    global fileno
    global time
    global prevtime
    global fpsavg
    
    bridge = CvBridge()
    cvimg = bridge.imgmsg_to_cv(d,"bgr8")
    npimg = numpy.asarray(cvimg)
    #print "got image"
    filepath = 'img'+str(fileno).zfill(5)+'.jpg'
    #print "Writing to '" + filepath  + "'"
    #cv2.imwrite(filepath, npimg)
    #cv2.imshow("Output",npimg)

    time = rospy.Time.now()
    fps = 1.0/(time.secs+time.nsecs/1E9-prevtime.secs-prevtime.nsecs/1E9)
    fpsavg = (fpsavg*fileno + fps)/(fileno+1)
    prevtime = time
    fileno+=1 
    print str(fpsavg)
    process_frame(npimg)
    #cv2.waitKey(1)

def connect():
    rospy.init_node('dronecontroller')
    #rospy.Subscriber('/image_/navdata',ardrone_autonomy.msg.Navdata,navdata)
    #rospy.Subscriber('/ardrone/front/image_raw',Image,imgproc)
    rospy.Subscriber('/ardrone/front/image_rect_color',Image,imgproc)

def nothing(*arg):
        pass

def setup_trackbars():
    cv2.namedWindow('sliders')
    cv2.resizeWindow('sliders',400,480)
    cv2.createTrackbar('B enable', 'sliders', 0, 1, nothing)
    cv2.createTrackbar('B thresh', 'sliders', 50, 255, nothing)
    cv2.createTrackbar('C thrs1', 'sliders', 50, 500, nothing)
    cv2.createTrackbar('C thrs2', 'sliders', 275, 500, nothing)
    cv2.createTrackbar('G enable', 'sliders', 1, 1, nothing)
    cv2.createTrackbar('G sigma', 'sliders', 5, 250, nothing)
    cv2.createTrackbar('H minlen', 'sliders', 10, 100, nothing)
    cv2.createTrackbar('H maxgap', 'sliders', 2, 100, nothing)
    

def initialise():
    global prevtime
    global time
    
    connect()
    prevtime = rospy.Time.now()
    time = rospy.Time.now() 
    setup_trackbars()
    

def run():
    initialise()    
    prevtime = rospy.Time.now()
    time = rospy.Time.now()     
    rospy.spin()

if __name__ == '__main__':
    run()










