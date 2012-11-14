#!/usr/bin/env python


import roslib; roslib.load_manifest('feature_track')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
import cv
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg as gm
import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D
import tf
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Empty
from nav_msgs.msg import Path
import math
import time
import threading
import os

    
class DisparityMapper:
    def __init__(self):
        self.disparity_map = None

    def cut(self, image, threshold):
        disparity = self.disparity_map
        for i in range(0, image.height-1):
            print i
            for j in range(0, image.width-1):
                # keep closer object
                if cv.GetReal2D(disparity,i,j) > threshold:
                    cv.Set2D(disparity,i,j,cv.Get2D(image,i,j))
        return disparity
        
    def cut_cv2(self, image, threshold):
        disparity = self.disparity_map_cv2.copy()
        return disparity

    def create_disparity_map(self):
        # loading the stereo pair
        print "Creating Disparity Map..."
        launch_time = time.time()
        print "Loading images"
        self.left  = cv.LoadImage('/home/alex/testData/0.jpg',cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.right = cv.LoadImage('/home/alex/testData/1.jpg',cv.CV_LOAD_IMAGE_GRAYSCALE)
        
        print "Converting to opencv"
        disparity_left  = cv.CreateMat(self.left.height, self.left.width, cv.CV_16S)
        disparity_right = cv.CreateMat(self.left.height, self.left.width, cv.CV_16S)

        # data structure initialization
        print "Initialising Data Structure"
        state = cv.CreateStereoGCState(16,2)
        # running the graph-cut algorithm
        print "Carrying out graph cut"
        cv.FindStereoCorrespondenceGC(self.left,self.right,
                                  disparity_left,disparity_right,state)
        print "Converting to opencv"
        disp_left_visual = cv.CreateMat(self.left.height, self.left.width, cv.CV_8U)
        print "Convert Scale"
        cv.ConvertScale( disparity_left, disp_left_visual, -20 );
        self.disparity_map = disp_left_visual
        cv.ShowImage('Disparity map2', self.disparity_map)
        cv.WaitKey(2)
        print "Done in ", time.time()-launch_time
        
    def create_disparity_map_cv2(self):
        # loading the stereo pair
        print "Creating Disparity Map (cv2)..."
        launch_time = time.time()
        
        print "Loading images"
        
        self.left  = cv2.imread('/home/alex/testData/3.jpg')
        self.right = cv2.imread('/home/alex/testData/4.jpg')
        
        min_disp = 16
        num_disp = 64-min_disp
        
        stereo = cv2.StereoBM(1)#, ndisparities = num_disp, SADWindowSize = 9)
        
        
        self.left = cv2.cvtColor(self.left, cv2.COLOR_BGR2GRAY)
        self.right = cv2.cvtColor(self.right, cv2.COLOR_BGR2GRAY)
        
        
        # disparity range is tuned for 'aloe' image pair
        window_size = 3
        
        '''
        stereo = cv2.StereoSGBM(minDisparity = min_disp, 
            numDisparities = num_disp, 
            SADWindowSize = window_size,
            uniquenessRatio = 10,
            speckleWindowSize = 100,
            speckleRange = 32,
            disp12MaxDiff = 1,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2,
            fullDP = False
        )
        disparity = stereo.compute(self.left, self.right).astype(np.float32) / 16.0
        print disparity
        
        #disparity = cv2.convertScaleAbs(disparity, alpha=-20)
        self.disparity_map_cv2 = disparity
        #print disparity    
        
        cv2.imshow('disparity', (disparity-min_disp)/num_disp)
        cv2.waitKey(2)
        
        print "Done in ", time.time()-launch_time
        '''
        
        print 'loading images...'
        imgL = cv2.pyrDown( cv2.imread('/home/alex/testData/0.jpg') )  # downscale images for faster processing
        imgR = cv2.pyrDown( cv2.imread('/home/alex/testData/1.jpg') )
        
        '''
        # disparity range is tuned for 'aloe' image pair
        window_size = 7
        min_disp = 16
        num_disp = 112-min_disp
        stereo = cv2.StereoSGBM(minDisparity = min_disp, 
            numDisparities = num_disp, 
            SADWindowSize = window_size,
            uniquenessRatio = 10,
            speckleWindowSize = 100,
            speckleRange = 32,
            disp12MaxDiff = 1,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2,
            fullDP = False
        )
        '''

        print 'computing disparity...'
        disp = stereo.compute(self.left,self.right).astype(np.float32) / 16.0

        #cv2.imshow('left', imgL)
        cv2.imshow('disparity', (disp-min_disp)/num_disp)
        print "Done in ", time.time()-launch_time 
        cv2.waitKey() 

    def draw_cut(self, notused):
        thresh = cv2.getTrackbarPos('Thresh', 'sliders')
        print "Cutting at ", thresh
        image = self.cut(self.left, thresh)
        cv.ShowImage('Disparity map', image)
        cv.WaitKey()
        print "Done"
        
    def draw_cut_cv2(self, notused):
        thresh = cv2.getTrackbarPos('Thresh', 'sliders')
        print "Cutting at ", thresh
        image = self.cut_cv2(self.left, thresh)
        cv.ShowImage('Disparity map', image)
        cv.WaitKey()
        print "Done"


def setup_trackbars():
    cv2.namedWindow('sliders')
    cv2.resizeWindow('sliders',400,480)
    cv2.createTrackbar('Thresh', 'sliders', 120, 500, nothing)
    cv2.waitKey(2)

def nothing(*arg):
    pass
    
def connect(m):
    rospy.init_node('Disparity_Mapper')
    #rospy.Subscriber('/ardrone/front/image_raw',Image,m.draw_cut_cv2)


def run():
    setup_trackbars()
    m = DisparityMapper()
    #m.create_disparity_map()
    m.create_disparity_map_cv2()
    # Initialise ROS node
    connect(m)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
