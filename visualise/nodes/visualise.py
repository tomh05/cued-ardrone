#!/usr/bin/env python

import roslib; roslib.load_manifest('visualise')
import rospy
import cv2 
import cv
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from custom_msgs.msg import StampedFeaturesWithImage
from custom_msgs.msg import Visualisation
import time
    

class Visualiser:
    def __init__(self):
        self.image_buffer = []
        self.pts_draw_buffer = []
        self.bridge = CvBridge()
        cv2.namedWindow('track')
        self.connect()
                
    def connect(self):
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        rospy.Subscriber('/feature_extractor/features',StampedFeaturesWithImage,self.on_got_image)
        rospy.Subscriber('/scan/visualisation', Visualisation, self.on_got_draw)

    def on_got_image(self, sfwi):
        if len(self.image_buffer) > 20:
            self.image_buffer.pop(0)
            self.pts_draw_buffer.pop(0)
        self.image_buffer.append(sfwi.image)
        self.pts_draw_buffer.append(sfwi.points)
    
    def on_got_draw(self, draw):
        grey_previous, grey_now, draw1, draw2 = self.lookup_images(draw.header1, draw.header2)
        if grey_previous == None or grey_now == None:
            print "Buffer underrun"
            return
        
        pts1, pts2, repro1, repro2 = self.decode_message(draw)
        
        pts1_draw = self.distort_points(pts1.T)
        pts2_draw = self.distort_points(pts2.T)
        
        """====================================================================
        # Plot fully tracked points
        # Only that fit with the calculated geometry are plotted
        # Note: This plots undistorted points on the distorted image
        ===================================================================="""
        img2 = np.vstack((grey_previous, grey_now))
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        imh = grey_previous.shape[0]
        
        # Draw lines linking fully tracked points
        for p1, p2 in zip(pts1_draw, pts2_draw):
            cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (0, 255 , 255), 1)
        
        # Reproject triangulated points
        for p in repro1:
            cv2.circle(img2, (int(p[0]),int(p[1])), 3, (255, 0, 0), 1)
        for p in repro2:
            cv2.circle(img2, (int(p[0]),int(p[1]+imh)), 3, (0, 255, 0), 1)
        
        # Draw
        cv2.imshow("track", img2)
        
        # Render Windows
        cv2.waitKey(10)
        
    @staticmethod
    def to_np(msg):
        return np.reshape(np.array(msg), (-1, 2))
    
    def distort_points(self, pts):
        
        k = self.distCoeffs
        k1 = k[0][0] # Radial coeff 1
        k2 = k[0][1] # Radial coeff 2
        p1 = k[0][2] # Tangential coeff 1
        p2 = k[0][3] # Tangential coeff 2
        
          
        K = self.cameraMatrix
        fx = K[0,0]
        fy = K[1,1]
        cx = K[0,2]                    
        cy = K[1,2]
        
        x_dash = (pts[0] - cx)/fx;   
        y_dash = (pts[1] - cy)/fy;
        
        # Precalc terms (Note: This is significantly simpler than if we had all 8 distCoeffs)
        r_2 = x_dash*x_dash + y_dash*y_dash
        r_4 = r_2*r_2
        terms = (1+k1*r_2+k2*r_4)
        
        # Distortion
        x_dash2 = x_dash*terms + 2*p1*x_dash*y_dash + p2*(r_2+2*x_dash*x_dash)
        y_dash2 = y_dash*terms + 2*p2*x_dash*y_dash + p1*(r_2+2*y_dash*y_dash)
        
        # To pixel
        u = fx*x_dash2+cx
        v = fy*y_dash2+cy
          
        return np.array([u,v]).T
    
    @staticmethod
    def decode_message(msg):
        # Need to re-numpy the array kp & descriptors
        pts1 = np.reshape(np.array(msg.points1), (-1, 2))
        pts2 = np.reshape(np.array(msg.points2), (-1, 2))
        repro1 = np.reshape(np.array(msg.reprojected1), (-1, 2))
        repro2 = np.reshape(np.array(msg.reprojected2), (-1, 2))
        return pts1, pts2, repro1, repro2
    
    @staticmethod
    def stack_images_vertically(top_image, bottom_image):
        """Takes two cv2 numpy array images top_image, bottom_image
           Returns a cv2 numpy array image of the two stacked vertically
           Image dimensions need not match, output it left aligned"""
        # Get image dimensions
        h1, w1 = top_image.shape[:2]
        h2, w2 = bottom_image.shape[:2]
        # Create an empty array that is bounding box size of image stack
        stacked = np.zeros((h1+h2, max(w1,w2)), np.uint8)
        # Drop in the top_image
        stacked[:h1, :w1] = top_image
        # Drop in the bottom_image
        stacked[h1:h1+h2, :w2] = bottom_image
        return stacked
        
    def lookup_images(self, header1, header2):
        grey_previous = None
        grey_now = None
        draw1 = None
        draw2 = None
        for i, image in enumerate(self.image_buffer):
            if image.header.frame_id == header1.frame_id and image.header.stamp == header1.stamp:
                grey_previous = self.ros_to_greyscale_np(image)
                draw1 = self.to_np(self.pts_draw_buffer[i])
            if image.header.frame_id == header2.frame_id and image.header.stamp == header2.stamp:
                grey_now = self.ros_to_greyscale_np(image)
                draw2 = self.to_np(self.pts_draw_buffer[i])
        return grey_previous, grey_now, draw1, draw2
    
    def ros_to_greyscale_np(self,image):
        # ROS to monochrome cv image
        cvimg = self.bridge.imgmsg_to_cv(image,"mono8")
        # cv to cv2 numpy array image
        return np.asarray(cvimg)
        
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
        self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
        self.distCoeffs = np.array([ci.D], dtype=np.float32)
        self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
        print "                    Calibration Initialised\r\n"
        
def run():
    rospy.init_node('Visualisation')
    # Initialise controller
    v = Visualiser()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
