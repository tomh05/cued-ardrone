#!/usr/bin/env python

import roslib; roslib.load_manifest('feature_track')
import rospy
import ardrone_autonomy.msg
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg

    
def stackImagesVertically(top_image, bottom_image):
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
    
class FeatureTracker:
    def __init__(self):
        self.previous = None
        self.frameskip = 0
        self.calibrated = False
        cv2.namedWindow("track")
    
    def update_previous(self, thing):
        """Takes a cv2 numpy array image and sets the FeatureTracker
        previous image to it"""
        #print('Now: %s' % (thing,))
        #print('Previous: %s' % (self.previous))
        self.previous = thing
        
    def featureTrack(self, img):
        """Takes a cv2 numpy array image and compared to a previously
        buffered image. Features are extracted from each frame, 
        undistorted and matched."""
        
        # Initialise previous image buffer
        if self.previous == None:
            self.update_previous(img)

        # Skip frames. Need to add ROS parameter to allow setting
        self.frameskip += 1
        if self.frameskip == 4:
            self.frameskip = 0
            return
            
        # Convert working images to monochrome
        grey_previous = cv2.cvtColor(self.previous, cv2.COLOR_BGR2GRAY)
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Create feature detector and extracts points
        fd = cv2.FeatureDetector_create('SIFT')
        pts1 = fd.detect(grey_previous)
        pts2 = fd.detect(grey_now)

        # Create a descriptor extractor and describe points
        de = cv2.DescriptorExtractor_create('SIFT')
        kp1, desc1 = de.compute(grey_previous, pts1)
        kp2, desc2 = de.compute(grey_now, pts2)
        
        # Bottom out if failed to get features
        if desc1 == None or desc2 == None or len(desc1) == 0 or len(desc2) == 0:
            return
        
        # Create descriptor matcher and match features
        dm = cv2.DescriptorMatcher_create('BruteForce')
        matches = dm.match(desc1, desc2)

        # Produce ordered arrays of paired points
        i1_indices = list(x.queryIdx for x in matches)
        i2_indices = list(x.trainIdx for x in matches)
        kp1_array = np.array(list(x.pt for x in kp1))
        kp2_array = np.array(list(x.pt for x in kp2))
        i1_pts = kp1_array[i1_indices,:]
        i2_pts = kp2_array[i2_indices,:]
        
        # Check for sufficient pairs for fundamental matrix extraction
        if (len(i1_pts) > 4):
            if (len(i2_pts) > 4): # this is redundant as matched lists
                
                if self.calibrated:
                    # Undistort points using calibration data
                    i1_mat = np.array([i1_pts])
                    i2_mat = np.array([i2_pts])                    
                    i1_pts_undistorted = cv2.undistortPoints(i1_mat, self.cameraMatrix, self.distCoeffs, P=self.P)[0]
                    i2_pts_undistorted = cv2.undistortPoints(i2_mat, self.cameraMatrix, self.distCoeffs, P=self.P)[0]
                else:
                    # Use distorted points as calibration missing
                    print "WARNING: No calibration info. Using distorted feature positions"
                    i1_pts_undistorted = i1_pts
                    i2_pts_undistorted = i2_pts

                # Extract fundamental matrix
                H, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_RANSAC)
                # H contains fundamental matrix
                # mask is a binary mask of points fitting the matrix

                # Plot tracked features on stacked images
                ml = list(mask.flat)                
                img2 = stackImagesVertically(grey_previous, grey_now)
                imh = grey_previous.shape[0]
                idx = 0
                for p1, p2 in zip(i1_pts_undistorted, i2_pts_undistorted):
                    idx += 1
                    if ml[idx - 1] == 0:
                        continue
                    cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (255, 0 , 255), 1)
                cv2.imshow("track", img2)
                cv2.waitKey(1)
            
        # Update previous image buffer
        self.update_previous(img)


    def imgproc(self, d):
        """Converts the ROS published image to a cv2 numpy array
        and passes to FeatureTracker"""
        # ROS to cv image
        bridge = CvBridge()
        cvimg = bridge.imgmsg_to_cv(d,"bgr8")
        # cv to cv2 numpy array image
        npimg = np.asarray(cvimg)
        # Pass to FeatureTrackerr
        self.featureTrack(npimg)
        
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        if not self.calibrated:
            if len(ci.D) == 0:
                return
            self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
            self.distCoeffs = np.array([ci.D], dtype=np.float32)
            self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
            self.calibrated = True      
  
def connect(m):
    rospy.init_node('Feature_Tracker')
    rospy.Subscriber('/ardrone/front/image_raw',Image,m.imgproc)
    rospy.Subscriber('/ardrone/front/camera_info',sensor_msgs.msg.CameraInfo, m.setCameraInfo) 


def run():
    # Initialise tracker
    m = FeatureTracker()
    # Initialise ROS node
    connect(m)
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
