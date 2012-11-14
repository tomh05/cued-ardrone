#!/usr/bin/env python

#===================Changelog===========================================
# Initial Creation
# Added undistort
# Added 3D via fundamental matrix
# Added 3D via essential matrix
# Efficiency improvements by buffering now to previous
#=======================================================================

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
        self.fd = cv2.FeatureDetector_create('ORB')
        self.de = cv2.DescriptorExtractor_create('FREAK')
        self.dm = cv2.DescriptorMatcher_create('BruteForce')
        self.cameraMatrix =  np.array([[561.245,0.,306.914], [0.,563.104,190.375], [0.,0.,1]], dtype=np.float32)
        self.distCoeffs = np.array([-0.5233,0.3020,-0.0068,0.0024,0.], dtype=np.float32)
        self.P = np.array([[458.229,0.,304.262,0.],[0,530.191,189.659,0],[0.,0.,1.,0.]], dtype=np.float32)
        cv2.namedWindow("track")
        
    def featureTrack(self):
        """Takes a cv2 numpy array image and compared to a previously
        buffered image. Features are extracted from each frame, 
        undistorted and matched."""

        img1 = cv2.imread("/home/alex/1.jpg")
        img2 = cv2.imread("/home/alex/2.jpg")      
            
        # Convert working images to monochrome
        grey_previous = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        grey_now = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        # Extracts points
        pts1 = self.fd.detect(grey_previous)
        pts2 = self.fd.detect(grey_now)

        # Create a descriptor extractor and describe points
        
        kp1, desc1 = self.de.compute(grey_previous, pts1)
        kp2, desc2 = self.de.compute(grey_now, pts2)
        
        # Bottom out if failed to get features
        if desc1 == None or desc2 == None or len(desc1) == 0 or len(desc2) == 0:
            print "No Features Found"
            return
        
        # Create descriptor matcher and match features
        
        matches = self.dm.match(desc1, desc2)

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
                # Undistort points using calibration data
                i1_mat = np.array([i1_pts])
                i2_mat = np.array([i2_pts])                    
                i1_pts_undistorted = cv2.undistortPoints(i1_mat, self.cameraMatrix, self.distCoeffs)[0]
                i2_pts_undistorted = cv2.undistortPoints(i2_mat, self.cameraMatrix, self.distCoeffs, P=self.P)[0]
                print i1_pts_undistorted
                print i1_pts_undistorted.shape
                print "---"
                print i2_pts_undistorted
                print i2_pts_undistorted.shape
                #i1_pts_undistorted = i1_pts
                #i2_pts_undistorted = i2_pts

                # Extract fundamental matrix
                F, mask = cv2.findFundamentalMat(i1_pts_undistorted, i2_pts_undistorted, cv2.FM_LMEDS, param1 = 1., param2 = 0.99)
                # F contains fundamental matrix
                # mask is a binary mask of points fitting the matrix                           
                
                # Filter points that fit F
                #print i1_pts_undistorted
                i1_pts_undistorted = np.array([i1_pts_undistorted])
                #print i1_pts_undistorted
                #print "--"
                i2_pts_undistorted = np.array([i2_pts_undistorted])
                #print i2_pts_undistorted
                i1_pts_corr, i2_pts_corr = cv2.correctMatches(F, i1_pts_undistorted, i2_pts_undistorted)
                #i1_pts_corr, i2_pts_corr = i1_pts_undistorted, i2_pts_undistorted
                
                print "i1_pts_corr"
                print i1_pts_corr[(i1_pts_corr>=0)]
                print "---"
                print i2_pts_corr[(i2_pts_corr>=0)]
                
                # Filter nans
                i1_pts_corr = np.reshape(i1_pts_corr[(i1_pts_corr>=0)], (-1, 2))
                i2_pts_corr = np.reshape(i2_pts_corr[(i2_pts_corr>=0)], (-1, 2))
                
                #print "i1_pts_corr"
                #print i1_pts_corr
                
                '''
                # Calculate via essential matrix
                '''
                
                print "F"
                print F
                
                E = self.cameraMatrix.transpose().dot(F)
                E = E.dot(self.cameraMatrix) 
                E /= E[2,2]
                print "E"
                print E

                
                W = np.array([[0, -1, 0],[1, 0, 0], [0, 0, 1]])
                Z = np.array([[0, 1, 0],[-1, 0, 0], [0, 0, 0]])
                #print "W"
                #print W   
                #print "W'"
                #print W.transpose() 
                #print "Z"
                #print Z
                               
                U,SIGMA,V = np.linalg.svd(E)
                #print "True SIGMA"
                #print SIGMA
                sigma_avg = (SIGMA[0]+SIGMA[1])/2
                #print "sigma avg"
                #print sigma_avg
                #SIGMA = np.diag([sigma_avg, sigma_avg, 0])
                SIGMA = np.diag(SIGMA)
                SIGMA[2,2] = 0
                if SIGMA[0,0] < 0.7*SIGMA[1,1] or SIGMA[1,1] < 0.7*SIGMA[0,0]:
                    print "WARNING: Disparate singular values"
                E2 = U.dot(SIGMA).dot(V)
                print "E2"
                print E2
                #print "U"
                #print U
                #print "U'"
                #print U.transpose()
                print "SIGMA"
                print SIGMA
                #print "V"
                #print V
                #print "V'"
                #print V.transpose()
                
                """=====================================================
                # Compute R and S
                # According to HZ 9.14 there are four factorisations
                # S = (+/-)UZU', R1 = (+/-)UWV' or R2 = (+/-)UW'V'
                ====================================================="""                
                S = U.dot(Z).dot(U.transpose())
                print "S"
                print S
                R1 = U.dot(W).dot(V.transpose())
                print "R1"
                print R1
                #print "R.R'"
                #print R.dot(R.transpose())                
                R2 = U.dot(W.transpose()).dot(V.transpose())
                print "R2"
                print R2
                #print "wiki_R.wiki_R'"
                #print wiki_R.dot(wiki_R.transpose())
                SR1 = S.dot(R1)
                SR1 /= SR1[2,2]
                print "SR1"
                print SR1
                SR2 = S.dot(R2)
                SR2 /= SR2[2,2]
                print "SR2"
                print SR2
                R = R2
                
                t = U[:, 2]
                print "t"
                print t
                
                # First Projection Matrix
                P1 = np.append(np.identity(3), [[0], [0], [0]], 1)
                #print "P1"
                #print P1
                
                P2E = np.array([[R[0,0],R[0,1],R[0,2],t[0]],[R[1,0], R[1,1], R[1,2], t[1]],[R[2, 0], R[2,1], R[2,2], t[2]]])
                #print "P2E"
                #print P2E
                
                # Bottom out on no accepted points
                if i1_pts_corr.size == 0:
                    print "No Accepted Points"
                    return
                
                points4D = cv2.triangulatePoints(P1, P2E, i1_pts_corr.transpose(), i2_pts_corr.transpose())
               
                # Plot tracked features on stacked images
                ml = list(mask.flat)                
                img2 = stackImagesVertically(grey_previous, grey_now)
                imh = grey_previous.shape[0]
                idx = 0
                for p1, p2 in zip(i1_pts_corr, i2_pts_corr):
                    idx += 1
                    if ml[idx - 1] == 0:
                        continue
                    cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (255, 0 , 255), 1)
                cv2.imshow("track", img2)
                
                while True:
                    cv2.waitKey(5)
                #plot.pause(0.05)


def run():
    # Initialise tracker
    m = FeatureTracker()
    m.featureTrack()

if __name__ == '__main__':
    run()
