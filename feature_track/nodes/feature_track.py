#!/usr/bin/env python

#===================Changelog==================================================
# Initial Creation
# Added undistort
# Added 3D via fundamental matrix
# Added 3D via essential matrix
# Efficiency improvements by buffering now to previous
#------------------------------------------------------------------------------
# Fixed buffering
# Added fundamental matrix error check
# Added extraction of projection matrices
# Added selection of best case projection matrix
# Added 3D point projection
#------------------------------------------------------------------------------
# Switched to efficient np-style filters
# Corrected filtering of rejected points through each stage
#==============================================================================

#=====================Notes====================================================
# 
# It is unclear whether the translation matrix needs rescaling by camera proj
#
# It may be worth reading camera cal .yamls from disk if missing
#
# The various checks and filters carried out here should allow us to provide
# a quantified measure of confidence in the result
#==============================================================================

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
        self.previous = None
        self.grey_previous = None
        self.frameskip = 0
        self.calibrated = False
        self.fd = cv2.FeatureDetector_create('ORB')
        self.de = cv2.DescriptorExtractor_create('FREAK')
        self.dm = cv2.DescriptorMatcher_create('BruteForce')
        self.pts1 = None
        self.desc1 = None
        self.kp1 = None
        cv2.namedWindow("track")
        self.cloud_pub = rospy.Publisher('pointCloud', PointCloud)
    
    def update_previous(self, thing):
        """Takes a cv2 numpy array image and sets the FeatureTracker
        previous image to it"""
        #print('Now: %s' % (thing,))
        #print('Previous: %s' % (self.previous))
        self.previous = thing
        
    def compute_fundamental(self, x1,x2):
        n = x1.shape[1]
        # build matrix for equations
        A = np.zeros((n,9))
        for i in range(n):
            A[i] = [x1[0,i]*x2[0,i], x1[0,i]*x2[1,i], x1[0,i]*1.,
            x1[1,i]*x2[0,i], x1[1,i]*x2[1,i], x1[1,i]*1.,
            1.*x2[0,i], 1.*x2[1,i], 1. ]
        # compute linear least square solution
        U,S,V = np.linalg.svd(A)
        F = V[-1].reshape(3,3)
        # constrain F
        # make rank 2 by zeroing out last singular value
        U,S,V = np.linalg.svd(F)
        S[2] = 0
        F = np.dot(U,np.dot(np.diag(S),V))
        return F

        
    def compute_F_error(self, F, x1_32, x2_32):
        errs = []
        for i, p in enumerate(zip(x1_32.T, x2_32.T)):
            errs.append(np.r_[p[1], 1].T.dot(F).dot(np.r_[p[0], 1]))
        return np.mean(errs)
        
    def compute_correct_projection(self, F):        
        return

        
    def featureTrack(self, img):
        """Takes a cv2 numpy array image and compared to a previously
        buffered image. Features are extracted from each frame, 
        undistorted and matched."""
        
        print ""

        #img = cv2.imread("/home/alex/frame0000.jpg")
        
        # Initialise previous image buffer
        if self.previous == None:
            self.update_previous(img)
            self.grey_previous = cv2.cvtColor(self.previous, cv2.COLOR_BGR2GRAY)
            

        # Skip frames. Need to add ROS parameter to allow setting
        
        self.frameskip += 1
        if self.frameskip < 2:
            return
            
        self.frameskip = 0
        
            
        # Convert working images to monochrome
        grey_previous = self.grey_previous
        #grey_previous = cv2.cvtColor(cv2.imread("/home/alex/frame0001.jpg"), cv2.COLOR_BGR2GRAY)
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #img = cv2.imread("/home/alex/frame0002.jpg")
        grey_now = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Initialise features and descriptor
        if self.pts1 == None:
            self.pts1 = self.fd.detect(grey_now)
            self.kp1, self.desc1 = self.de.compute(grey_now, self.pts1)
            return

        # Create feature detector and extracts points
        pts1 = self.pts1
        pts2 = self.fd.detect(grey_now)

        # Create a descriptor extractor and describe points
        
        kp1, desc1 = self.kp1, self.desc1
        kp2, desc2 = self.de.compute(grey_now, pts2)
        
        # Bottom out if failed to get features
        if desc1 == None or desc2 == None or len(desc1) == 0 or len(desc2) == 0:
            self.update_previous(img)
            self.grey_previous = grey_now
            self.pts1 = pts2
            self.kp1, self.desc1 = kp2, desc2
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
        if (len(i1_pts) > 8):
            if (len(i2_pts) > 8): # this is redundant as matched lists
                
                
                
                if self.calibrated:
                    # Undistort points using calibration data
                    i1_mat = np.array([i1_pts])
                    i2_mat = np.array([i2_pts])                    
                    i1_pts_undistorted = cv2.undistortPoints(i1_mat, self.cameraMatrix, self.distCoeffs, P=self.P) #Do not pass camera P here as working in normalised
                    #print i1_pts_undistorted
                    i2_pts_undistorted = cv2.undistortPoints(i2_mat, self.cameraMatrix, self.distCoeffs, P=self.P)
                    
                    #print i2_pts_undistorted
                else:
                    # Use distorted points as calibration missing
                    print "WARNING: No calibration info. Using distorted feature positions"
                    print "This will be wrong as not normalised"
                    i1_pts_undistorted = i1_pts
                    i2_pts_undistorted = i2_pts
                
                    
                #print i1_pts_undistorted #= i1_pts
                #print i2_pts_undistorted #= i2_pts
                
                #cv2.stereoRectify(self.cameraMatrix, self.distCoeff, self.cameraMatrix, self.distCoeff, img.shape, 
                
                """============================================================
                # Extract fundamental matrix and then remove outliers
                # FM_RANSAC should be good with lowish outliers
                # FM_LMEDS may be more robust in some cases
                ============================================================"""
                # Extract fundamental matrix
                # F contains fundamental matrix
                # mask is a binary mask of points fitting the matrix
                F, mask = cv2.findFundamentalMat(i1_pts, i2_pts, cv2.FM_RANSAC, param1 = 1., param2 = 0.99)
                # Expand mask for easy filtering
                mask_prepped = np.append(mask, mask, 1.)
                print "No of matched points : ", len(i1_pts_undistorted[0])
                # Efficient np-style filtering, then reform
                i1_pts_masked = np.reshape(i1_pts_undistorted[0][mask_prepped==1], (-1, 2))
                i2_pts_masked = np.reshape(i2_pts_undistorted[0][mask_prepped==1], (-1, 2))
                print "No of masked points : ", len(i1_pts_masked) 
                """========================================================="""
                
                i1_pts_undistorted = np.array([i1_pts_masked])
                i2_pts_undistorted = np.array([i2_pts_masked])

                
                
                #np.set_printoptions(suppress=True, precision=6)


                """============================================================
                # Examine quality of F
                # Reject if error is too high and go to next frame
                # Error is given to a scale of in pixels depending on if input
                # is normalised
                ============================================================"""
                print "F : "
                print F             
                                
                avg_error = self.compute_F_error(F, i1_pts_undistorted[0].transpose(), i2_pts_undistorted[0].transpose())
                print "Avg error : ", avg_error
                if (abs(avg_error)>1):
                    print "===================="
                    print "F Error too high"
                    print "===================="
                    self.update_previous(img)
                    self.grey_previous = grey_now
                    self.pts1 = pts2
                    self.kp1, self.desc1 = kp2, desc2
                    return
                """========================================================="""
                
                '''    
                F2 = self.compute_fundamental(i1_pts, i2_pts)
                print "F2 : "
                print F2
                avg_error = self.compute_F_error(F, i1_pts_undistorted[0].transpose(), i2_pts_undistorted[0].transpose())
                print "Avg error : ", avg_error
                '''
                
                

                
                
                
                
                #epilines = cv.CreateMat(len(i1_pts_undistorted), 3, cv.CV_32F)
                #cv.ComputeCorrespondEpilines(cv.fromarray(i1_pts_undistorted), 1, cv.fromarray(F), epilines)
                #epilines =  np.array(epilines)
                
                """============================================================
                # Filter points that fit F using cv2.correctMatches
                # This unhelpfully overwrites np.nan over rejected entried
                # np.nan == np.nan returns false so have to use np.isnan(.)
                # NB: This check appears redundant as F is calculated to match
                ============================================================"""
                i1_pts_corr, i2_pts_corr = cv2.correctMatches(F, i1_pts_undistorted, i2_pts_undistorted)
                mask_nan = np.isnan(i1_pts_corr[0])
                i1_pts_corr = np.reshape(i1_pts_corr[0][mask_nan == False], (-1, 2))
                i2_pts_corr = np.reshape(i2_pts_corr[0][mask_nan == False], (-1, 2))
                print "No of corrected points: ", len(i1_pts_corr)
                """========================================================="""
                
                
                E = self.cameraMatrix.transpose().dot(F.dot(self.cameraMatrix))
                #E = E.dot(self.cameraMatrix) 
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
                if np.linalg.det(U.dot(V))<0:
                    V = -V
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
                #print "E2"
                #print E2
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
                # First Projection Matrix
                P1 = np.append(np.identity(3), [[0], [0], [0]], 1)
                #print "P1"
                #print P1
                """============================================================
                # Compute the four possible P2 projection matrices
                # Note in particular the matrix multiplication order
                # This caught me out for a long while
                # Also the V returned by np's svd is V'
                ============================================================"""
                projections = []
                # UWV'|u3
                projections.append(np.append(U.dot(W.dot(V)),np.array([U[:,2]]).transpose(),1))
                #print [np.vstack((np.dot(U,np.dot(W,V)).T,U[:,2])).T]
                # UWV'|-u3
                projections.append(np.append(U.dot(W.dot(V)),np.array([-U[:,2]]).transpose(),1))
                #print [np.vstack((np.dot(U,np.dot(W,V)).T,-U[:,2])).T]
                # UW'V'|u3
                projections.append(np.append(U.dot(W.transpose().dot(V)),np.array([U[:,2]]).transpose(),1))
                #print [np.vstack((np.dot(U,np.dot(W.T,V)).T,U[:,2])).T]
                # UW'V'|-u3
                projections.append(np.append(U.dot(W.transpose().dot(V)),np.array([-U[:,2]]).transpose(),1))
                #print [np.vstack((np.dot(U,np.dot(W.T,V)).T,-U[:,2])).T]
                """========================================================="""
                
                """Check E=RS as t appears wrong"""                
                R = projections[0][:,:3]
                print "Rotation Matrix : ", R
                t = projections[0][:,3:4]
                print "magnitude sq", t.transpose().dot(t)
                print "Translation Vector : ", t
                
                S = np.array([[0., t[2], -t[1]],
                              [-t[2], 0., t[0]],
                              [t[1], -t[0], 0.]])
                #print "S : ", S
                SR = S.dot(R)
                SR = SR/SR[2,2]
                #print "SR : ", SR
                
                print t.shape
                print self.cameraMatrix.shape
                print self.cameraMatrix.dot(t)
                
                # Bottom out on no accepted points
                if i1_pts_corr.size == 0:
                    self.update_previous(img)
                    self.grey_previous = grey_now
                    self.pts1 = pts2
                    self.kp1, self.desc1 = kp2, desc2
                    print "No Accepted Points"
                    return
                
                """============================================================
                # Determine projection with most valid points
                # Produce boolean mask for best case and filter pts
                ===============================================================
                # Note: This currently checks for positive P1 and P2 projection
                # However, the P2 check is assuming the camera does not move
                # Given the P1 camera is set as origin, P2 check should be wrt
                # the translation vector
                # As translation vector appears wrong, it may be worth going
                # with P1 check only (P1 and P2 projection produce exact same
                # 3D points in the ideal case)
                # -----------> This didn't help trying, disparate PI
                ============================================================"""
                
                ind = 0
                maxfit = 0
                for i, P2 in enumerate(projections):
                    # infront accepts only both dimensions
                    points4D = cv2.triangulatePoints(P1, P2, i1_pts_corr.transpose(), i2_pts_corr.transpose())
                    d1 = np.dot(P1,points4D)[2]
                    d2 = np.dot(P2,points4D)[2]
                    #PI =  sum(d1>0)+sum(d2>0)
                    PI = sum((d1>0) & (d2>0))
                    #PI = sum((d1>0))
                    #print P2
                    print "Performance index ", i, " : ", PI
                    if PI > maxfit:
                        #maxfit = sum(d1>0)+sum(d2>0)
                        maxfit = sum((d1>0) & (d2>0))
                        #maxfit = sum((d1>0))
                        #print "maxfit : ", maxfit
                        ind = i
                        infront = (d1>0) & (d2>0)
                #print ind
                #print "infront : ", infront
                if (maxfit == 0):
                    print "===================="
                    print "P2 not extracted"
                    print "===================="
                    self.update_previous(img)
                    self.grey_previous = grey_now
                    self.pts1 = pts2
                    self.kp1, self.desc1 = kp2, desc2
                    return
                P2 = projections[ind]
                print "P1"
                print P1                
                print "P2 selected : "
                print projections[ind]
                print "No of valid points : ", sum(infront)
                
                
                #print "i1_pts_corr : ", i1_pts_corr
                #print "i2_pts_corr : ", i2_pts_corr
                
                #print "infront : ", infront 
                # Filter points
                infront = np.array([infront]).transpose()
                infront = np.append(infront, infront, 1)
                #print "Prepped infront : ", print infront
                i1_pts_corr = np.reshape(i1_pts_corr[infront==True], (-1, 2))
                i2_pts_corr = np.reshape(i2_pts_corr[infront==True], (-1, 2))
                #print "i1_pts_infront : ", i1_pts_corr
                #print "i2_pts_infront : ", i2_pts_corr
                print "No of points infront : ", len(i1_pts_corr)
                """========================================================="""
                
                print "Rotation Matrix : "
                R = P2[:,:3]
                R4 = np.diag([0., 0., 0., 1.])
                R4[:3, :3] = R
                print "R4 : ", R4
                quat = tf.transformations.quaternion_from_matrix(R4)
                
                br = tf.TransformBroadcaster() #create broadcaster
                br.sendTransform((0, 0, 0),
                         quat,
                         rospy.Time.now(), # NB: 'now' is not the same as time data was sent from drone
                         "image_rotation",
                         "world")
                
                print "Translation Vector : "
                t = P2[:,3:4]
                print t
                
                points4D = cv2.triangulatePoints(P1, projections[ind], i1_pts_corr.transpose(), i2_pts_corr.transpose())
                #print "4D points"
                #print X
                X1 = np.dot(P1,points4D)
                print "X1"
                #print X1
                X2 = P2.dot(points4D)
                print "X2"
                #print X2
                
                points3D1 = zip(*X1)
                points3D2 = zip(*X2)
                print "points3D1 max x, y, z: ", max(X1[0]), max(X1[1]), max(X1[2])
                print "points3D1 min x, y, z: ", min(X1[0]), min(X1[1]), min(X1[2])
                                
                # Publish point cloud
                cloud = PointCloud()
                cloud.header.stamp = rospy.Time.now()
                cloud.header.frame_id = "ardrone_base_link" #Should be front camera really
                #print "cloud"
                #print cloud
                
                for i, p in enumerate(points3D1):
                    cloud.points.append(gm.Point32())
                    cloud.points[i].x = p[0]
                    cloud.points[i].y = p[1]
                    cloud.points[i].z = p[2]
                
                self.cloud_pub.publish(cloud)
                
                
                #print points3D1
                
                '''
                # 3D plot
                import matplotlib.pyplot as plot
                from mpl_toolkits.mplot3d import Axes3D
                first = True
                if first:
                    first = False
                    fig = plot.figure(1)
                    ax = Axes3D(fig)
                ax.scatter(-X1[0],X1[1],X1[2])
                '''
                
                '''
                U,SIGMA,V = np.linalg.svd(F)
                
                SIGMA = np.diag(SIGMA)
                
                SIGMA[2] = [0., 0., 0.]
                #print SIGMA
                
                #print V
                left_epipolar = V[:, 2]
                print "left epi"
                print left_epipolar
                
                E_SM = np.array([[0, -left_epipolar[2], left_epipolar[1]], [left_epipolar[2], 0, -left_epipolar[0]], [-left_epipolar[1], left_epipolar[0], 0]], dtype=np.float32)
                #print "E_SM"
                #print E_SM
                
                # Second Projection Matrix
                left_epipolar = np.array([left_epipolar]).transpose()
                P2 = np.append(E_SM.dot(F),left_epipolar,1)
                print "P2"
                print P2
                '''
                
                '''
                print i1_pts_corr
                print zip(*i1_pts_corr[0])[0]
                x1 = np.array([zip(*i1_pts_corr[0])[0]]).transpose()
                x2 = np.array([zip(*i1_pts_corr[0])[1]]).transpose()
                print np.append(x1, x2, 1)
                
                proj1 = zip(*i1_pts_corr[0])[0]
                print proj1
                '''
                
                '''
                # Calculate via essential matrix
                '''
                

                '''
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
                #print "SR2"
                #print SR2
                R = R2
                
                t = U[:, 2]
                print "t"
                print t
                '''
                

                
                '''
                P2E = np.array([[R[0,0],R[0,1],R[0,2],t[0]],[R[1,0], R[1,1], R[1,2], t[1]],[R[2, 0], R[2,1], R[2,2], t[2]]])
                #print "P2E"
                #print P2E
                '''
                
                
                '''
                fig = plot.figure(1)
                ax = Axes3D(fig)
                ax.scatter(zip(*points3D)[0], zip(*points3D)[1],zip(*points3D)[2])
                '''
               
                

                
                
                
                
                
                    
                    
                
                #points3D = cv2.convertPointsFromHomogeneous(points4D)
                
                #print "points3D"
                #print points3D
                

                
                
                #print "no of pre-drawn points : ", len(i1_pts_corr)

                # Plot tracked features on stacked images   
                img2 = stackImagesVertically(grey_previous, grey_now)
                imh = grey_previous.shape[0]
                county = 0
                for p1, p2 in zip(i1_pts_corr, i2_pts_corr):
                    #idx += 1
                    #if ml[idx - 1] == 0:
                    #    continue
                    county += 1
                    cv2.line(img2,(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1] + imh)), (255, 0 , 255), 1)
                cv2.imshow("track", img2)
                print "No of drawn points : ", county
                cv2.waitKey(5)
                #plot.pause(0.05)
                
            
        # Update previous image buffer
        self.update_previous(img)
        self.grey_previous = grey_now
        self.pts1 = pts2
        self.kp1, self.desc1 = kp2, desc2


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
            print "Calibration Initialised"
  
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
