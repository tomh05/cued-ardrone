#!/usr/bin/env python

import roslib
roslib.load_manifest('floor_gridder')
import rospy
import math
import cv2
import tf
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PolygonStamped
from custom_msgs.msg import Described3DPoints
from copy import deepcopy

GRIDFACTOR = 100.
SEGMENTS = 3.
ACCUMULATED = False

class FloorGridder():
    def __init__(self):        
        self.connect()
        self.clear_maps()
        cv2.namedWindow('test')
        
        
        
    
    def connect(self):
        global ACCUMULATED
        print 'Waiting for calibration ... '
        camera_info = rospy.wait_for_message('/ardrone/front/camera_info', CameraInfo)
        self.setCameraInfo(camera_info)
        if ACCUMULATED:
            print "Using Accumulated"
            rospy.Subscriber('/accumulator/shifted_described_cloud',Described3DPoints,self.on_got_cloud)
        else:
            rospy.Subscriber('/scan/relative_described_cloud',Described3DPoints,self.on_got_cloud)
        self.poly_pub = rospy.Publisher('/accumulator/polygon', PolygonStamped)
        
    def clear_maps(self):
        # Gridded map is simply a display purpose map
        self.gridded_map = np.ones(shape=(768,768), dtype = np.uint8)
        self.gridded_map = 255*self.gridded_map
        # Nav map is a map of valid drone centre positions
        self.nav_map = np.ones(shape=(      768,768), dtype = np.uint8)
        self.nav_map = 255*self.gridded_map
        # Centre offset gives the [x; y] distance from top left to world origin
        self.centre_offset = np.array([self.gridded_map.shape[1]/2, 
                                       self.gridded_map.shape[0]/2])
        
    def image_to_map(self, pt, position_i, quat_i_to_w):
        sub = np.add(pt, position_i)
        p_w = tf.transformations.quaternion_matrix(quat_i_to_w)[:3,:3].dot(sub)
        p_m = np.add(GRIDFACTOR*p_w, self.centre_offset[0]) # This only works due to square map
        print self.centre_offset[0]
        return np.array(p_m[:2].T, dtype=np.int32)
    
    def get_subset_fov(self, pts, bounds):
        
        # Psuedo-orthographic
        camera_relative = pts/pts[2]
        
        mask_left = camera_relative[0,:] > bounds[0,0]
        mask_right = camera_relative[0,:] < bounds[0,1]
        mask = np.logical_and(mask_left, mask_right)
        
        masked = pts[:, mask]
        
        return masked
    
    def on_got_cloud(self, msg):
        # Decode points
        points = np.reshape(np.array(msg.points3D), (3, -1))
        position_i = np.array([msg.position_i]).T
        quat_i_to_w = msg.quat_i_to_w
        
        for i in range(int(SEGMENTS)):
            print "i: ", i
            seg_points = self.get_subset_fov(points, self.segs[:,i:i+2])
            print seg_points.shape[1], " points"
            if seg_points.shape[1] >= 4:
            # Get 10th percentile in image z
                mean = np.mean(seg_points[2,:])
                variance = np.var(seg_points[2,:])
                z = mean - 1.75*variance
                print z
                pts_i = np.hstack((z*self.segs[:,i:i+2], 25.*self.segs[:,i+1:i+2], 25.*self.segs[:,i:i+1]))
                pts_m = self.image_to_map(pts_i, position_i, quat_i_to_w)
                cv2.fillConvexPoly(self.gridded_map, pts_m, (0,0,0))
        
        ret,thresh = cv2.threshold(self.gridded_map,127,255,0)
        cnt, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        print cnt[0].shape
        
        
        spoly = PolygonStamped()
        poly = Polygon()
        for pt in cnt[0]:
            p = Point32()
            p.x = float(pt[0,0]-self.centre_offset[0])/GRIDFACTOR; p.y = float(pt[0,1]-self.centre_offset[1])/GRIDFACTOR
            poly.points.append(deepcopy(p))
        spoly.polygon = poly
        spoly.header.frame_id = '/world'
        spoly.header.stamp = msg.header.stamp
        self.poly_pub.publish(spoly)
        #approx = cv2.approxPolyDP(cnt[0],8,True)
        
        
        
        
        
        
        
        
        
        
        
        
        im = cv2.cvtColor(self.gridded_map, cv2.COLOR_GRAY2BGR)
        
        cv2.drawContours(im, cnt,-1,(0,255,0),3)
        
        cv2.imwrite('test.png', self.gridded_map)
        cv2.imshow('test', im)
        cv2.waitKey(2)
        
        #send_gridded_map = Int16MultiArray()
        #send_gridded_map.data=self.gridded_map.reshape(-1,).tolist()
        #send_gridded_map.layout.dim = [MultiArrayDimension('width', self.gridded_map.shape[1]*self.gridded_map.shape[0], self.gridded_map.shape[1])]
        #print send_gridded_map.data[0]
        
        
        #self.gridded_map_pub.publish(send_gridded_map)
        
    def on_got_clean_cloud(self, point_cloud):        
        # Nav map is a map of valid drone centre positions
        self.nav_map_coarse = np.ones(shape=(1024,1024), dtype = np.uint8)
        self.nav_map_coarse = 255*self.gridded_map
        for p in point_cloud.points:
            cv2.circle(self.nav_map_coarse, (int(p.x)+self.centre_offset[0], int(p.y)+self.centre_offset[1]), 3, (0,0,0), -1)
        cv2.imwrite('test2.png', self.nav_map_coarse)
        
    def setCameraInfo(self, ci):
        """Converts the ROS published camera info into numpy arrays and 
        stores in FeatureTracker"""
        self.cameraMatrix =  np.array([[ci.K[0], ci.K[1], ci.K[2]], [ci.K[3], ci.K[4], ci.K[5]], [ci.K[6], ci.K[7], ci.K[8]]], dtype=np.float32)
        self.inverseCameraMatrix = np.linalg.inv(self.cameraMatrix)
        self.distCoeffs = np.array([ci.D], dtype=np.float32)
        self.P = np.array([ci.P[:4],ci.P[4:8],ci.P[8:12]])
        step_w = float(ci.width)/SEGMENTS
        print step_w
        steps = np.ones((3,SEGMENTS+1))
        for i in range(int(SEGMENTS)+1):
            steps[:,i:i+1] = np.array([[step_w*i],[ci.K[5]],[1.]])
        print "steps: ", steps
        self.segs = self.inverseCameraMatrix.dot(steps)
        print self.segs
        
        print "                    Calibration Initialised\r\n"

def run():  
    global ACCUMULATED  
    rospy.init_node('Plane_Project')
    
    # Get parameters
    ACCUMULATED = rospy.get_param('~acc', False)
    print "\r\n"
    print "===================== Plane Project ======================="
    print "Use accumulated clouds: ", ACCUMULATED," - (set with _acc)"
    print "==================================================================="
    print "\r\n"
    
    fg = FloorGridder()
    # Begin ROS loop
    rospy.spin()

if __name__ == '__main__':
    run()
