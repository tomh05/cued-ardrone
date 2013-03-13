#!/usr/bin/env python  
import roslib
roslib.load_manifest('particlefilter')
import rospy
import ardrone_autonomy.msg
import tf
import math
import random
import numpy as np
from   std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped


class Particle:
    def __init__(self):
        x = -5.0 + (random.random() * 10.0)
        y = -5.0 + (random.random() * 10.0)
        #x = 0.0
        #y = 0.0
        z = 0.0
        self.pos = np.array([x,y,z])
        self.prevtime = None

        # for drawing debug pointcloud
        self.markerPos = np.array([0.0,0.0,0.0])

        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        #self.gamma = -2.0 + (random.random() * 4.0)
        self.orientation = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)
        self.morientation = tf.transformations.quaternion_from_euler(self.alpha, self.beta, self.gamma)
        self.xy_noise           = 0.0
        self.altitude_noise     = 0.0
        self.pitch_noise        = 0.0
        self.yaw_noise          = 0.0
        self.roll_noise         = 0.0
        self.marker_noise_p     = 0.0
        self.marker_noise_r     = 0.0

    def setNoise(self,xy,alt,roll,pitch,yaw,marker_p,marker_r):
            self.xy_noise           = xy
            self.altitude_noise     = alt
            self.pitch_noise        = pitch
            self.yaw_noise          = yaw
            self.roll_noise         = roll
            self.marker_noise_p     = marker_p
            self.marker_noise_r     = marker_r

    def move(self,movement):
        # rosbag: reset if time starts again
        time = movement.absoluteTransform.header.stamp
        if self.prevtime == None:
            self.prevtime = time 
        deltat = (time - self.prevtime).to_sec()
        self.prevtime = time
        if (deltat < 0): # rosbag looping
            #x=-5.0+ (random.random() * 10.0)
            print 'looping rosbag'
            #y=-5.0+ (random.random() * 10.0)
            x=0.0
            y=0.0
            z=0.0
            self.pos = np.array([x,y,z])
            self.orientation = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)

        # work out relative displacement
        # convert translation to numpy format
        t = movement.relativeTranslation
        displacement = np.array([t.x,t.y,t.z])

        #convert rotation quaternion to numpy format
        q = movement.absoluteTransform.transform.rotation
        #np_quaternion = np.array([q.x,q.y,q.z,q.w])
        new_quaternion = np.array([q.x,q.y,q.z,q.w])
        #k = 0.5 # 0.1 worked well
        #np_quaternion = np.add(k * new_quaternion,(1-k) * self.orientation)


        # ROTATION
        # make 10% of particles rotate much more
        if (random.random() > 0.1):
            boostfactor = 1.0
        else:
            boostfactor = 10.0
        new_quaternion = self.addRotNoise(new_quaternion,self.roll_noise*boostfactor,self.pitch_noise*boostfactor,self.yaw_noise*boostfactor)
        #np_quaternion = np.add(new_quaternion,[nx,ny,nz,0.0]) 

        # integrate up orientation and normalise
        new_quaternion = tf.transformations.quaternion_multiply(new_quaternion, self.orientation)
        new_quaternion /= np.sqrt(np.inner(new_quaternion,new_quaternion))

        self.orientation = new_quaternion



        # POSITION 
        # For 90% of particles, add small amount of noise
        # For 10%, add a large amount of noise
        if (random.random() > 0.1):
            xyboostfactor = 1.0
            zboostfactor = 1.0
        else:
            xyboostfactor = 10.0
            zboostfactor = 1.0
        
        nx = random.gauss(0.0,xyboostfactor*self.xy_noise)
        ny = random.gauss(0.0,xyboostfactor*self.xy_noise)
        nz = random.gauss(0.0,zboostfactor*self.altitude_noise)
 
        # add noise
        translationNoise = np.array([nx,ny,nz])

        # perform position update
        rotationmat = tf.transformations.quaternion_matrix(new_quaternion)[:3,:3]
        self.pos = self.pos + np.dot(rotationmat,displacement) + translationNoise
        # for 10% of particles, use absolute altitude measurement
        if (random.random() < 0.1):
            self.pos[2] = movement.absoluteTransform.transform.translation.z
            self.pos[2] += nz



    def addRotNoise(self,quaternion,roll,pitch,yaw):

        nx = random.gauss(0.0,roll)
        ny = random.gauss(0.0,pitch)
        nz = random.gauss(0.0,yaw)

        noise_quaternion = tf.transformations.quaternion_from_euler(nx, ny, nz)

        return tf.transformations.quaternion_multiply(quaternion, noise_quaternion)

        
    def clone(self):
        # return new particle at the current position
        newself = Particle()
        newself.pos = self.pos
        newself.orientation = self.orientation
        newself.morientation = self.morientation
        newself.prevtime = self.prevtime
        return newself

    def gauss(self, mu, sigma, x):
        return np.exp( - ((mu-x)**2) / (sigma ** 2) / 2.0) / np.sqrt( 2.0 * np.pi * (sigma **2))

    def likelihood(self,worldmap,ar_markers):
        #TODO use covariance data?
        weight = 1.0
        #print '---- start of likelihood ------'
        #print 'weight', weight
        # markers are currently relative to base. Set up transform to particle base location...
        rot_mat_world_to_base = tf.transformations.quaternion_matrix(self.orientation)[:3,:3]
        # ...and from particle to front camera frame
        q_base_frontcam = np.array([-0.5,0.5,-0.5,0.5])
        rot_mat_base_to_frontcam = tf.transformations.quaternion_matrix(q_base_frontcam)[:3,:3]
        #print 'MARKERS', len(ar_markers.markers)
        #for i in range(len(ar_markers.markers)):
        for i in range(1):
            i = len(ar_markers.markers) - 1
            # convert marker position to numpy format
            t = ar_markers.markers[i].pose.pose.position
            displacement = np.array([t.x,t.y,t.z])
            q = ar_markers.markers[i].pose.pose.orientation
            np_quaternion = np.array([q.x,q.y,q.z,q.w])
            
            # if marker is behind camera, throw it out
            if (displacement[2]<0):
                continue

            # transform marker from front_camera to base_link
            displacement += np.array([0.0,0.0,0.21])
            displacement = np.dot(rot_mat_base_to_frontcam,displacement) 

            # compute marker position in world coordinates (transform base_link to /world)
            self.markerPos = self.pos + np.dot(rot_mat_world_to_base,displacement)

            # search the map for a marker with matching ID  
            mapPosition = None
            for j in range(len(worldmap.markers)):
                if (worldmap.markers[j].id == ar_markers.markers[i].id):
                    mapPosition    = worldmap.markers[j].pose.pose.position
                    mapOrientation = worldmap.markers[j].pose.pose.orientation
                    break

            conf = ar_markers.markers[i].confidence

            if (mapPosition is not None and conf>70):
                print "I'm looking at marker " + str(worldmap.markers[j].id)
                error2 = (mapPosition.x-self.markerPos[0])**2 + (mapPosition.y-self.markerPos[1])**2 + (mapPosition.z-self.markerPos[2])**2
                error2 = np.sqrt(error2)
                #print 'prior w', weight 
                weight *= self.gauss(0,self.marker_noise_p,error2) #/ (1.01-0.01*conf)
                #print 'err2 ',error2 
                #print 'final w' ,weight
                
                #print error2
                #weight *= 1.0/error2
                #weight *= np.exp(-error2)
                #weight = 10.99
                #self.orientation = np.array([0.0,0.0,0.0,1.0])
                q_base_frontcam = np.array([-0.5,0.5,-0.5,0.5])
                #q_base_frontcam = np.array([-0.0,1.0,0.0,0.0])
                #self.morientation = np_quaternion * np.array([0.707,0.707,0.0,0.0]) * np.array([0.707,0.0,0.0,0.707])
                self.morientation = np_quaternion
                # transform from camera frame to base
                self.morientation = tf.transformations.quaternion_multiply(q_base_frontcam,self.morientation)
                # transform from base to world
                self.morientation = tf.transformations.quaternion_multiply(self.orientation,self.morientation)
                
                

                np_mapOrientation = np.array([mapOrientation.x,mapOrientation.y,mapOrientation.z,mapOrientation.w])
                np_mapOrientation /= np.sqrt(np.inner(np_mapOrientation,np_mapOrientation))
                self.morientation /= np.sqrt(np.inner(self.morientation,self.morientation))
                quaternionerror = abs(1.0 - abs(np.inner(self.morientation,np_mapOrientation)))
                '''
                print '----'
                print 'starting weight', weight
                print 'mapOrientation', np_mapOrientation
                print 'selfOrientation',self.morientation 
                print 'error',quaternionerror
                '''
                orientationlik = self.gauss(0.0,self.marker_noise_r,quaternionerror)
                #print 'gauss func' , orientationlik
                #if (orientationlik > 0.001):
                weight *= orientationlik
                #print 'end weight', weight
                #weight = 1.0
        return weight

