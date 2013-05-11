#!/usr/bin/env python

'''
PositionController.py

Modified for particle filter inclusion

This program receives current position and desired position in world coordinates, and calculates commands in the drone command frame, using ssm
Also provides 'fail-safe mechanism' which uses navdata integration to arrive at estimated world positions.
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8


import tf

from time import time, sleep
import sys
from math import sin, cos, radians, degrees, pi
from visualization_msgs.msg import MarkerArray, Marker


class PositionController:
    
    # args: destination position, destination yaw, (current position unused)?, unused?, control params
    def __init__(self, dpw0, dyw0, cpw0, cyw0, d0=1.0/2400, d1=-0.008, d2=-0.00028):

        self.d0=d0            #1/2600 for marker and indep modes control
        self.d1=d1            #-0.008 for marker and indep modes control
        self.d2=d2            #-0.00032 for marker and indep modes control  
        self.dpw=dpw0
        self.dyw=dyw0
        
        self.reftm = time()
        self.cmd_log = {'tm':0.0, 'tw':0.0}
        self.nd_log = {'tm':0.0, 'nd':0.0, 'ph':0.0, 'th':0.0, 'ps':0.0, 'vx':0.0, 'vy':0.0, 'vz':0.0, 'al':0.0, 'cpw':cpw0, 'cyw':dyw0}
        
        self.ref = {'al':1500}    #'al' for height
        self.error = {'al':[]}
        self.refalon = True
        self.yawon = True
        
        self.twist = Twist()
        self.cmdpub = rospy.Publisher('cmd_vel', Twist)
        self.landpub = rospy.Publisher('ardrone/land', Empty)
        self.resetpub = rospy.Publisher('ardrone/reset', Empty)
        self.takeoffpub = rospy.Publisher('ardrone/takeoff', Empty)
        self.navdatasub = rospy.Subscriber('ardrone/navdata', Navdata, self.navdataCallback)
        self.camselectclient = rospy.ServiceProxy('ardrone/setcamchannel', CamSelect)
        
        # reset z rotation offset 
        self.resetsub = rospy.Subscriber('/xboxcontroller/button_b', Empty, self.resetZOffset)
        self.zOffset = 0.0
        self.resetZ = False
        
        self.justgotpose = True

        # Listen for particle filter
        self.tfListener = tf.TransformListener();
        self.kParticle = 0.3 # particle filter merging constant
        self.kParticle = 0.1 # particle filter merging constant
        self.kParticleRot = 0.05 # particle filter merging constant
        self.kParticleRot = 0.01 # particle filter merging constant
        #self.kParticle = 1.0 # particle filter merging constant

        self.markerPub = rospy.Publisher('positionControlMarkers',MarkerArray)
        self.br = tf.TransformBroadcaster() #create broadcaster

        self.cleartwist()

    def resetZOffset(self,data):
        self.resetZ = True
        self.cleartwist() 
         
    # creates a timer that runs at 20 Hz
    def pc_timer_init(self):
        try:
            self.pc_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.pc_timer_callback)
        except:
            pass
            print 'pc_timer_init error'
    
    # kill timer    
    def pc_timer_shutdown(self):
        try:
            self.pc_timer.shutdown()
        except:
            pass
            print 'pc_timer_shutdown error'
        
    # reset outgoing robot commands            
    def cleartwist(self):
        self.twist.linear.x= 0.0; self.twist.linear.y= 0.0; self.twist.linear.z= 0.0
        self.twist.angular.x= 0.0; self.twist.angular.y= 0.0; self.twist.angular.z= 0.0; 
    
    
    def navdataCallback(self, msg):
        self.nd_logger(msg)
        if(True): #try
            if self.justgotpose:
                # TODO permanant fix
                ######################################################self.nd_logger(msg)
                self.justgotpose = False
                #self.justgotpose = True
            else:
                
                dt=(msg.header.stamp - self.nd_log['nd'].header.stamp).to_sec()
                dx=(msg.vx+self.nd_log['vx'])*dt/2.0/1000.0
                dy=(msg.vy+self.nd_log['vy'])*dt/2.0/1000.0
                dz=(msg.altd-self.nd_log['al'])/1000.0
                #print 'dz: ', dz
                 # bring in particle filter estimates...v
                (self.pfPos,self.pfRot) = self.tfListener.lookupTransform('/world',"/"+rospy.get_param('tf_prefix')+"/ardrone_base_link_particle_filter",rospy.Time(0)) # get latest transform

                drotZ=msg.rotZ - self.nd_log['ps'] #TODO rotZ offset fix
                if drotZ > 300:        #to account for discontinuity in yaw around 0
                    drotZ = drotZ - 360
                elif drotZ<-300:
                    drotZ = drotZ + 360
                
                self.nd_logger(msg)
                pfRotZ = tf.transformations.euler_from_quaternion(self.pfRot)[2]
                         
                pfRotError = ( pfRotZ - self.nd_log['cyw'])

                if pfRotError > pi:
                    pfRotError -= 2.0 * pi
                elif pfRotError < -pi:
                    pfRotError += 2.0 * pi
                pfRotError *= self.kParticleRot 
                cywnew = self.nd_log['cyw'] + radians(drotZ) + pfRotError
                if cywnew > pi:
                    cywnew -= 2.0 * pi
                elif cywnew < -pi:
                    cywnew += 2.0 * pi   
                yaw=(cywnew+self.nd_log['cyw'])/2.0            #yaw refers to yaw wrt to world frame
                #print 'cywnew', cywnew
                #print 'yaw   ', yaw
                
                
                du=dx*cos(yaw)-dy*sin(yaw)                        #du, dv are displacement in  world frame
                dv=dx*sin(yaw)+dy*cos(yaw)
                
                # pfPos = (0.0,500.0,0.0) # debug - test an extreme position
                du += dt * self.kParticle * (self.pfPos[0] - self.nd_log['cpw'][0]) 
                dv += dt * self.kParticle * (self.pfPos[1] - self.nd_log['cpw'][1]) 
                dz += dt * self.kParticle * (self.pfPos[2] - self.nd_log['cpw'][2]) 
                self.nd_log['cpw'] = ((self.nd_log['cpw'][0]+du, self.nd_log['cpw'][1]+dv, self.nd_log['cpw'][2]+dz))
                self.nd_log['cyw'] =(cywnew)

                #self.drawMarkers()
            
    # throw all navdata into storage 
    def nd_logger(self,msg):
        if (self.resetZ):
            print 'resetting z offset to ', msg.rotZ
            self.zOffset = msg.rotZ
            self.nd_log['cyw'] = 0.0
            self.resetZ = False
        

        #print msg.rotZ
        self.nd_log['tm'] = (time()-self.reftm)
        self.nd_log['nd'] = (msg)
    
        self.nd_log['th'] = (msg.rotY)        # forw+ backw-
        self.nd_log['ph'] = (msg.rotX)        # left- right+
        self.nd_log['ps'] = (msg.rotZ)  
        #self.nd_log['ps'].append(msg.rotZ - self.zOffset)  #TODO review this      # ccw+  cw-
        self.nd_log['vx'] = (msg.vx)
        self.nd_log['vy'] = (msg.vy)
        self.nd_log['al'] = (msg.altd)
        
        self.error['al'] = (self.ref['al']-msg.altd)
        #self.error['ps'].append(self.ref['ps']-msg.rotZ)


    def cmd_logger(self,twcmd):
        self.cmd_log['tm'] = (time()-self.reftm)
        self.cmd_log['tw'] = (twcmd)
    
    
    def refal_handler(self, refal):
        self.ref['al']=refal
        
    # can be called to set the current position from another module - particle filter or deadreckon.py?
    def cpw_cyw_handler(self, cpw, cyw):
        self.nd_log['cyw'] = (cyw)    #pose_handler uses cow[2] (z rot, euler) to determine rot of cmd frame wrt. world frame
        self.nd_log['cpw'] = (cpw)
        self.justgotpose = True
        #print '\ngotpose - \nposition: ' + str(cpw) + '\norientation-z: ' + str(cyw) + '\ntime: ' + str(time()-self.reftm)
        #print 'gotpose'
    
    
    def dpw_handler(self, dpw):
        self.dpw = dpw   #dpw is the destination world coordinate
        
    
    def dyw_handler(self, dyw):
        self.dyw = dyw        #dyw is the required euler z rot of the drone wrt. world frame, in radians


    def pc_timer_callback(self,event):
        # emergency landing routine
        #if (self.nd_log['al'][-1]>(2000)): 
        #    self.landpub.publish(Empty())    
        #    print 'error: altitude too high - landing' 
        # altitude control    
        if self.refalon == True: # true by default
            self.twist.linear.z = max(min(0.0013*self.error['al'], 1.0), -1.0) # bound to +-1
        else:
            zerror = self.dpw[2]-self.nd_log['cpw'][2]                        #zerror is in meters
            #print 'zerror: ', zerror, ' is ', self.dpw[2], ' minus ', self.nd_log['cpw'][-1][2]
            #print 'dpw: ', self.dpw[2]
            self.twist.linear.z = max(min(0.0013*zerror*1000, 1.0), -1.0)
    
        # yaw control    
        if self.yawon == True:
            yawwerror = -self.computeYawDiff(degrees(self.nd_log['cyw']),degrees(self.dyw))        # computes error in yaw world in degrees
            yawspeedlimit=0.6 #0.3 was Rujian's default
            self.twist.angular.z = max(min(yawwerror/130, yawspeedlimit), -yawspeedlimit)
    
        # xy position control    
        if True:
            (xcw, ycw, zcw) = self.nd_log['cpw'] # previous xyz

            (xdw, ydw, zdw) = self.dpw               # destination xyz
            xrw = xdw - xcw # desired displacement vector
            yrw = ydw - ycw
            zrw = zdw - zcw
            psw = self.nd_log['cyw']                #psi world
            #print 'error x: ' + str(xrw) + ' y: ' +str(yrw)

            # rotate xy into world coordinates    
            xrc =  xrw*cos(psw) + yrw*sin(psw) 
            yrc = -xrw*sin(psw) + yrw*cos(psw)
            
            oldph=-self.nd_log['ph']
            oldth= self.nd_log['th']
            oldvy= self.nd_log['vy']
            oldvx= self.nd_log['vx']
            rx=1000.0*xrc*self.d0+oldth*self.d1+oldvx*self.d2
            ry=1000.0*yrc*self.d0+oldph*self.d1+oldvy*self.d2
            
            #print rx, ry
            #print psw
            #llim=0.5    #linear cmd limit
            llim=0.01    #linear cmd limit
            llim=0.2    #linear cmd limit
            self.twist.linear.x=max(min(rx,llim),-llim)
            self.twist.linear.y=max(min(ry,llim),-llim)
            #print 'resultant twist x: ' + str(self.twist.linear.x) + ' twist y: ' + str(self.twist.linear.y) 
        else:
            pass
            #print 'calculate command error'
        
        #self.cleartwist() #TODO REMOVE
        self.cmdpub.publish(self.twist)
        self.cmd_logger(self.twist)
        #print 'twist: \n', self.twist
        
    #compute yaw difference    
    def computeYawDiff(self, yaw, center):
        #compute yaw-desiyaw in interval (-180,180)
        rem = (yaw-center)%360
        if rem>180:
            rem=rem-360
        return rem


    def sendTransform(self):
       self.br.sendTransform((self.pfPos[0],self.pfPos[1], self.pfPos[2]),
                                  (self.pfRot[0], self.pfRot[1], self.pfRot[2], self.pfRot[3]),
                                  time, 
                                  "ardrone_base_link_control",
                                  "/world")

         
    # draw visualisation      
    def drawMarkers(self):
        markerArray = MarkerArray()

        # faded white arrow at desired position
        marker = Marker()
        marker.header.frame_id = '/world'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 5
        marker.scale.z = 1
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.id=1
        quat = tf.transformations.quaternion_from_euler(0.0,0.0,self.dyw)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.pose.position.x = self.dpw[0]
        marker.pose.position.y = self.dpw[1]
        marker.pose.position.z = self.dpw[2]

        markerArray.markers.append(marker)
        
        # white arrow at current position
        marker = Marker()
        marker.header.frame_id = '/world'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 6
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.id=2
        quat = tf.transformations.quaternion_from_euler(radians(self.nd_log['ph']), radians(self.nd_log['th']), self.nd_log['cyw'])
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.pose.position.x = self.nd_log['cpw'][0]
        marker.pose.position.y = self.nd_log['cpw'][1]
        marker.pose.position.z = self.nd_log['cpw'][2]

        markerArray.markers.append(marker)

        # particle filter
        marker = Marker()
        marker.header.frame_id = '/world'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 3
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.id=3
        marker.pose.orientation.x = self.pfRot[0]
        marker.pose.orientation.y = self.pfRot[1]
        marker.pose.orientation.z = self.pfRot[2]
        marker.pose.orientation.w = self.pfRot[3]
        marker.pose.position.x = self.pfPos[0]
        marker.pose.position.y = self.pfPos[1]
        marker.pose.position.z = self.pfPos[2]

        markerArray.markers.append(marker)

        self.markerPub.publish(markerArray)


def main(args):
    pass
