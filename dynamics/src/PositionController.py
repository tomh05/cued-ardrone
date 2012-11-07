#!/usr/bin/env python

'''
PositionController.py
This program receives current position and desired position in world coordinates, and calculates commands in the drone command frame, using ssm
Also provides 'fail-safe mechanism' which uses navdata integration to arrive at estimated world positions.
'''
import roslib; roslib.load_manifest('dynamics')
import rospy

from dynamics.msg import Navdata
from dynamics.srv import CamSelect
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8

from time import time, sleep
import sys
from math import sin, cos, radians, pi
import pickle


class PositionController:
	
	def __init__(self):
		
		self.d0=1.0/2800
		self.d1=-0.011
		self.d2=-0.00028
		self.dpw=(0.0, 0.0, 0.0)
		self.cpw=(0.0, 0.0, 0.0)
		
		self.reftm = time()
		self.cmd_log = {'tm':[], 'tw':[]}
		self.nd_log = {'tm':[], 'nd':[], 'ph':[], 'th':[], 'ps':[], 'vx':[], 'vy':[], 'vz':[], 'al':[], 'cpw':[self.cpw,], 'psiw':[0,]}
		
		self.ref = {'al':1400, 'ps':0.0}
		self.error = {'al':[], 'ps':[]}
		
		self.twist = Twist()
		self.cmdpub = rospy.Publisher('cmd_vel', Twist)
		self.landpub = rospy.Publisher('/ardrone/land', Empty)
		self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
		self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
		self.navdatasub = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		self.camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
				
		self.posecmd = {'tm': [], 'cpw':[], 'cow':[], 'dpw':[]}
		self.justgotpose = False
		#self.drone_pos_valid = False


	def pc_timer_init(self):
		try:
			self.pc_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.pc_timer_callback)
		except:
			pass
			print 'pc_timer_init error'
	
	
	def pc_timer_shutdown(self):
		try:
			self.pc_timer.shutdown()
		except:
			pass
			print 'pc_timer_shutdown error'
		
		
	def posecmd_logger(self, cpw, cow, dpw):
		self.posecmd['tm'].append(time()-self.reftm)
		self.posecmd['cpw'].append(cpw)
		self.posecmd['cow'].append(cow)
		self.posecmd['dpw'].append(dpw)
		
			
	def cleartwist(self):
		self.twist.linear.x= 0.0; self.twist.linear.y= 0.0; self.twist.linear.z= 0.0
		self.twist.angular.x= 0.0; self.twist.angular.y= 0.0; self.twist.angular.z= 0.0; 
	
	
	def navdataCallback(self, msg):
		self.nd_logger(msg)
		try:
			if self.justgotpose:
				self.justgotpose = False
			else:
				dt=(msg.header.stamp - self.nd_log['nd'][-2].header.stamp).to_sec()
				dx=(msg.vx+self.nd_log['vx'][-2])*dt/2.0/1000
				dy=(msg.vy+self.nd_log['vy'][-2])*dt/2.0/1000
				#print msg.rotZ
				drotZ=msg.rotZ - self.nd_log['ps'][-2]
				if drotZ > 300:										#to account for discontinuity in yaw around 0
					drotZ = drotZ - 360
				elif drotZ<-300:
					drotZ = drotZ + 360
				psiwnew = self.nd_log['psiw'][-1] + radians(drotZ)
				yaw=(psiwnew+self.nd_log['psiw'][-1])/2.0			#yaw refers to yaw wrt to world frame
				du=dx*cos(yaw)-dy*sin(yaw)
				dv=dx*sin(yaw)+dy*cos(yaw)
				self.nd_log['cpw'].append((self.nd_log['cpw'][-1][0]+du, self.nd_log['cpw'][-1][1]+dv, 0.0))
				self.nd_log['psiw'].append(psiwnew)
				#print yaw
				#print self.nd_log['cpw'][-1]
		except:
			#self.drone_pos_valid = False
			print 'navdata callback error'
			

	def nd_logger(self,msg):
		self.nd_log['tm'].append(time()-self.reftm)
		self.nd_log['nd'].append(msg)
	
		self.nd_log['th'].append(msg.rotY)		# forw+ backw-
		self.nd_log['ph'].append(msg.rotX)		# left- right+
		self.nd_log['ps'].append(msg.rotZ)		# ccw+  cw-
		self.nd_log['vx'].append(msg.vx)
		self.nd_log['vy'].append(msg.vy)
		self.nd_log['al'].append(msg.altd)
		
		self.error['al'].append(self.ref['al']-msg.altd)
		self.error['ps'].append(self.ref['ps']-msg.rotZ)


	def cmd_logger(self,twcmd):
		self.cmd_log['tm'].append(time()-self.reftm)
		self.cmd_log['tw'].append(twcmd)
	

	def pose_handler(self, cpw, cow, dpw):
		self.posecmd_logger(cpw, cow, dpw)
		self.nd_log['psiw'].append(cow[2])
		self.nd_log['cpw'].append(cpw)
		self.dpw = dpw		
		self.justgotpose = True
		print 'gotpose - ' + str(time()-self.reftm)
	

	def pc_timer_callback(self,event):
		if (self.nd_log['al'][-1]>(self.ref['al']+500)):
			self.landpub.publish(Empty())	#height switch
		
		self.twist.linear.z = max(min(0.0013*self.error['al'][-1], 1.0), -1.0)
		#self.twist.angular.z = max(min(self.error['ps'][-1]/150, 1.0), -1.0)
		#print self.nd_log['al'][-1]

		#endif height and yaw control are activated
		
		if True:
			(xcw, ycw, zcw) = self.nd_log['cpw'][-1]
			(xdw, ydw, zdw) = self.dpw
			xrw = xdw - xcw
			yrw = ydw - ycw
			zrw = zdw - zcw
			psw = self.nd_log['psiw'][-1]				#psi world
			
			#print xrw, yrw
			#print psw
			
			xrc =  xrw*cos(psw) + yrw*sin(psw)
			yrc = -xrw*sin(psw) + yrw*cos(psw)
			
			oldph=-self.nd_log['ph'][-1]
			oldth= self.nd_log['th'][-1]
			oldvy= self.nd_log['vy'][-1]
			oldvx= self.nd_log['vx'][-1]
			rx=1000.0*xrc*self.d0+oldth*self.d1+oldvx*self.d2
			ry=1000.0*yrc*self.d0+oldph*self.d1+oldvy*self.d2
			
			#print rx, ry
			#print psw
			self.twist.linear.x=max(min(rx,0.5),-0.5)
			self.twist.linear.y=max(min(ry,0.5),-0.5)
			
			'''
			file1 = open('ssm-r-0-05','w')
			pickle.dump([self.nd_log, self.cmd_log],file1)
			file1.close()
			self.main_timer.shutdown()
			'''
		
		else:
			pass
			#print 'calculate command error'
		
		#if self.drone_pos_valid == False:
		#	self.twist.linear.x = 0; self.twist.linear.y=0

		self.cmdpub.publish(self.twist)
		self.cmd_logger(self.twist)
		#print 'twist: \n', self.twist
			
			
def main(args):
	pass

if __name__ == '__main__':
	main(sys.argv)
