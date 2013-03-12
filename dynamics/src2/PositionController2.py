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
from math import sin, cos, radians, degrees, pi
import pickle


class PositionController:
	
	def __init__(self, dpw0, dyw0, cpw0, cyw0, d0=1.0/2400, d1=-0.008, d2=-0.00028):
		
		self.d0=d0			#1/2600 for marker and indep modes control
		self.d1=d1			#-0.008 for marker and indep modes control
		self.d2=d2			#-0.00032 for marker and indep modes control		#-0.00026 for good 'walk the dog'
		self.dpw=dpw0
		self.dyw=dyw0
		
		self.reftm = time()
		self.cmd_log = {'tm':[], 'tw':[]}
		self.nd_log = {'tm':[], 'nd':[], 'ph':[], 'th':[], 'ps':[], 'vx':[], 'vy':[], 'vz':[], 'al':[], 'cpw':[cpw0,], 'cyw':[dyw0,]}
		
		self.ref = {'al':1000}	#'al' for height
		self.error = {'al':[]}
		self.refalon = True
		self.yawon = False
		
		self.twist = Twist()
		self.cmdpub = rospy.Publisher('cmd_vel', Twist)
		self.landpub = rospy.Publisher('/ardrone/land', Empty)
		self.resetpub = rospy.Publisher('/ardrone/reset', Empty)
		self.takeoffpub = rospy.Publisher('/ardrone/takeoff', Empty)
		self.navdatasub = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)
		self.camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
				
		self.justgotpose = False


	def pc_timer_init(self):
		try:
			self.pc_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.pc_timer_callback)
		except:
			pass
			print 'pc_timer_init error'
	
	
	def pc_timer_shutdown(self):
		try:
			self.pc_timer.shutdown()
			sleep(0.2)
			self.cmdpub.publish(Twist())
			#print 'PositionController2.py: shut down pc timer, zero twist'
		except:
			pass
			print 'pc_timer_shutdown error'
		
			
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
				dx=(msg.vx+self.nd_log['vx'][-2])*dt/2.0/1000.0
				dy=(msg.vy+self.nd_log['vy'][-2])*dt/2.0/1000.0
				dz=(msg.altd-self.nd_log['al'][-2])/1000.0
				#print 'dz: ', dz
				#print msg.rotZ
				drotZ=msg.rotZ - self.nd_log['ps'][-2]
				#if drotZ > 300:										#to account for discontinuity in yaw around 0
					#drotZ = drotZ - 360
				#elif drotZ<-300:
					#drotZ = drotZ + 360
				#cywnew = self.nd_log['cyw'][-1] + radians(drotZ)
				cywnew = degrees(self.nd_log['cyw'][-1]) + drotZ
				cywnew = self.cyd(cywnew,0.0)					#centers the new cyw around 0
				cywnew = radians(cywnew)
				yaw=(cywnew+self.nd_log['cyw'][-1])/2.0			#yaw refers to yaw wrt to world frame
				du=dx*cos(yaw)-dy*sin(yaw)						#du, dv are displacement in  world frame
				dv=dx*sin(yaw)+dy*cos(yaw)
				self.nd_log['cpw'].append((self.nd_log['cpw'][-1][0]+du, self.nd_log['cpw'][-1][1]+dv, self.nd_log['cpw'][-1][2]+dz))
				self.nd_log['cyw'].append(cywnew)
				#print yaw
				#print self.nd_log['cpw'][-1]
		except:
			#self.drone_pos_valid = False
			print 'PositionController2.py: Exception: navdata callback error'
			

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
		#self.error['ps'].append(self.ref['ps']-msg.rotZ)


	def cmd_logger(self,twcmd):
		self.cmd_log['tm'].append(time()-self.reftm)
		self.cmd_log['tw'].append(twcmd)
	
	
	def refal_handler(self, refal):
		self.ref['al']=refal
		

	def cpw_cyw_handler(self, cpw, cyw):
		self.nd_log['cyw'].append(cyw)	#pose_handler uses cow[2] (z rot, euler) to determine rot of cmd frame wrt. world frame
		self.nd_log['cpw'].append(cpw)
		self.justgotpose = True
		print '\ngotpose - \nposition: ' + str(cpw) + '\norientation-z: ' + str(cyw) + '\ntime: ' + str(time()-self.reftm)
		#print 'gotpose'
	
	def cpw_handler(self, cpw):
		self.nd_log['cpw'].append(cpw)
		self.justgotpose = True
		print 'PositionController2.py: received cpw: ' + str(cpw)
		#print 'got cpw'
			
	def dpw_handler(self, dpw):
		self.dpw = dpw
		print 'PositionController2.py: set dpw = ', dpw
	
	def dyw_handler(self, dyw):
		self.dyw = dyw		#dyw is the required euler z rot of the drone wrt. world frame, in radians
		print 'PositionController2.py: set dyw = ', dyw


	def pc_timer_callback(self,event):
		if (self.nd_log['al'][-1]>(2000)):
			self.landpub.publish(Empty())	#height switch
			print 'PositionController: Error: altitude too high - landing'
		
		if self.refalon == True:
			self.twist.linear.z = max(min(0.0013*self.error['al'][-1], 1.0), -1.0)
		else:
			zerror = self.dpw[2]-self.nd_log['cpw'][-1][2]						#zerror is in meters
			#print 'zerror: ', zerror
			#print 'dpw: ', self.dpw[2]
			self.twist.linear.z = max(min(0.0013*zerror*1000, 1.0), -1.0)
		
		if self.yawon == True:
			yawwerror = -self.cyd(degrees(self.nd_log['cyw'][-1]),degrees(self.dyw))		# computes error in yaw world in degrees
			self.twist.angular.z = max(min(yawwerror/70, 0.3), -0.3) #130
			#print degrees(self.nd_log['psiw'][-1]),self.ref['ps']
			#print yawwerror
		
		if True:
			(xcw, ycw, zcw) = self.nd_log['cpw'][-1]
			(xdw, ydw, zdw) = self.dpw
			xrw = xdw - xcw
			yrw = ydw - ycw
			zrw = zdw - zcw
			psw = self.nd_log['cyw'][-1]				#psi world
			
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
			llim=0.5	#linear cmd limit
			self.twist.linear.x=max(min(rx,llim),-llim)
			self.twist.linear.y=max(min(ry,llim),-llim)
		
		else:
			pass
			#print 'calculate command error'
		

		self.cmdpub.publish(self.twist)
		self.cmd_logger(self.twist)
		#print 'twist: \n', self.twist
		
		
	def cyd(self, yaw, center):
		#compute yaw-desiyaw in interval (-180,180)
		rem = (yaw-center)%360
		if rem>180:
			rem=rem-360
		return rem
			
def main(args):
	pass

if __name__ == '__main__':
	main(sys.argv)
