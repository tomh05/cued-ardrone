#!/usr/bin/env python

from time import time

class Template:
	def __init__(self, tm=0, kppt=[], desc=[], objc=[], objm=[], Q=[], S=[], mid=0):
		
		self.tm = round(time()*1000)	# time of creation
		self.kppt = kppt	# keypoints in pixel coordinates
		self.desc = desc	# descriptors
		self.objc = objc	# object coordinates in flattened camera frame
		self.objm = objm	# object coordinates in model (world) frame
		self.Q = Q			# transformation from camera frame to world frame
		self.S = S			# transformation from camera frame to world frame
		self.mid = mid		# model frame id
