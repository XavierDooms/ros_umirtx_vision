#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ros_umirtx_driver.msg import *
from ros_umirtx_driver.srv import *
import sys, os
import time

#TODO: min maxes rounds
class RobParam:
	
	def __init__(self, name):
		#All values stored in servo values
		self.shoulder = int(0)
		self.elbow    = int(0)
		self.zed      = int(-120)
		self.wrist1   = int(0)
		self.wrist2   = int(0)
		self.yaw      = int(0)
		self.gripper  = int(1000)
		
		self.zmin1    = int(50) #mm min hight first fase (Approach)
		self.zmin2    = int(25) #mm min hight second fase (Grab)
		
	#Sets new robot values
	def setRob(self,shoulder,elbow,zed,wrist1,wrist2,yaw,gripper):
		self.shoulder = shoulder
		self.elbow    = elbow
		self.zed      = zed
		self.wrist1   = wrist1
		self.wrist2   = wrist2
		self.yaw      = yaw
		self.gripper  = gripper
		return
		
	#Gets the current robot values
	def getRob(self):
		return [self.shoulder,self.elbow,self.zed,self.wrist1,self.wrist2,self.yaw,self.gripper]
		
	#Sets the robot values with real world coordinates (mm & degrees)
	def setReal(self,shoulder,elbow,zed,pitch,roll,yaw,gripper):	
		self.shoulder =  int(round(14.6113*shoulder)))
		self.elbow    =  int(round(29.2227*elbow))
		self.zed      =  int(round(3.74953*(915-zed)))
		self.wrist1   =  int(round(13.4862*(pitch+roll)))
		self.wrist2   =  int(round(13.4862*(pitch-roll)))
		self.yaw      =  int(round(9.73994*yaw))
		self.gripper  =  int(round(gripper)) #TODO
		return
	
	#Get real world values
	def getReal(self):
		robsho = self.shoulder/14.6113
		robelb = self.elbow/29.2227
		robzed = 915+self.zed/3.74953
		robpit = (self.wrist1+self.wrist2)/13.4862
		robroll= (self.wrist1-self.wrist2)/13.4862
		robyaw = self.yaw/9.73994
		robgri = self.gripper #TODO
		return [robsho,robelb,robzed,robpit,robroll,robyaw,robgr]
		
	def chLRUD(self,lr=0,ud=0):
		self.shoulder = self.shoulder + lr - ud
		self.elbow = self.elbow + 2*ud
		return [self.shoulder,self.elbow,self.zed,self.wrist1,self.wrist2,self.yaw,self.gripper]
		
	def goDown1(self,value=100)
		iv = int(round(value))
		self.zed = self.zed - iv
		hgrip = int(round(177*math.sin(radians((self.wrist1+self.wrist2)/13.4862))))
		if(self.zed <= (self.zmin1+hgrip))
			self.zed = self.zmin1+hgrip
			return True
		return False
		
	#Calculates the robot paramters with its x,y,z coordinates and the desired grip angle (yaw,pitch,roll)
	def xyzypr2rob(self,xtip,ytip,ztip,yawd,pitchd,rolld):
		#Constants and radians values
		lgrip  = 177
		yawr   = math.radians(yawd)
		pitchr = math.radians(pitchd)
		rollr  = math.radians(rolld)
		
		#x,y,z of wrist to grip
		xgrip  = lgrip*math.cos(pitchr)*math.cos(yawr)
		ygrip  = lgrip*math.cos(pitchr)*math.sin(yawr)
		zgrip  = lgrip*math.sin(pitchr)
		
		#x,y,z of base to wrist
		xwrist = xtip - xgrip
		ywrist = ytip - ygrip
		zwrist = ztip - zgrip
		
		#Angles to get to wrist
		radius = math.hypot(xwrist,ywrist)
		thetax = math.atan(ywrist,xwrist)
		thetay = math.acos(radius/(2*253.5))
		
		theta2 = thetax + thetay
		theta3 = 2*thetay
		theta4 = yaw-thetax
		
		#Conversion to robot parameters
		robsho = -14.6113*theta2*(180/math.pi)
		robelb =  29.2227*theta3*(180/math.pi)
		robyaw =  9.73994*theta4*(180/math.pi)
		robwr1 =  13.4862*(pitchd+rolld)
		robwr2 =  13.4862*(pitchd-rolld)
		robzed =  3.74953*(915-zwrist)
		
		return  int(round([robsho,robelb,robzed,robwr1,robwr2,robyaw]))
		
	def setxyzypr2rob(self,x,y,z,yaw,pitch,roll)
		param = self.xyzypr2rob(x,y,z,yaw,pitch,roll)
		self.shoulder = param(1)
		self.elbow    = param(2)
		self.zed      = param(3)
		self.wrist1   = param(4)
		self.wrist2   = param(5)
		self.yaw      = param(6)
	
	#Calculates the robot paramters with an x,y,z coordinates and the desired grip angle (pitch,roll) yaw extended
	def xyzpr2rob1(self,xtip,ytip,ztip,pitchd,rolld):
		#Constants and radians values
		lgrip  = 177
		#yawr    = math.radians(yawd)
		pitchr  = math.radians(pitchd)
		rollr   = math.radians(rolld)
		
		#Poolcoord of wrist
		thg    = math.atan2(ytip,xtip)
		radgr  = lgrip*math.cos(pitchr)
		radti  = math.hypot(xtip,ytip)
		radwr  = radti - radgr
		
		#Angle of base to wrist and angle of elbow
		thetax = thg
		thetay = math.acos(radwr/(2*253.5))
		
		theta2 = thetax + thetay
		theta3 = 2*thetay
		#theta4 = yaw-thetax
		
		robsho = -14.6113*theta2*(180/math.pi)
		robelb =  29.2227*theta3*(180/math.pi)
		robzed =  3.74953*(915-zwrist)
		robwr1 =  13.4862*(pitchd+rolld)
		robwr2 =  13.4862*(pitchd-rolld)
		#robyaw =  9.73994*theta4*(180/math.pi)
		
		return  int(round([robsho,robelb,robzed,robwr1,robwr2,0]))
	
	def setxyzpr2rob1(self,x,y,z,yaw,pitch,roll)
		param = self.xyzpr2rob1(x,y,z,yaw,pitch,roll)
		self.shoulder = param(1)
		self.elbow    = param(2)
		self.zed      = param(3)
		self.wrist1   = param(4)
		self.wrist2   = param(5)
		self.yaw      = param(6)
