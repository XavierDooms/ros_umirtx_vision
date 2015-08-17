#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ros_umirtx_driver.msg import *
from ros_umirtx_driver.srv import *
import sys, os
import time
import math

#TODO: min maxes rounds
class RobParam:
	
	def __init__(self):
		#All values stored in servo values
		self.elbow    = int(0)
		self.shoulder = int(0)
		self.zed      = int(-120)
		self.wrist1   = int(0)
		self.wrist2   = int(0)
		self.yaw      = int(0)
		self.gripper  = int(600)
		
		self.zmin1    = int(100) #mm min hight first fase (Approach)
		self.zmin2    = int(25) #mm min hight second fase (Grab)
		
	#Sets new robot values
	def setRob(self,elbow,shoulder,zed,wrist1,wrist2,yaw,gripper):
		self.elbow    = elbow
		self.shoulder = shoulder
		self.zed      = zed
		self.wrist1   = wrist1
		self.wrist2   = wrist2
		self.yaw      = yaw
		self.gripper  = gripper
		return
		
	#Gets the current robot values
	def getRob(self):
		return [self.elbow,self.shoulder,self.zed,self.wrist1,self.wrist2,self.yaw,self.gripper]
		
	#Sets the robot values with real world coordinates (mm & degrees)
	def setReal(self,elbow,shoulder,zed,pitch,roll,yaw,gripper):
		self.elbow    =  int(round(14.6113*elbow))	
		self.shoulder =  int(round(29.2227*shoulder))
		self.zed      =  int(round(3.74953*(zed-915)))
		self.wrist1   =  int(round(13.4862*(pitch+roll)))
		self.wrist2   =  int(round(13.4862*(pitch-roll)))
		self.yaw      =  int(round(9.73994*yaw))
		self.gripper  =  int(round(12*gripper)) #TODO
		return
	
	#Get real world values
	def getReal(self):
		robelb = self.elbow*0.06844
		robsho = self.shoulder*0.03422
		robzed = 915+self.zed*0.2667
		robpit = (self.wrist1+self.wrist2)*0.07415/2
		robroll= (self.wrist1-self.wrist2)*0.07415/2
		robyaw = self.yaw*0.10267
		robgri = self.gripper/12 #TODO
		return [robelb,robsho,robzed,robpit,robroll,robyaw,robgr]
		
	def searchLR(self,direction,minangle,maxangle,step=400):
		if(direction>0):
			self.shoulder = self.shoulder + int(round(step))
			if(self.shoulder > (maxangle)):
				self.shoulder = int(round(maxangle))
				return -1
			return 1
		else:
			self.shoulder = self.shoulder - int(round(step))
			if(self.shoulder < (minangle)):
				self.shoulder = int(round(minangle))
				return 1
			return -1
		
	def chLRUD(self,lr=0,ud=0):
		print "lr: ",lr," ud: ",ud
		elb = self.elbow + ud
		sho = self.shoulder + lr
		if(elb>=0):
			self.elbow = int(0)
			self.shoulder = int(sho)
		else:
			self.elbow = int(elb)
			self.shoulder = int(sho)
		return [self.elbow,self.shoulder,self.zed,self.wrist1,self.wrist2,self.yaw,self.gripper]
		
	def goDown(self,value=100):
		#iv = int(round(value))
		#self.zed = self.zed - iv
		angle = (self.wrist1+self.wrist2)*0.07415/2
		hgrip = int(round(-177*math.sin(math.radians(angle))))
		zmin = int(round(3.74953*(value+hgrip-915)))
		#print "zmin:",zmin," angle:",angle," hgrip:",hgrip
		if(zmin<-2700):
			zmin=-2700
		#if(self.zed <= (zmin)):
		self.zed = zmin
		#	return True
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
		robelb = -14.6113*theta3*(180/math.pi)
		robsho =  29.2227*theta2*(180/math.pi)
		robyaw =  9.73994*theta4*(180/math.pi)
		robwr1 =  13.4862*(pitchd+rolld)
		robwr2 =  13.4862*(pitchd-rolld)
		robzed =  3.74953*(915-zwrist)
		
		return  int(round([robelb,robsho,robzed,robwr1,robwr2,robyaw]))
		
	def setxyzypr2rob(self,x,y,z,yaw,pitch,roll):
		param = self.xyzypr2rob(x,y,z,yaw,pitch,roll)
		self.elbow    = param(1)
		self.shoulder = param(2)
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
		
		robelb = -14.6113*theta3*(180/math.pi)
		robsho =  29.2227*theta2*(180/math.pi)
		robzed =  3.74953*(915-zwrist)
		robwr1 =  13.4862*(pitchd+rolld)
		robwr2 =  13.4862*(pitchd-rolld)
		#robyaw =  9.73994*theta4*(180/math.pi)
		
		return  int(round([robelb,robsho,robzed,robwr1,robwr2,0]))
	
	def setxyzpr2rob1(self,x,y,z,yaw,pitch,roll):
		param = self.xyzpr2rob1(x,y,z,yaw,pitch,roll)
		self.elbow    = param(1)
		self.shoulder = param(2)
		self.zed      = param(3)
		self.wrist1   = param(4)
		self.wrist2   = param(5)
		self.yaw      = param(6)
