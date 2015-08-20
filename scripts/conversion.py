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
		self.setElbow(elbow)
		self.setShoulder(shoulder)
		self.setZed(zed)
		self.setWrists(wrist1,wrist2)
		self.setYaw(yaw)
		self.setGripper(gripper)
		return
		
	def setRobLst(self,param):
		self.setElbow(param[0])
		self.setShoulder(param[1])
		self.setZed(param[2])
		self.setWrists(param[3],param[4])
		self.setYaw(param[5])
		if(len(param)>=7):
			self.setGripper(param[6])
				
	def setRobLstPR(self,param):
		self.setElbow(param[0])
		self.setShoulder(param[1])
		self.setZed(param[2])
		self.setPitchRoll(param[3],param[4])
		self.setYaw(param[5])
		if(len(param)>=7):
			self.setGripper(param[6])
		
	#Gets the current robot values
	def getRob(self):
		return [self.elbow,self.shoulder,self.zed,self.wrist1,self.wrist2,self.yaw,self.gripper]
		
	#Sets the robot values with real world coordinates (mm & degrees)
	def setReal(self,elbow,shoulder,zed,pitch,roll,yaw,gripper):
		self.setElbow(14.6113*elbow)
		self.setShoulder(29.2227*shoulder)
		self.setZed(3.74953*(zed-915))
		self.setPitchRoll(13.4862*pitch,13.4862*roll)
		self.setYaw(9.73994*yaw)
		self.setGrippermm(gripper) #TODO
		return
	
	#Get real world values
	def getReal(self):
		robelb = self.elbow*0.06844
		robsho = self.shoulder*0.03422
		robzed = 915+self.zed*0.2667
		robpit = (self.wrist1+self.wrist2)*0.07415/2
		robroll= (self.wrist1-self.wrist2)*0.07415/2
		robyaw = self.yaw*0.10267
		robgri = self.gripper*90/12 #TODO check correctness (manual = unclear. said non-linear)
		return [robelb,robsho,robzed,robpit,robroll,robyaw,robgri]
		
	def searchLR(self,direction,minangle,maxangle,step=400):
		if(direction>0):
			self.setShoulder(self.shoulder + int(round(step)))
			if(self.shoulder > (maxangle)):
				self.setShoulder(maxangle)
				return -1
			return 1
		else:
			self.setShoulder(self.shoulder - int(round(step)))
			if(self.shoulder < (minangle)):
				self.setShoulder(minangle)
				return 1
			return -1
		
	def chLRUD(self,lr=0,ud=0):
		print "lr: ",lr," ud: ",ud
		elb = self.elbow + ud
		sho = self.shoulder + lr
		if(elb>=0):
			self.setElbow(0)
			self.setShoulder(sho)
		else:
			self.setElbow(elb)
			self.setShoulder(sho-ud)
		return [self.elbow,self.shoulder,self.zed,self.wrist1,self.wrist2,self.yaw,self.gripper]
		
	def moveZedmm(self,value=100): #Protects against hight of groundplate with gripper
		angle = (self.wrist1+self.wrist2)*0.07415/2
		hgrip = int(round(-177*math.sin(math.radians(angle))))
		zmin = int(round(3.74953*(value+hgrip-915)))
		#print "zmin:",zmin," angle:",angle," hgrip:",hgrip
		if(zmin<-2700):
			zmin=-2700
		self.setZed(zmin)
		return False
		
	def setElbow(self,value):
		if(self.elbow > 2206):
			self.elbow = int(round(2206))
		elif(self.elbow < -2630):
			self.elbow = int(round(-2630))
		else:
			self.elbow = int(round(value))
		
	def setShoulder(self,value):
		if(self.shoulder > 2630):
			self.shoulder = int(round(2630))
		elif(self.shoulder < -2630):
			self.shoulder = int(round(-2630))
		else:
			self.shoulder = int(round(value))
		
	def setZed(self,value):
		if(self.zed > 0):
			self.zed = int(round(0))
		elif(self.zed < -3554):
			self.zed = int(round(-3554))
		else:
			self.zed = int(round(value))
			
	def setPitchRoll(self,pitch,roll):
		#print "Pitch:",pitch," Roll:",roll
		if(pitch > 108):
			wr1 = 54
			wr2 = 54
		elif(pitch < -2642):
			wr1 = -1321
			wr2 = -1321
		else:
			wr1 = pitch
			wr2 = pitch
		
		if(roll > 4882):
			wr1 = wr1 + 2441
			wr2 = wr2 - 2441
		elif(roll < -3560):
			wr1 = wr1 - 1780
			wr2 = wr2 + 1780
		else:
			wr1 = wr1 + roll
			wr2 = wr2 - roll
				
		#print "wr1:",wr1," wr2:",wr2
		self.wrist1 = int(round(wr1))
		self.wrist2 = int(round(wr2))
			
			
	def setWrists(self,wr1,wr2):
		pitch = wr1 + wr2
		roll  = wr1 - wr2
		self.setPitchRoll(pitch,roll)
		
		
	def setWrist1(self,value): #not correct
		self.wrist1 = int(round(value))
		if(self.wrist1 > 2642):
			self.wrist1 = int(round(2642))
		elif(self.wrist1 < -2642):
			self.wrist1 = int(round(-2642))
		
	def setWrist2(self,value): #not correct
		self.wrist2 = int(round(value))
		if(self.wrist2 > 2642):
			self.wrist2 = int(round(2642))
		elif(self.wrist2 < -2642):
			self.wrist2 = int(round(-2642))
		
	def setYaw(self,value):
		self.yaw = int(round(value))
		if(self.yaw > 1071+(self.elbow/3)):
			self.yaw = int(round(1071+(self.elbow/3)))
		elif(self.yaw < -1071+(self.elbow/3)):
			self.yaw = int(round(-1071+(self.elbow/3)))
		
	def setGripper(self,value):
		self.gripper = int(round(value))
		if(self.gripper > 1200):
			self.gripper = int(round(1200))
		elif(self.gripper<-30):
			self.gripper = int(round(-30))
		#print "grip:",self.gripper
			
	def setGripperPC(self,PC=100):
		self.setGripper(PC*12)
		
	def setGrippermm(self,mm):
		self.setGripper(mm*1200/90)
		#TODO make correct
		
	#Calculates the robot paramters with its x,y,z coordinates and the desired grip angle (yaw,pitch,roll)
	def xyzypr2rob(self,xtip,ytip=30,ztip=50,yawd=0,pitchd=0,rolld=0,lgrip=177):
		#Constants and radians values
		#lgrip  = 177 #can vary when gripper is open or if gripping with "tips" or "jaw"
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
		thetax = math.atan2(ywrist,xwrist)
		thetay = math.acos(radius/(2*253.5))
		
		theta2 = thetax + thetay
		theta3 = 2*thetay
		theta4 = yawr-thetax
		
		#Conversion to robot parameters
		robelb = -14.6113*theta3*(180/math.pi)
		robsho =  29.2227*theta2*(180/math.pi)
		robyaw =   9.73994*theta4*(180/math.pi)
		#robwr1 =  13.4862*(pitchd+rolld)
		#robwr2 =  13.4862*(pitchd-rolld)
		robpit =  13.4862*(pitchd)
		robrol =  13.4862*(rolld)
		robzed =   3.74953*(zwrist-915)
		
		return  [robelb,robsho,robzed,robpit,robrol,robyaw]
		
	def setxyzypr2rob(self,x,y,z,yaw,pitch,roll):
		param = self.xyzypr2rob(x,y,z,yaw,pitch,roll)
		self.setRobLstPR(param)
	
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
		#robwr1 =  13.4862*(pitchd+rolld)
		#robwr2 =  13.4862*(pitchd-rolld)
		robpit =  13.4862*(pitchd)
		robrol =  13.4862*(rolld)
		#robyaw =  9.73994*theta4*(180/math.pi)
		
		return  int(round([robelb,robsho,robzed,robpit,robpit,0]))
	
	def setxyzpr2rob1(self,x,y,z,yaw,pitch,roll):
		param = self.xyzpr2rob1(x,y,z,yaw,pitch,roll)
		self.setRobLstPR(param)
		
	def getxyzyprRob(self):
		
		
		return
