#!/usr/bin/env python
import rospy
import sys,os,time
from std_msgs.msg import String
from ros_umirtx_driver.msg import *
from ros_umirtx_driver.srv import *
from ros_umirtx_vision.msg import *
from functools import partial
from conversion import RobParam
import math




def visionrecvcallback(gimod,data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard: x=%f y=%f area=%f", data.x, data.y, data.area)
	
	#gimod.xpos = 2*((data.x*0.01)-0.5)
	#gimod.ypos = 2*((data.y*0.01)-0.5)
	gimod.xpos = data.x*0.01
	gimod.ypos = data.y*0.01
	gimod.area = data.area
	
	return
	

def fillInMsg(respmsg,msg): #order is wrong TODO: rearrange
	msgarray = list(respmsg)
	msg.status   = msgarray[0]
	msg.elbow    = msgarray[1]
	msg.shoulder = msgarray[2]
	msg.zed      = msgarray[3]
	msg.wrist1   = msgarray[4]
	msg.wrist2   = msgarray[5]
	msg.yaw      = msgarray[6]
	msg.gripper  = msgarray[7]
	
def fillOutMsg(reqmsg,msglst): #order is wrong TODO: rearrange
	while len(msglst) > 0: msglst.pop() #clear list
	msglst.append(reqmsg.status)
	msglst.append(reqmsg.elbow)
	msglst.append(reqmsg.shoulder)
	msglst.append(reqmsg.zed)
	msglst.append(reqmsg.wrist1)
	msglst.append(reqmsg.wrist2)
	msglst.append(reqmsg.yaw)
	msglst.append(reqmsg.gripper)


#def robotreqcallback(gimod,event):
#	try:
#		resp1 = gimod.reqhandl(x, y)
#		return
#	except rospy.ServiceException, e:
#		print "Service call failed: %s"%e
#	return
	

class genint:
	
	def __init__(self):
		self.setup()
		self.MainFunc()
		
	def setup(self):
		print "Setting up..."
		self.xpos = 0
		self.ypos = 0
		self.area = 0
		self.robpar = RobParam()
		self.shscanmin = -80
		self.shscanmax = 45
		
		rospy.init_node('generalintelli', anonymous=True)
		self.rate = rospy.Rate(10) # 10hz
		
		try:
			rospy.wait_for_service('request_arm')
			self.reqhandl = rospy.ServiceProxy('request_arm', ArmRequest)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		rospy.Subscriber("visionposition", VisPMsg, partial(visionrecvcallback,self))
		print "Set up complete"
		
	def MainFunc(self):
		print "Starting controlschemes"
		
		self.initRobot()
		
		self.searchObject()
		
		#TODO in loop till low enough
		self.centerObject(700)
		
		#Rough lowering
		hight = 400
		while(hight>=100):
			self.robpar.goDown(hight)
			self.centerObject(hight,0.5,0.5)
			hight = hight -100
		hight = 50
		self.robpar.goDown(hight)
		self.centerObject(hight-10,0.3,0.6)
		self.robpar.goDown(27)
		self.moveMotTo()
		while(self.isBusy()):
			rospy.sleep(0.1)
			
		self.robpar.setGripperPC(30)
		self.moveMotTo()
		while(self.isBusy()):
			rospy.sleep(0.1)
		
		self.goToDropPosition()
		self.robpar.goDown(30)
		rospy.sleep(1)
		
		self.robpar.setGripperPC(70)
		self.moveMotTo()
		while(self.isBusy()):
			rospy.sleep(0.1)
		
		rospy.sleep(1)
		self.robpar.setZed(-1200)
		while(self.isBusy()):
			rospy.sleep(0.1)
		rospy.sleep(1)
		
		#Done
		
	def close(self):
		print "Closing..."
		#TODO
		return
		
	def req2Robot(self,msgtup):
		msg = ArmMsg()
		fillInMsg(msgtup,msg)
		
		resp = self.reqhandl(msg)
		
		print "response: ", resp.armresp.elbow
		return resp.armresp
		
	def moveMotTo(self,motcoord=[]):
		if not motcoord:
			motcoord = self.robpar.getRob()
		senddata = list(motcoord)
		senddata.insert(0,49)
		print "Data: ",senddata
		self.req2Robot(senddata)
		
	def isBusy(self):
		resp = self.req2Robot((34,0,0,0, 0,0,0,0))
		#if (resp.status == 65): #No problems occured?
		if((resp.elbow&1)>0): #Busy bit set?
			stat = True
		else:
			stat = False
		return stat
		
	def stopAndRelease(self):
		#resp = self.req2Robot([0,0,0,0,0,0,0,0]) #FREE_STOP
		resp = self.req2Robot([0,3,0,0,0,0,0,0]) #FREE_OFF
		return
	
	def testConn(self):
		self.req2Robot((66,0,0,0, 0,0,0,0))
		
		
	def initRobot(self):
		print "Initialising robot: "
		print "Connecting to serial..."
		senddata = [16,0,0,0, 0,0,0,0] #16 = init serial connection
		resp = self.req2Robot(senddata)
		#print "Resp = ",resp
		rospy.sleep(1)
		
		print "Initialising arm..."
		senddata = [17,0,0,0, 0,0,0,0] #17 = init arm
		resp = self.req2Robot(senddata)
		rospy.sleep(3)
		
		print "Going to start of search position"
		self.goToSearchPosition()
		#rospy.sleep(15)
		
		print "Initialised"
		return
		
	def goToSearchPosition(self):
		#Going to start position
		self.robpar.setReal(-60,self.shscanmin,690,-90,0,0,70)
		#motcoord = (300,1000,-500,2000,2000,0,600)
		#par = self.robpar.getRob()
		#print "Coord: ",par
		#self.moveMotTo(par)
		self.moveMotTo()
		while(self.isBusy()):
			rospy.sleep(0.1)
			
	def goToDropPosition(self):
		self.robpar.setReal(0,-90,300,-90,0,0,35)
		self.moveMotTo()
		while(self.isBusy()):
			rospy.sleep(0.1)
		
	def searchObject(self):
		print "Searching Object..."
		direction = 1
		minangle = 29.2227*self.shscanmin
		maxangle = 29.2227*self.shscanmax
		
		#Turn around shoulder till object is found
		found = 0
		while found==0:
			if(self.area>=30): #If object found
				#self.stopAndRelease()
				print "Found"
				break
			#Check if all motors axis is stoped
			if(not self.isBusy()): #If motors stopped
				#Change direction
				if(direction>0):
					direction = self.robpar.searchLR(direction,minangle,maxangle)
					par = self.robpar.getRob()
					self.moveMotTo(par)
					rospy.sleep(0.05)
				else:
					direction = self.robpar.searchLR(direction,minangle,maxangle)
					par = self.robpar.getRob()
					self.moveMotTo(par)
					rospy.sleep(0.05)
			rospy.sleep(0.05)
		return
		
	def centerObject(self,power=100,yc=0.5,xc=0.5,prec=0.05):
		print "Centering Object..."
		centered = False
		while (not centered):
			xpos = 2*((self.xpos)-xc)
			ypos = 2*((self.ypos)-yc)
			#while(self.area <=30): #if
			#	#TODO object lost
			#	rospy.sleep(0.1)
			lr = int(round(-xpos*power))
			ud = int(round(-ypos*power/2))
			print "x:  ",xpos,"  y: ",ypos
			print "lr: ",lr," ud: ",ud
			par = self.robpar.chLRUD(lr,ud)
			self.moveMotTo(par)
			while(self.isBusy()):
				rospy.sleep(0.1)
			#print "No longer busy"
			centered = ((abs(xpos)<=prec) and (abs(ypos)<=prec))
			#print "Centered?",centered
			#if(centered):
			#	break
		print "Centered: x=",xpos," y=",ypos
		return
		
	def lowerGripper(self):
		print "Lowering..."
		self.robpar.goDown(300)
		par = self.robpar.getRob()
		self.moveMotTo(par)
		print "waiting till no longer busy"
		while(self.isBusy()):
			rospy.sleep(0.1)
		print "Lowered"
		return
		

if __name__ == '__main__':
    try:
        genint()
    except rospy.ROSInterruptException:
        pass
