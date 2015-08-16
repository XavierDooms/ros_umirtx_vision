#!/usr/bin/env python
import rospy
import sys,os,time
from std_msgs.msg import String
from ros_umirtx_driver.msg import *
from ros_umirtx_driver.srv import *
from ros_umirtx_vision.msg import *
from functools import partial
from conversion import RobParam




def visionrecvcallback(gimod,data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard: x=%f y=%f area=%f", data.x, data.y, data.area)
	
	gimod.xpos = 2*(data.x-0.5)
	gimod.ypos = 2*(data.y-0.5)
	gimod.area = data.area
	
	return
	

def fillInMsg(respmsg,msg): #order is wrong TODO: rearrange
	msgarray = list(respmsg)
	msg.status   = msgarray[0]
	msg.shoulder = msgarray[1]
	msg.elbow    = msgarray[2]
	msg.zed      = msgarray[3]
	msg.wrist1   = msgarray[4]
	msg.wrist2   = msgarray[5]
	msg.yaw      = msgarray[6]
	msg.gripper  = msgarray[7]
	
def fillOutMsg(reqmsg,msglst): #order is wrong TODO: rearrange
	while len(msglst) > 0: msglst.pop() #clear list
	msglst.append(reqmsg.status)
	msglst.append(reqmsg.shoulder)
	msglst.append(reqmsg.elbow)
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
		self.shscanmin = -90
		self.shscanmax = 15
		
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
		self.centerObject()
		self.lowerGripper()
		self.centerObject()
		
		
		
		#Done
		
	def close(self):
		print "Closing..."
		#TODO
		return
		
	def req2Robot(self,msgtup):
		msg = ArmMsg()
		fillInMsg(msgtup,msg)
		
		resp = self.reqhandl(msg)
		
		print "response: ", resp.armresp.status
		return resp.armresp
		
	def moveMotTo(self,motcoord):
		sendarray = (49+motcoord)
		self.req2Robot(senddata)
		
	def isStopped(self):
		resp = self.req2Robot((34,0,0,0, 0,0,0,0))
		if (resp.status == 65): #No problems occured?
			if((resp.shoulder&1)>0): #Stopped bit set?
				stat = 1
			else
				stat = 0
		else
			stat = -1
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
		time.sleep(1)
		
		print "Initialising arm..."
		senddata = [17,0,0,0, 0,0,0,0] #17 = init arm
		resp = self.req2Robot(senddata)
		time.sleep(15)
		
		self.goToSearchPosition()
		time.sleep(3)
		
		print "Initialised"
		return
		
	def goToSearchPosition(self):
		#Going to start position
		self.robpar.setReal(self.shscanmin,90,700,-90,0,0,1200)
		#motcoord = (300,1000,-500,2000,2000,0,600)
		par = self.robpar.getRob()
		self.moveMotTo(par)
		while(self.isStopped() == 0):
			time.sleep(0.1)
		
	def searchObject(self):
		print "Searching Object..."
		direction = 0
		
		
		#Turn around shoulder till object is found
		found = 0
		while found==0:
			if(area>=1000): #If object found
				self.stopAndRelease()
				print "Found"
				break
			#Check if all motors axis is stoped
			if(self.isStopped()) #If motors stopped
				#Change direction
				if(direction>0):
					self.robpar.shoulder = 14.6113*self.shscanmax #TODO only change and send shoulder
					par = self.robpar.getRob()
					self.moveMotTo(par)
				else
					self.robpar.shoulder = 14.6113*self.shscanmax
					par = self.robpar.getRob()
					self.moveMotTo(par)
			time.sleep(0.1)
		return
		
	def centerObject(self):
		print "Centering Object..."
		while ((abs(self.xpos)<=0.1) and (abs(self.ypos)<=0.1)):
			if(self.area <=1000)
				#TODO object lost
				time.sleep(0.1)
			lr = int(round(self.xpos*15))
			ud = int(round(self.ypos*15))
			par = self.robpar.chLRUD(self,lr,ud)
			self.moveMotTo(par)
			time.sleep(0.1)
		print "Centered"
		return
		
	def lowerGripper(self):
		print "Lowering..."
		self.robpar.goDown1()
		
		print "Lowered"
		return
		

if __name__ == '__main__':
    try:
        genint()
    except rospy.ROSInterruptException:
        pass
