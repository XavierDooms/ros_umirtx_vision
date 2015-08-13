#!/usr/bin/env python
import rospy
import sys,os
from std_msgs.msg import String
from ros_umirtx_driver.msg import *
from ros_umirtx_driver.srv import *
from ros_umirtx_vision.msg import *
from functools import partial
import time




def visionrecvcallback(gimod,data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard: x=%f y=%f area=%f", data.x, data.y, data.area)
	
	gimod.xpos = data.x
	gimod.ypos = data.y
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
		self.xpos = 0.5
		self.ypos = 0.5
		self.area = 0
		
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
		#rospy.Timer(rospy.Duration(1), partial(robotreqcallback,self))
		#while not rospy.is_shutdown():
		#	hello_str = "hello world %s" % rospy.get_time()
		#	rospy.loginfo(hello_str)
		#	pub.publish(hello_str)
		#	rate.sleep()
		#rospy.spin()
		
		self.initRobot()
		self.searchObject()
		
		
	def close(self):
		print "Closing..."
		return
		
	def req2Robot(self,msgtup):
		msg = ArmMsg()
		#fillInMsg(msgtup,msg)
		msg.status   = 5
		msg.shoulder = 0
		msg.elbow    = 0
		msg.zed      = 0
		msg.wrist1   = 0
		msg.wrist2   = 0
		msg.yaw      = 0
		msg.gripper  = 0
		
		resp = self.reqhandl(msg)
		
		print "response: ", resp.armresp.status
		return resp
		
		#try:
		#	resp1 = self.reqhandl(msg)
		#	print "Response: " resp1
		#	return
		#except rospy.ServiceException, e:
		#	print "Service call failed: %s"%e
		#return
		
		
	def initRobot(self):
		print "Initialising robot: "
		print "Connecting to serial..."
		#senddata = [5,0,0,0, 0,0,0,0] #5 = init serial connection
		#resp = self.req2Robot(senddata)
		msg = ArmRequestRequest()
		msg.armreq.status   = 5
		msg.armreq.shoulder = 0
		msg.armreq.elbow    = 0
		msg.armreq.zed      = 0
		msg.armreq.wrist1   = 0
		msg.armreq.wrist2   = 0
		msg.armreq.yaw      = 0
		msg.armreq.gripper  = 0
		
		resp = self.reqhandl(msg)
		time.sleep(1)
		
		print "Initialising arm..."
		senddata = [6,0,0,0, 0,0,0,0] #5 = init serial connection
		resp = self.req2Robot(senddata)
		time.sleep(15)
		
		return
		
	def searchObject(self):
		print "Searching Object..."
		motcoord = (-500,-300,300,200,200,-200,600)
		self.moveMotTo(motcoord)
		
		return
		
	def moveMotTo(self,motcoord):
		sendarray = (20+motcoord)
		self.req2Robot(senddata)
	
	def testConn(self):
		self.req2Robot((25,0,0,0, 0,0,0,0))
		
	def getUserInput(self):
		loopTest = 0
		while (loopTest==0):
			uistr = input("Give command: ")
			if(len(uistr)!=0):
				pos = uistr.find(' ')
				if(pos==-1):
					comm = uistr
					var  = ""
				else:
					comm = uistr[0:pos-1]
					var  = uistr[pos+1:]
				loopTest = self.execUsInput(comm,var)
		
	def execUsInput(self,comm,var):
		
		if   (comm=="test"):
			print "Testing connection"
			self.testConn()
			
		elif (comm=="mov1"):
			print "Moving to position 1"
			motcoord = (-500,-300,300,200,200,-200,600)
			self.moveMotTo(motcoord)
			
		elif (comm=="mov2"):
			print "Moving to position 2"
			motcoord = (-500,-300,300,200,200,-200,600)
			self.moveMotTo(motcoord)
			
		elif (comm=="mov"):
			print "Moving to position"
			motcoord = (-500,-300,300,200,200,-200,600)
			self.moveMotTo(motcoord)
			
		else:
			print "No command found for input"
	

if __name__ == '__main__':
    try:
        genint()
    except rospy.ROSInterruptException:
        pass
