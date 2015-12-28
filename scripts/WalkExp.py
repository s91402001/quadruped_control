#!/usr/bin/env python

import rospy
from math import cos
from math import fabs
import time
import RobotFuncs
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from quadruped_control.msg import MotorAngles


iniTime =0
leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = 90,0,-90,0,90,0,-90,0
LeftH = 0.04
RightH = 0.04
LeftL = 0.06
RightL = 0.06
deltaLmax = 0.05
Orientation = 0.0
desiredOrientation =0.0
Ta = 0.35
Td = 0.8
ptd = [-0.02,-0.15]
ptd1 = [0.0,-0.15]
isStop = False
isTurn = False
State = 1

def RightHCallback(data):
	global RightH
	RightH = RightH + round(data.data,3)
	print data.data
def LeftHCallback(data):
	global LeftH
	LeftH = LeftH + round(data.data,3)
	print data.data
def LeftLCallback(data):
	global LeftL
	LeftL = LeftL + round(data.data,3)
	print data.data
def RightLCallback(data):
	global RightL
	RightL = RightL + round(data.data,3)
	print data.data
def TaCallback(data):
	global Ta
	Ta = Ta + round(data.data,1)
	print data.data
def TdCallback(data):
	global Td
	Td = Td + round(data.data,1)
	print data.data
def ptdxCallback(data):
	global ptd
	ptd[0] = ptd[0] + round(data.data,3)
	print data.data
def ptdyCallback(data):
	global ptd
	ptd[1] = ptd[1] + round(data.data,3)
	print data.data
def ptd1xCallback(data):
	global ptd1
	ptd1[0] = ptd1[0] + round(data.data,3)
	print data.data
def ptd1yCallback(data):
	global ptd1
	ptd1[1] = ptd1[1] + round(data.data,3)
	print data.data
def isStopCallback(data):
	global isStop
	isStop = not isStop
	print data.data
def isTurnCallback(data):
	global isTurn
	print data.data
def OrientationCallback(data):
	global Orientation
	Orientation = data.x
	print data.x
	print Orientation
def desiredOrientationCallback(data):
	global desiredOrientation
	desiredOrientaion = data.data
	print data.data
			
def MainProssece():
	global leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2
	global iniTime,LeftH,RightH,LeftL,RightL,Ta,Td
	global ptd,ptd1
	global deltaLmax,Orientation,desiredOrientation
	pub1 = rospy.Publisher('MotorSetpoint', MotorAngles, queue_size=1024) 
	rospy.init_node('reference')
	rospy.Subscriber("LeftH", Float32, LeftHCallback)
	rospy.Subscriber("RightH", Float32, RightHCallback)
	rospy.Subscriber("LeftL", Float32, LeftLCallback)
	rospy.Subscriber("RightL", Float32, RightLCallback)
	rospy.Subscriber("Ta", Float32, TaCallback)
	rospy.Subscriber("Td", Float32, TdCallback)
	rospy.Subscriber("ptdx", Float32, ptdxCallback)
	rospy.Subscriber("ptdy", Float32, ptdyCallback)
	rospy.Subscriber("ptd1x", Float32, ptd1xCallback)
	rospy.Subscriber("ptd1y", Float32, ptd1yCallback)
	rospy.Subscriber("isStop", Float32, isStopCallback)
	rospy.Subscriber("BodyOrientation", Point, OrientationCallback)
	angles = MotorAngles()
	rate = rospy.Rate(50) # 10hz
	
	while not rospy.is_shutdown():
		if iniTime ==0:
			iniTime = time.time()
		t = time.time() -iniTime
		if t <=5:
			global Orientation
			leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = 90,0,-90,0,90,0,-90,0
			#rospy.loginfo("t = %s"%t)
			print ("t = %s"%t)
			print("Orientation = %s"%Orientation)
		elif t >5 and t <15:
			print ("t = %s"%t)
			x1 = 0
			x2 = 0
			x3 = 0
			x4 = 0
			y1 = -0.16
			y2 = -0.16
			y3 = -0.16
			y4 = -0.16
			leg1th1_des,leg1th2_des,leg2th1_des,leg2th2_des ,leg3th1_des,leg3th2_des,leg4th1_des,leg4th2_des = RobotFuncs.actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
			tt =t-5			
			leg1th1 = 90 + (leg1th1_des-90)*tt/10
			leg2th1 = -90 + (leg2th1_des+90)*tt/10			
			leg3th1 = 90 + (leg3th1_des-90)*tt/10
			leg4th1 = -90 + (leg4th1_des+90)*tt/10
			leg1th2 = leg1th2_des*tt*0/10
			leg2th2 = leg2th2_des*tt*0/10
			leg3th2 = leg3th2_des*tt*0/10
			leg4th2 = leg4th2_des*tt*0/10
		else :
			
			if (fabs(desiredOrientation-Orientation)) > 1:
				deltaL = (deltaLmax/fabs(desiredOrientation-Orientation))*(desiredOrientation-Orientation)
				print("deltaL 1=%s"%deltaL)
			else:
				deltaL = deltaLmax*(desiredOrientation-Orientation)
				#print("deltaL =%s"%deltaL)
				print("delta =%s"%(desiredOrientation-Orientation))
			leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = RobotFuncs.generateRemoteCommand(ptd,ptd1,LeftH,RightH,LeftL+deltaL,RightL-deltaL,Ta,Td,t) 
			#rospy.loginfo("Reference : %s,\r\n%s,\t%s,\t%s,\t%s,\r\n%s,\t%s,\t%s,\t%s\r\n"%(t,leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2))
			print("t = %s\r\nReference : %s,\t%s,\t%s,\t%s,\r\n%s,\t%s,\t%s,\t%s\r\n" 
			%(t,leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2))
			print("Right H = %s  Left H = %s  Right L = %s  Left L = %s  Ta = %s  Td = %s  ptd = %s  ptd1 = %s  isStop = %s"
			%(RightH,LeftH,RightL-deltaL,LeftL+deltaL,Ta,Td,ptd,ptd1,isStop))
		angles.ang1 = leg1th1
		angles.ang2 = leg2th1
		angles.ang3 = leg3th1
		angles.ang4 = leg4th1
		angles.ang5 = leg1th2
		angles.ang6 = leg2th2
		angles.ang7 = leg3th2
		angles.ang8 = leg4th2
		pub1.publish(angles)
		rate.sleep()



if __name__ == '__main__':
    try:
		
		MainProssece()
		
    except rospy.ROSInterruptException:
        pass
