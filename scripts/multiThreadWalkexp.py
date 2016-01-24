#!/usr/bin/env python
import Tkinter as tk
import time
import rospy
from math import cos
from math import fabs
import RobotFuncs
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from quadruped_control.msg import MotorAngles
import thread

iniTime =0
leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = 90,0,-90,0,90,0,-90,0
LeftH = 0.04
RightH = 0.04
LeftL = 0.06
RightL = 0.06
deltaLmax = 0.05
Orientation = 0.0
desiredOrientation =0.0
Ta = 0.4
Td = 0.8
ptd = [-0.02,-0.15]
ptd1 = [0.02,-0.15]
isStop = False
isTurn = False
State = 1

def onKeyPress(event):
	global leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2
	global iniTime,LeftH,RightH,LeftL,RightL,Ta,Td
	global ptd,ptd1,isStop
	global deltaLmax,Orientation,desiredOrientation
	#text.delete("1.0",tk.END)
	#text.insert('end', 'You pressed %s\n' % (event.char, ))
	if event.char =='q': #Left H
		LeftH = LeftH + 0.005
	elif event.char =='a': 
		LeftH = LeftH - 0.005
	elif event.char =='w': #Right H
		RightH = RightH + 0.005
	elif event.char =='s':
		RightH = RightH - 0.005	
	elif event.char =='e': #Left L
		LeftL = LeftL + 0.005
	elif event.char =='d':
		LeftL = LeftL - 0.005
	elif event.char =='r': #Right L
		RightL = RightL + 0.005
	elif event.char =='f':
		RightL = RightL - 0.005
	elif event.char =='t': #Ta
		Ta = Ta + 0.1
	elif event.char =='g':
		Ta = Ta - 0.1
	elif event.char =='y': #Td
		Td = Td + 0.1
	elif event.char =='h':
		Td = Td - 0.1
	elif event.char =='u': #Ptd X
		ptd[0] = ptd[0] + 0.005
		print ''
	elif event.char =='j':
		ptd[0] = ptd[0] - 0.005
		print ''	
	elif event.char =='i': #Ptd Y
		ptd[1] = ptd[1] + 0.005
		print ''
	elif event.char =='k': 
		ptd[1] = ptd[1] - 0.005
	elif event.char =='o': #Ptd1 X
		ptd1[0] = ptd1[0] + 0.005
	elif event.char =='l': 
		ptd1[0] = ptd1[0] - 0.005
	elif event.char =='p': #Ptd1 Y
		ptd1[1] = ptd1[1] + 0.005
	elif event.char ==';':
		ptd1[1] = ptd1[1] - 0.005
	elif event.char =='z':
		isStop = not isStop
	elif event.char ==',':#turn left
		LeftL = LeftL - 0.005
		RightL = RightL + 0.005
	elif event.char =='.':#turn right
		LeftL = LeftL + 0.005
		RightL = RightL - 0.005
	elif event.char =='x':
		desiredOrientation = -90.0
	elif event.char =='c':
		desiredOrientation = 0.0
	elif event.char =='v':
		desiredOrientation = 90.0




def OrientationCallback(data):
	global Orientation
	Orientation = data.x
	print data.x
	print Orientation
			
def MainProssece(*args):
	global leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2
	global iniTime,LeftH,RightH,LeftL,RightL,Ta,Td
	global ptd,ptd1
	global deltaLmax,Orientation,desiredOrientation
	pub1 = rospy.Publisher('MotorSetpoint', MotorAngles, queue_size=1024) 
	rospy.Subscriber("BodyOrientation", Point, OrientationCallback)
	rospy.init_node('reference')
	angles = MotorAngles()
	rate = rospy.Rate(50) # 10hz
	
	while not rospy.is_shutdown():
		global Orientation
		if iniTime ==0:
			iniTime = time.time()
		t = time.time() -iniTime
		if t <=5:
			
			leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = 90,0,-90,0,90,0,-90,0
			#rospy.loginfo("t = %s"%t)
			#print ("t = %s"%t)
			#print("Orientation = %s"%Orientation)
		elif t >5 and t <15:
			#print ("t = %s"%t)
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
			
			if (fabs(desiredOrientation-Orientation)) > 5:
				deltaL = (deltaLmax/fabs(desiredOrientation-Orientation))*(desiredOrientation-Orientation)
				print("deltaL 1=%s"%deltaL)
			else:
				deltaL = deltaLmax*(desiredOrientation-Orientation)/5.0
				#print("deltaL =%s"%deltaL)
			#	print("delta =%s"%(desiredOrientation-Orientation))
			#leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = RobotFuncs.generateRemoteCommand(ptd,ptd1,LeftH,RightH,LeftL+deltaL,RightL-deltaL,Ta,Td,t) 
			#rospy.loginfo("Reference : %s,\r\n%s,\t%s,\t%s,\t%s,\r\n%s,\t%s,\t%s,\t%s\r\n"%(t,leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2))
			#print("t = %s\r\nReference : %s,\t%s,\t%s,\t%s,\r\n%s,\t%s,\t%s,\t%s\r\n" 
			#%(t,leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2))
			#print("Right H = %s  Left H = %s  Right L = %s  Left L = %s  Ta = %s  Td = %s  ptd = %s  ptd1 = %s  isStop = %s"
			#%(RightH,LeftH,RightL-deltaL,LeftL+deltaL,Ta,Td,ptd,ptd1,isStop))
			#print("Orientation = %s"%Orientation)
		#outFile = open('ExpData.txt','a')
		#outFile.write(str(t)+'\t'+str(Orientation)+'\t'+str(desiredOrientation)+'\r\n')
		text.delete("1.0",tk.END)
		text.insert('end',"Orientation = %.3f\n"%Orientation)
		text.insert('end',"Desired Orientation = %.3f\n\n"%desiredOrientation)
		text.insert('end', "t = %.3f\n\nLeft H\tRight H\tLeft L\tRight L\tTa\tTd\tptd\t\tptd1\t\tisStop\n\
%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t[%.3f, %.3f]\t\t[%.3f, %.3f]\t\t%s"\
%(t,LeftH,RightH,LeftL,RightL,Ta,Td,ptd[0],ptd[1],ptd1[0],ptd1[1],isStop))	
		text.insert('end',"\n\nMotor Reference\n %.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f\n"%(leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2))
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

def TkProssece(*args):
	global root,text
	root = tk.Tk()
	root.geometry('1300x400')
	text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 16),height=400,width=1300)
	text.pack()
	root.bind('<KeyPress>', onKeyPress)
	root.mainloop()
if __name__ == '__main__':
    try:
		thread.start_new_thread(TkProssece,(0,0))
		MainProssece()
		
    except rospy.ROSInterruptException:
        pass



