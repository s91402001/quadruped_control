#!/usr/bin/env python
import Tkinter as tk
import time
import rospy
from math import cos
from math import fabs
import RobotFuncs
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from quadruped_control.msg import MotorAngles
import thread

iniTime =0
leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = 90,0,-90,0,90,0,-90,0
LeftH = 0.04
RightH = 0.04
LeftL = 0.04
RightL = 0.04
deltaLmax = 0.02
Orientation = 0.0
desiredOrientation =0.0
Ta = 0.3
Td = 0.8
ptd = [-0.02,-0.15]
ptd1 = [0.02,-0.15]
isStop = False
isTurn = False
State = 1
pawls = 80
pawlAng = Float32()
pawlAng.data = pawls
Ang9 = Float32()
x1 = 0
x2 = 0
x3 = 0
x4 = 0
y1 = -0.16
y2 = -0.16
y3 = -0.16
y4 = -0.16
t=0


def onKeyPress(event):
	global pawlAng,pawls
	global Ang9,pub2,pub3,iniTime
	t_start = time.time() -iniTime
	t = time.time() -iniTime
	if event.char =='q':
		if pawls ==80:
			pawls =70
			pawlAng.data = pawls
			pub2.publish(pawlAng)
			print("%d\n"%pawls)
		else:
			pawls = 80
			pawlAng.data = pawls
			pub2.publish(pawlAng)
			print("%d\n"%pawls)
		
	elif event.char =='a': 
		Ang9.data = Ang9.data + 100
	elif event.char =='s': 
		Ang9.data = Ang9.data - 100
		




def OrientationCallback(data):
	global Orientation,label_Orientation_Value
	Orientation = data.x
	label_Orientation_Value["text"]  = str(round(Orientation,2))


def MotorAnglesCallback(data):
	global label_ActualAng_Value
	a1 = data.ang1
	a2 = data.ang2
	a3 = data.ang3
	a4 = data.ang4
	a5 = data.ang5
	a6 = data.ang6
	a7 = data.ang7
	a8 = data.ang8
	label_ActualAng_Value["text"] = str(round(a1,1)) + ' , ' + str(round(a2,1)) + ' , ' + str(round(a3,1)) + ' , ' + str(round(a4,1)) + ' , '\
	 + str(round(a5,1)) + ' , ' + str(round(a6,1)) + ' , ' + str(round(a7,1)) + ' , ' + str(round(a8,1)) 
	 
def StretchSpringCallback(data):
	global label_ActualStretchAng_Value
	label_ActualStretchAng_Value["text"] = str(round(data.data,1)) 
	 
		
def MainProssece(*args):
	global leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2, angles
	global iniTime,LeftH,RightH,LeftL,RightL,Ta,Td
	global ptd,ptd1
	global deltaLmax,Orientation,desiredOrientation,Ang9,pawlAng,pawls,pub1,pub2,pub3,pub4
	global x1,x2,x3,x4,y1,y2,y3,y4
	global label_RefAng_Value,label_RefStretchAng_Value, label_PawlState_Value, label_time_Value
	global t
	pub1 = rospy.Publisher('MotorSetpoint', MotorAngles, queue_size=1024) 
	pub2 = rospy.Publisher('ServoSetpoint',Float32 , queue_size=1024) 
	pub3 = rospy.Publisher('SpringMotorSetpoint',Float32 , queue_size=1024)
	rospy.Subscriber("BodyOrientation", Point, OrientationCallback)
	rospy.init_node('reference')
	angles = MotorAngles()

	rate = rospy.Rate(20) # 10hz
	
	while not rospy.is_shutdown():
		global Orientationt,t
		if iniTime ==0:
			iniTime = time.time()
		t = time.time() -iniTime
		
		if t > 10 and t < 13:
			delY = -0.16 + (0.05/3.0) *(t-10)
			#delX = 0 + (-0.02)/3.0 *(t-10)
			delX= 0;
			y1,y2,y3,y4 = delY, delY, delY, delY
			x1,x2 = delX, delX
		if t > 14 and t < 14.1:
			#y1,y2 = -0.1376, -0.1376
			#x1,x2 = 0.0816, 0.0816
			y1,y2 = -0.16, -0.16
			x1,x2 = 0.0, 0.0
		#if t >14.2:
			#x3, y3, x4, y4 = -0.113, -0.113, -0.113, -0.113
		#if t >14.3:
			#x3, y3, x4, y4 = 0.0, -0.16, 0.0, -0.16
		if t <=10:
			x1 = 0
			x2 = 0
			x3 = 0
			x4 = 0
			y1 = -0.16
			y2 = -0.16
			y3 = -0.16
			y4 = -0.16
			leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2  = RobotFuncs.actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
		elif t>10:
			leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2\
			= RobotFuncs.actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
		angles.ang1 = leg1th1
		angles.ang2 = leg2th1
		angles.ang3 = leg3th1
		angles.ang4 = leg4th1
		angles.ang5 = leg1th2
		angles.ang6 = leg2th2
		angles.ang7 = leg3th2
		angles.ang8 = leg4th2
		label_RefAng_Value["text"] = str(round(leg1th1,1)) + ' , ' + str(round(leg2th1,1)) + ' , ' + str(round(leg3th1,1)) + ' , ' + str(round(leg4th1,1)) + ' , '+ str(round(leg1th2,1)) + ' , ' + str(round(leg2th2,1)) + ' , ' + str(round(leg3th2,1)) + ' , ' + str(round(leg4th2,1))
		label_RefStretchAng_Value["text"] = str(Ang9.data)
		label_PawlState_Value["text"] = str(pawls)
		label_time_Value["text"] = str(round(t,1))
		pub1.publish(angles)
		pub2.publish(pawlAng)
		pub3.publish(Ang9)
		rate.sleep()

def TkProssece(*args):
	global root,text,label_ActualAng_Value,label_RefAng_Value,label_RefStretchAng_Value, label_PawlState_Value 
	global label_time_Value,pawls,label_ActualStretchAng_Value,label_Orientation_Value
	root = tk.Tk()
	root.geometry('1300x700')
	label_Orientation = tk.Label(root, text="Orientation: ", width="20", height="2", font=("Monospace",20))
	label_ActualAng = tk.Label(root, text="Actual Leg Ang: ", width="20", height="2", font=("Monospace",20))
	label_RefAng = tk.Label(root, text="Reference Leg Ang: ", width="20", height="2", font=("Monospace",20))
	label_ActualStretchAng = tk.Label(root, text="Actual Spring Ang: ", width="20", height="2", font=("Monospace",20))
	label_RefStretchAng = tk.Label(root, text="Reference Spring Ang: ", width="20", height="2", font=("Monospace",20))
	label_PawlState = tk.Label(root, text="State of Pawl: ", width="20", height="2", font=("Monospace",20))
	label_time = tk.Label(root, text="Time: ", width="20", height="2", font=("Monospace",20))
	label_Orientation.grid(row=1,column=1)
	label_ActualAng.grid(row=2,column=1)
	label_RefAng.grid(row=3,column=1)
	label_ActualStretchAng.grid(row=4,column=1)
	label_RefStretchAng.grid(row=5,column=1)
	label_PawlState.grid(row=6,column=1)
	label_time.grid(row=7,column=1)
	
	label_Orientation_Value = tk.Label(root, text="None", width="30", height="2", font=("Monospace",20))
	label_ActualAng_Value = tk.Label(root, text="None", width="45", height="2", font=("Monospace",20))
	label_RefAng_Value = tk.Label(root, text="None", width="45", height="2", font=("Monospace",20))
	label_ActualStretchAng_Value = tk.Label(root, text="None", width="30", height="2", font=("Monospace",20))
	label_RefStretchAng_Value = tk.Label(root, text="None", width="30", height="2", font=("Monospace",20))
	label_PawlState_Value = tk.Label(root, text="None", width="30", height="2", font=("Monospace",20))
	label_time_Value = tk.Label(root, text="None", width="30", height="2", font=("Monospace",20))
	label_Orientation_Value.grid(row=1,column=2)
	label_ActualAng_Value.grid(row=2,column=2)
	label_RefAng_Value.grid(row=3,column=2)
	label_ActualStretchAng_Value.grid(row=4,column=2)
	label_RefStretchAng_Value.grid(row=5,column=2)
	label_PawlState_Value.grid(row=6,column=2)
	label_time_Value.grid(row=7,column=2)
	resetSpring = tk.Button(root, text="Rest Motor", command=ResetStretchMotor, font=("Monospace",20))
	resetSpring.grid(row=1,column=3)
	
	pawlsControl = tk.Button(root, text="Fix & Release Pawls", command=PawlControl, font=("Monospace",20))
	pawlsControl.grid(row=2,column=3)
	
	JumpOut = tk.Button(root, text="Jump!!!!!!", command=JumpMethod, font=("Monospace",20))
	JumpOut.grid(row=3,column=3)
	#text = tk.Text(root, background='black', foreground='lawn green', font=('Comic Sans MS', 16),height=500,width=1300)
	#text.pack()
	root.bind('<KeyPress>', onKeyPress)
	root.mainloop()

def  ResetStretchMotor():
	global x1,x2,x3,x4,y1,y2,y3,y4
	print "HI"
	
def JumpMethod():
	global pub1,pub2, pawls, pawlAng,iniTime
	global leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2
	global x1,x2,x3,x4,y1,y2,y3,y4 
	print "Jump"
	t_start = time.time() -iniTime
	t = time.time() -iniTime
	pawls =80
	pawlAng.data = pawls
	pub2.publish(pawlAng)
	angles = MotorAngles()
	x3,x4,y3,y4 = -0.113, -0.113, -0.113, -0.113
	leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2\
	= RobotFuncs.actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
	angles.ang1 = leg1th1
	angles.ang2 = leg2th1
	angles.ang3 = leg3th1
	angles.ang4 = leg4th1
	angles.ang5 = leg1th2
	angles.ang6 = leg2th2
	angles.ang7 = leg3th2
	angles.ang8 = leg4th2
	pub1.publish(angles)


def PawlControl():
	global pawls, pawlAng
	if pawls ==70:
		pawls =80
		pawlAng.data = pawls
		print("%d\n"%pawls)
	else:
		pawls = 70
		pawlAng.data = pawls
		print("%d\n"%pawls)

if __name__ == '__main__':
    try:
		thread.start_new_thread(TkProssece,(0,0))
		MainProssece()
		
    except rospy.ROSInterruptException:
        pass



