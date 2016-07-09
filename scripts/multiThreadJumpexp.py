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
pawls = 50
pawlAng = Float32()
Ang9 = Float32()

def onKeyPress(event):
	global pawlAng,pawls
	global Ang9,pub2
	if event.char =='q':
		if pawls ==90:
			pawls =65
			pawlAng.data = pawls
			print("%d\n"%pawls)
		else:
			pawls = 90
			pawlAng.data = pawls
			print("%d\n"%pawls)
		
	elif event.char =='a': 
		Ang9.data = Ang9.data + 100
	elif event.char =='s': 
		Ang9.data = Ang9.data - 100




def OrientationCallback(data):
	global Orientation
	Orientation = data.x
	#print data.x
	#print Orientation
			
def MainProssece(*args):
	global leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2
	global iniTime,LeftH,RightH,LeftL,RightL,Ta,Td
	global ptd,ptd1
	global deltaLmax,Orientation,desiredOrientation,Ang9,pawlAng,pub2
	pub1 = rospy.Publisher('MotorSetpoint', MotorAngles, queue_size=1024) 
	pub2 = rospy.Publisher('Pawl',Float32 , queue_size=1024) 
	pub3 = rospy.Publisher('Motor9',Float32 , queue_size=1024)
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
		elif t >5 :
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
		
		outFile = open('./ExpData/WalkExpData20160205-3.txt','a')
		outFile.write(str(t)+'\t'+str(deltaLmax)+'\t'+str(Orientation)+'\t'+str(desiredOrientation)+'\r\n')
		text.delete("1.0",tk.END)
		text.insert('end',"Orientation = %.3f\n"%Orientation)
		text.insert('end',"Desired Orientation = %.3f\n\n"%desiredOrientation)
		text.insert('end',"Orientation Error = %.3f\n\n"%(Orientation - desiredOrientation))
		text.insert('end',"deltaLmax = %.3f\n\n"%deltaLmax)
		text.insert('end', "t = %.3f\n\nLeft H\tRight H\tLeft L\tRight L\tTa\tTd\tptd\t\tptd1\t\tisStop\n\
%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t[%.3f, %.3f]\t\t[%.3f, %.3f]\t\t%s"\
%(t,LeftH,RightH,LeftL,RightL,Ta,Td,ptd[0],ptd[1],ptd1[0],ptd1[1],isStop))	
		text.insert('end',"\n\nMotor Reference\n %.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f\n"%(leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2))
		#print("%.3f\t%.3f\n"%(leg1th1,leg1th2))
		angles.ang1 = leg1th1
		angles.ang2 = leg2th1
		angles.ang3 = leg3th1
		angles.ang4 = leg4th1
		angles.ang5 = leg1th2
		angles.ang6 = leg2th2
		angles.ang7 = leg3th2
		angles.ang8 = leg4th2
		pub1.publish(angles)
		pub2.publish(pawlAng)
		pub3.publish(Ang9)
		rate.sleep()

def TkProssece(*args):
	global root,text
	root = tk.Tk()
	root.geometry('1300x500')
	text = tk.Text(root, background='black', foreground='lawn green', font=('Comic Sans MS', 16),height=500,width=1300)
	text.pack()
	root.bind('<KeyPress>', onKeyPress)
	root.mainloop()
	
if __name__ == '__main__':
    try:
		thread.start_new_thread(TkProssece,(0,0))
		MainProssece()
		
    except rospy.ROSInterruptException:
        pass



