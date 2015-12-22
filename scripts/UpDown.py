#!/usr/bin/env python

import rospy
from math import cos
import time
import RobotFuncs
from std_msgs.msg import Float32
from quadruped_control.msg import MotorAngles

iniTime =0
leg1th1 = 0
leg1th2 = 0
leg2th1 = 0
leg2th2 = 0
leg3th1 = 0
leg3th2 = 0
leg4th1 = 0


def talker():
	global leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2,iniTime
	pub1 = rospy.Publisher('MotorSetpoint', MotorAngles, queue_size=1024) 
	rospy.init_node('reference', anonymous=True)
	angles = MotorAngles()
	rate = rospy.Rate(50) # 10hz
	while not rospy.is_shutdown():
		if iniTime ==0:
			iniTime = time.time()
		t = time.time() -iniTime
		if t <= 5:
			leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = 90,0,-90,0,90,0,-90,0
			rospy.loginfo("t = %s"%t)
		elif t >5 and t <15:
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
		else:
			leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = RobotFuncs.generateUpDownCommand(t-15) 
		rospy.loginfo("Reference : %s,\t%s,\t%s,\t%s,\t%s,\t%s,\t%s,\t%s"%(leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2))
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
		#listener()
		talker()
    except rospy.ROSInterruptException:
        pass
