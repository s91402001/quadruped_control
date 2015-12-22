#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point


def talker():
	pub1 = rospy.Publisher('Gains', Point, queue_size=1024) 
	rospy.init_node('SendGain', anonymous=True)
	rate = rospy.Rate(50) # 10hz
	gains = Point()
	while not rospy.is_shutdown():
		gain = input('Kp,Ki,Kd = ')
		gains.x = gain[0]
		gains.y = gain[1]
		gains.z = gain[2]
		pub1.publish(gains)
		rate.sleep()

if __name__ == '__main__':
    try:
		talker()
    except rospy.ROSInterruptException:
        pass
