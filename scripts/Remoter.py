#!/usr/bin/env python
import Tkinter as tk
from std_msgs.msg import Float32
import rospy


def onKeyPress(event):
	text.delete("1.0",tk.END)
	text.insert('end', 'You pressed %s\n' % (event.char, ))
		
	if event.char =='q': #Left H
		pub1.publish(0.005)
		print ''
	elif event.char =='a': 
		pub1.publish(-0.005)
		print ''
	elif event.char =='w': #Right H
		pub2.publish(0.005)
		print ''
	elif event.char =='s':
		pub2.publish(-0.005)
		print ''	
	elif event.char =='e': #Left L
		pub3.publish(0.005)
		print ''
	elif event.char =='d':
		pub3.publish(-0.005)
		print ''
	elif event.char =='r': #Right L
		pub4.publish(0.005)
		print ''
	elif event.char =='f':
		pub4.publish(-0.005)
		print ''
	elif event.char =='t': #Ta
		pub5.publish(0.1)
		print ''
	elif event.char =='g':
		pub5.publish(-0.1)
		print ''
	elif event.char =='y': #Td
		pub6.publish(0.1)
		print ''
	elif event.char =='h':
		pub6.publish(-0.1)
		print ''
	elif event.char =='u': #Ptd X
		pub7.publish(0.005)
		print ''
	elif event.char =='j':
		pub7.publish(-0.005)
		print ''	
	elif event.char =='i': #Ptd Y
		pub8.publish(0.005)
		print ''
	elif event.char =='k': 
		pub8.publish(-0.005)
		print ''
	elif event.char =='o': #Ptd1 X
		pub9.publish(0.005)
		print ''
	elif event.char =='l': 
		pub9.publish(-0.005)
		print ''
	elif event.char =='p': #Ptd1 Y
		pub10.publish(0.005)
		print ''
	elif event.char ==';':
		pub10.publish(-0.005)
		print ''
	elif event.char =='z':
		pub11.publish(0)
	elif event.char ==',':
		pub3.publish(-0.005)
		pub4.publish(0.005)
	elif event.char =='.':
		pub3.publish(0.005)
		pub4.publish(-0.005)	
														
pub1 = rospy.Publisher('LeftH', Float32, queue_size=1024) 
pub2 = rospy.Publisher('RightH', Float32, queue_size=1024) 
pub3 = rospy.Publisher('LeftL', Float32, queue_size=1024) 
pub4= rospy.Publisher('RightL', Float32, queue_size=1024) 
pub5 = rospy.Publisher('Ta', Float32, queue_size=1024) 
pub6 = rospy.Publisher('Td', Float32, queue_size=1024) 
pub7 = rospy.Publisher('ptdx', Float32, queue_size=1024) 
pub8 = rospy.Publisher('ptdy', Float32, queue_size=1024) 
pub9 = rospy.Publisher('ptd1x', Float32, queue_size=1024) 
pub10 = rospy.Publisher('ptd1y', Float32, queue_size=1024) 
pub11 = rospy.Publisher('isStop', Float32, queue_size=1024) 
rospy.init_node('Adjust')
		
root = tk.Tk()
root.geometry('300x200')
text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 12))
text.pack()
root.bind('<KeyPress>', onKeyPress)
root.mainloop()
