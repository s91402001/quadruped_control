#!/usr/bin/env python

import Tkinter as tk




def onKeyPress(event):
	text.delete("1.0",tk.END)
	text.insert('end', 'You pressed %s\n' % (event.char, ))
	if event.char =='a':
		print event.char

pub = rospy.Publisher('MotorSetpoint', MotorAngles, queue_size=1024) 
rospy.init_node('reference', anonymous=True)
		
root = tk.Tk()
root.geometry('300x200')
text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 12))
text.insert('end', 'hello')
text.pack()
root.bind('<KeyPress>', onKeyPress)
root.mainloop()
