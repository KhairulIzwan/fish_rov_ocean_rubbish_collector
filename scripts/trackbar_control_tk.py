#!/usr/bin/env python

#Title: Python Subscriber for Tank Color Calibration (Selection for HSV Range)
#Author: Khairul Izwan Bin Kamsani - [12-02-2020]
#Description: Tank Color Range Subcriber Nodes (Python)

from __future__ import print_function
from __future__ import division

import sys
import rospy
import os
import cv2
import imutils

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from Tkinter import *

e = """
Communications Failed
"""

if __name__=="__main__":
	rospy.init_node('tank_teleop')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	try:
		while(1):
			master = Tk()

			master.title('The Controller')
			#You can set the geometry attribute to change the root windows size
			master.geometry("70x100") #You want the size of the app to be 500x500
			master.resizable(0, 0) #Don't allow resizing in the x or y direction


			leftValue = IntVar()  # IntVars to hold
			rightValue = IntVar() # values of scales

			leftScale = Scale(master, from_=-255, to=255, variable=leftValue, showvalue=0)
			leftScale.set(0)

			rightScale = Scale(master, from_=-255, to=255, variable=rightValue, showvalue=0)
			rightScale.set(0)

			leftLabel = Label(master, textvariable=leftValue, wraplength=1)   # labels that will update
			rightLabel = Label(master, textvariable=rightValue, wraplength=1) # with IntVars as slider moves

			leftLabel.grid(row=0, column=0)
			leftScale.grid(row=0, column=1)
			rightLabel.grid(row=0, column=3)
			rightScale.grid(row=0, column=2)

			twist = Twist()
			twist.linear.x = leftValue
			twist.linear.y = 0.0
			twist.linear.z = 0.0

			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = rightValue
			pub.publish(twist)

			mainloop()

	except:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub.publish(twist)

		master.quit()

