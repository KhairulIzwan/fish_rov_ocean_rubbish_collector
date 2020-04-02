#!/usr/bin/env python
from __future__ import division
import rospy
import os
from Tkinter import *
#from mmmros.msg import Movement, SensorData


class ControlApp(Tk, object):

    def __init__(self):
	rospy.init_node('tank_teleop')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

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

	mainloop()

    
if __name__ == '__main__':
    os.system('xset r off')
    control = ControlApp()
    os.system('xset r on')
