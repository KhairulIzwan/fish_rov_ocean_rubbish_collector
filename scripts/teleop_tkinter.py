#!/usr/bin/env python

#Title: Python Publisher for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [28-01-2020]
#Description: Tank Navigation Publisher Nodes (Python) ** originally from turtlebot3_teleop

#remove or add the library/libraries for ROS
from __future__ import division
import rospy
import os
from Tkinter import *

#remove or add the message type
from geometry_msgs.msg import Twist

e = """
Communications Failed
"""

class ControlApp():

	def __init__(self):
		rospy.init_node('fish_teleop')

		rospy.on_shutdown(self.shutdown)

		self.cmdVel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.master = Tk()

		self.master.title('The Controller')
		self.master.geometry("70x100")
		self.master.resizable(0, 0)

		self.leftValue = IntVar()  # IntVars to hold
		self.rightValue = IntVar() # values of scales

		self.leftScale = Scale(self.master, from_=-255, to=255, 
					variable=self.leftValue, showvalue=0)
		self.leftScale.set(0)

		self.rightScale = Scale(self.master, from_=-255, to=255, 
					variable=self.rightValue, showvalue=0)
		self.rightScale.set(0)

		self.leftLabel = Label(self.master, textvariable=self.leftValue,
					 wraplength=1)
		self.rightLabel = Label(self.master, textvariable=self.rightValue, 
					wraplength=1)

		self.leftLabel.grid(row=0, column=0)
		self.leftScale.grid(row=0, column=1)
		self.rightLabel.grid(row=0, column=3)
		self.rightScale.grid(row=0, column=2)

		self.master.mainloop()

		self.pub_cmdVel()

	def pub_cmdVel(self):
		try:
			while(1):
				self.leftScale = Scale(self.master, from_=-255, 
					to=255, variable=self.leftValue, showvalue=0)
				self.leftScale.set(0)

				self.rightScale = Scale(self.master, from_=-255, 
					to=255, variable=self.rightValue, showvalue=0)
				self.rightScale.set(0)

				self.leftLabel = Label(self.master, 
					textvariable=self.leftValue, wraplength=1)
				self.rightLabel = Label(self.master, textvariable=self.rightValue, 
							wraplength=1)

				self.leftLabel.grid(row=0, column=0)
				self.leftScale.grid(row=0, column=1)
				self.rightLabel.grid(row=0, column=3)
				self.rightScale.grid(row=0, column=2)

				print(self.leftValue.get())

				self.twist = Twist()
				self.twist.linear.x = self.leftValue.get()
				self.twist.linear.y = 0.0
				self.twist.linear.z = 0.0

				self.twist.angular.x = 0.0
				self.twist.angular.y = 0.0
				self.twist.angular.z = self.rightValue.get()

				self.cmdVel_pub.publish(self.twist)

		except:
			print(e)

		finally:
			self.twist = Twist()

			self.twist.linear.x = 0.0
			self.twist.linear.y = 0.0
			self.twist.linear.z = 0.0

			self.twist.angular.x = 0.0
			self.twist.angular.y = 0.0
			self.twist.angular.z = 0.0

			self.cmdVel_pub.publish(self.twist)

	def shutdown(self):
		rospy.loginfo("preview_node [OFFLINE]...")

def main(args):
	vn = ControlApp()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("preview_node [OFFLINE]...")

if __name__ == '__main__':
	rospy.loginfo("preview_node [ONLINE]...")
	main(sys.argv)
