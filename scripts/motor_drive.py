#!/usr/bin/env python

## Description: Controlling Left/Right and Forward/Backward Movement
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}

# remove or add the library/libraries for ROS
import rospy
import time
from gpiozero import Motor
 
# remove or add the message type
from geometry_msgs.msg import Twist

# initiate Motor instance
fb_motor = Motor(forward=4, backward=14)
lr_motor = Motor(forward=17, backward=18)

#define function/functions to provide the required functionality
def speed_callback(msg):
	# for simplicity
	fb_power = msg.linear.x
	lr_power = msg.angular.z

#	Clip the PWM DC (0 ~ 1)
	fb_pwm = (abs(fb_power) - 0) * (1 - 0) / (0.22 - 0) + 0
	lr_pwm = (abs(lr_power) - 0) * (1 - 0) / (2.84 - 0) + 0

	# forward/backward navigation
	fb_navi(fb_power, fb_pwm)

	# left/right navigation
	lr_navi(lr_power, lr_pwm)

# def clip_pwm(power, min_pwm=0, max_pwm=1):
	# if power < min_pwm:
	# 	return min_pwm
	# elif power > max_pwm:
	# 	return max_pwm
	# return power

def fb_navi(fb_power, fb_pwm):
	# fish forward
	if fb_power > 0:
		fb_motor.forward(speed=fb_pwm)
	# fish backward	
	elif fb_power < 0:
		fb_motor.backward(speed=fb_pwm)
	# fish stop
	else:
		fb_motor.stop()

def lr_navi(lr_power, lr_pwm):
	# fish left
	if lr_power > 0:
		lr_motor.forward(speed=lr_pwm)
	# fish right	
	elif lr_power < 0:
		lr_motor.backward(speed=lr_pwm)
	# fish stop
	else:
		lr_motor.stop()

if __name__=='__main__':
	#Add here the name of the ROS. In ROS, names are unique named.
	rospy.init_node('fish_vel')

	#subscribe to a topic using rospy.Subscriber class
	sub=rospy.Subscriber('/cmd_vel', Twist, speed_callback)
	rospy.spin()
