#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np

import cv2

class preview_node:
	def __init__(self):

		# Initializing your ROS Node
		rospy.init_node('preview_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Give the OpenCV display window a name
		self.cv_window_name = "Camera Stream"

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to the raw camera image topic
		self.imgRaw_sub = rospy.Subscriber("/raspicam_node_robot/image/compressed", CompressedImage, self.callback, queue_size=1)

	def callback(self,data):
		# Convert the raw image to OpenCV format
		self.cvtImage(data)

		# Get the width and height of the image
		self.getCameraInfo()

		# Overlay some text onto the image display
		self.textInfo()

		# Refresh the image on the screen
		self.displayImg()

	# Get the width and height of the image
	def getCameraInfo(self):
		self.image_width = rospy.get_param("/raspicam_node_robot/width") 
		self.image_height = rospy.get_param("/raspicam_node_robot/height") 
		rospy.set_param("/raspicam_node_robot/brightness", 100)
	# Convert the raw image to OpenCV format
	def cvtImage(self, data):
		try:
			# Convert the raw image to OpenCV format """
			# self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# direct conversion to CV2 ####
			self.cv_image = np.fromstring(data.data, np.uint8)
			self.cv_image = cv2.imdecode(self.cv_image, cv2.IMREAD_COLOR)

			# OTIONAL -- image-rotate """
			self.cv_image = imutils.rotate(self.cv_image, angle=180)
			self.cv_image_copy = self.cv_image.copy()

		except CvBridgeError as e:
			print(e)

	# Overlay some text onto the image display
	def textInfo(self):
		img = self.cv_image

		text = "Online"
		org = (10, self.image_height - 10)

		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.1
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		text1 = "(%d, %d)" % (self.image_width, self.image_height)
		org1 = (self.image_width - 100, self.image_height - 10)

		cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin)
		cv2.putText(img, text1, org1, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin)

	# Refresh the image on the screen
	def displayImg(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.waitKey(1)

	def shutdown(self):
		try:
			rospy.loginfo("preview_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

def usage():
    print("%s" % sys.argv[0])

def main(args):
	vn = preview_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("preview_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("preview_node [ONLINE]...")
	main(sys.argv)
