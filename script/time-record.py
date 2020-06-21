#!/usr/bin/env python

################################################################################
## {Description}: Recording the Video Stream
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np

from pyzbar import pyzbar
import datetime
import time

class BarcodeVideo:
	def __init__(self):

		self.bridge = CvBridge()
		self.scanCode = String()
		self.image_received = False
		self.code_received = False

		# The duration in seconds of the video captured
		self.capture_duration = 5

		self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

		# Subscribe Image msg
		img_topic = "/cv_camera/image_raw"
		self.image_sub = rospy.Subscriber(img_topic, Image, self.cbImage)

		# Subscribe CameraInfo msg
		imgInfo_topic = "/cv_camera/camera_info"
		self.imageInfo_sub = rospy.Subscriber(imgInfo_topic, CameraInfo, self.cbImageInfo)

		# Subscribe String msg
		code_topic = "/scanned_barcode"
		self.code_sub = rospy.Subscriber(code_topic, String, self.cbCode)

		# Allow up to one second to connection
		rospy.sleep(1)

	def cbImage(self, msg):

		# Convert image to OpenCV format
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.image_received = True
		self.image = cv_image

	def cbImageInfo(self, msg):

		self.image_width = msg.width
		self.image_height = msg.height

	def cbCode(self, msg):

		try:
			qrcode = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.code_received = True
		self.qr = qrcode

	def preview(self):

		# Overlay some text onto the image display
		timestr = time.strftime("%Y%m%d-%H%M%S")
		cv2.putText(self.image, timestr, (10, 20), 1, 1, 
			(255, 255, 255), 1, cv2.LINE_AA, False)

	def record(self):

		if self.image_received and self.code_received:
			timestr = time.strftime("%Y%m%d-%H%M%S")
			filename = timestr + "-output.avi"
			out = cv2.VideoWriter(filename,self.fourcc, 20.0, (self.image_width,self.image_height))
			start_time = time.time()
			while(int(time.time() - start_time) < self.capture_duration):
				self.preview()
				out.write(self.image)
		else:
			rospy.logerr("No images recieved")

if __name__ == '__main__':

	rospy.loginfo("Barcode Video node [ONLINE]...")

	# Initialize
	rospy.init_node("barcode_video", anonymous=False)
	video = BarcodeVideo()

	# Camera preview
	while not rospy.is_shutdown():
		video.record()
