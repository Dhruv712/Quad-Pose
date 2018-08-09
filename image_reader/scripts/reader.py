#!/usr/bin/env python
import rospy
import numpy as np
import cv2

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class imageReader():

	def __init__(self):

		self.bridge = CvBridge()

		self.image = Image()
		
		# --only do this if you have a custom image to load as the background image--
		# self.background = cv2.imread('/home/msl/catkin_ws/src/image_reader/scripts/background.jpg', 1)
		
		self.background = Image()
		self.result = Image()
		self.box = Image()

		#self.raw = Image()

		self.msg = Image()

		self.boxData = Float32MultiArray()
		self.pixelsData = Float32MultiArray()

		self.counter = 1

		rospy.init_node('reader', anonymous=True)
		rospy.Subscriber('pixels', Float32MultiArray, self.handle_pixel_CB)
		rospy.Subscriber('camera/image_raw', Image, self.handle_img_CB)

		self.publishResult = rospy.Publisher('result', Image, queue_size=10)
		self.publishBoxViz = rospy.Publisher('box', Image, queue_size=10)
		self.publishBoxData = rospy.Publisher('boxData', Float32MultiArray, queue_size=10)

	def handle_img_CB(self, data):

		self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		
		#Sets background image to be the initial image picked up by the camera
		if self.counter == 1:
			self.background = self.image
			self.counter = 0
			# --uncomment the following code if you want to save the first picture that gets taken to a file
			# cv2.imwrite("/home/msl/catkin_ws/src/image_reader/scripts/background-pic.jpg", 
			# 	self.background)

		self.result = cv2.subtract(self.background, self.image)

		self.msg = self.bridge.cv2_to_imgmsg(self.result, "bgr8")

		#Isolate pixels that fall within the color range and place them in an array
		COLOR_MIN = np.array([50, 50, 50], np.uint8)
		COLOR_MAX = np.array([255, 255, 255], np.uint8)

		array = cv2.inRange(self.result, COLOR_MIN, COLOR_MAX)

		#Find contours in image
		ret, thresh = cv2.threshold(array, 40, 150, 0)
		edges = cv2.Canny(self.result, 20, 255)
		_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		#Find the index of the largest contour
		areas = [cv2.contourArea(c) for c in contours]
		rospy.loginfo(areas)
		if len(areas) > 0:
			max_index = np.argmax(areas)
			cnt = contours[max_index]

			#Draw green rectangle
			x,y,w,h = cv2.boundingRect(cnt)
			cv2.rectangle(self.result, (x,y), (x+w,y+h), (0,255,0),2)

			self.box = self.result[y:y+h, x:x+w]

			self.msg = self.bridge.cv2_to_imgmsg(self.box, "bgr8")
			self.publishBoxViz.publish(self.msg)

			self.boxData.data = [x, y, x+w, y+h]
			self.publishBoxData.publish(self.boxData)

			#Isolate midpoint with white dot
			midpointX = x + (w / 2)
			midpointY = y + (h / 2)
			cv2.rectangle(self.result, 
				(midpointX - 2, midpointY - 2), 
				(midpointX + 2, midpointY + 2), 
				(255, 255, 255), 2)

			self.pixelsData.data = [midpointX, midpointY, 1.0]

		self.publishResult.publish(self.msg)

		#--uncomment all lines to show image and edge detection (in separate windows) and publish data

		# cv2.imshow("Edges", edges)
		# cv2.imshow("Result", self.result)
		# cv2.waitKey(1)

	def handle_pixel_CB(self, msg):
		cv2.circle(self.image, (int(msg.data[0]), int(msg.data[1])), 5, (255, 255, 255), -1)

		cv2.imshow("Raw", self.image)
		cv2.waitKey(1)


if __name__ == '__main__':

	reader = imageReader()
	rospy.spin()