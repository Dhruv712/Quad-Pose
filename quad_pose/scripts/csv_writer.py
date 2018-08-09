#!/usr/bin/env python
import rospy
import csv
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

class csvWriter():

	def __init__(self):

		rospy.init_node('csv_writer', anonymous=True)

		rospy.Subscriber('pixels',
						 Float32MultiArray,
						 self.pixelCB)

  		rospy.Subscriber('quad_pose',
  						Float32MultiArray,
  						self.poseCB)

  		rospy.Subscriber('boxData',
  						Float32MultiArray,
  						self.boxCB)

		self.xPixelData = Float32MultiArray()
		self.yPixelData = Float32MultiArray()

		self.xPoseData = Float32MultiArray()
		self.yPoseData = Float32MultiArray()
		self.zPoseData = Float32MultiArray()

		self.boxSize = Float32MultiArray()

		self.xEditedData = Float32MultiArray()
		self.yEditedData = Float32MultiArray()
		self.zEditedData = Float32MultiArray()

		self.editedBoxSize = Float32MultiArray()

	def controller(self):
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			rate.sleep()

	def pixelCB(self, msg):
		self.xPixelData.data.append(msg.data[0])
		self.yPixelData.data.append(msg.data[1])

		self.xEditedData.data.append(self.xPoseData.data[-1])
		self.yEditedData.data.append(self.yPoseData.data[-1])
		self.zEditedData.data.append(self.zPoseData.data[-1])

		self.editedBoxSize.data.append(self.boxSize.data[-1])

	def poseCB(self, msg):
		self.xPoseData.data.append(msg.data[0])
		self.yPoseData.data.append(msg.data[1])
		self.zPoseData.data.append(msg.data[2])

	def boxCB(self, msg):
		w = msg.data[2] - msg.data[0]
		h = msg.data[3] - msg.data[1]

		self.boxSize.data.append(math.sqrt(w**2 + h**2))

	def writeToFile(self):
		##### FILENAME
		with open('/home/msl/catkin_ws/src/quad_pose/pixel_pose.csv', mode='w') as csv_file:
			fields = ['xPixel', 'yPixel', 'boxSize', 'xPose', 'yPose', 'zPose']
			writer = csv.DictWriter(csv_file, fieldnames=fields)
			writer.writeheader()

			rospy.loginfo(len(self.xPixelData.data))
			rospy.loginfo(len(self.xEditedData.data))

			for i in range(len(self.xPixelData.data)):
				writer.writerow({'xPixel':self.xPixelData.data[i],
								 'yPixel':self.yPixelData.data[i],
								'boxSize':self.editedBoxSize.data[i],
								  'xPose':self.xEditedData.data[i],
								  'yPose':self.yEditedData.data[i],
								  'zPose':self.zEditedData.data[i]})

if __name__ == '__main__':
	writer = csvWriter()
	while not rospy.is_shutdown():
		try:
			writer.controller()
		except rospy.ROSException:
			pass
	writer.writeToFile()