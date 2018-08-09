#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

class cameraTransform():

	def __init__(self):

		rospy.init_node('camera_transform', anonymous=True)

		self.pubPixels = rospy.Publisher('pixels', Float32MultiArray, queue_size=10)

		rospy.Subscriber('vrpn_client_node/RigidBody01/pose',
                     PoseStamped,
                     self.handle_rb01_CB)

		rospy.Subscriber('vrpn_client_node/RigidBody02/pose',
					 PoseStamped,
					 self.handle_rb02_CB)

		self.rb01 = PoseStamped()
		self.rb02 = PoseStamped()

		self.pixelsPosQuad = Float32MultiArray()

		self.rb02PosCam = np.array([0.03, 0.0, 0.0])
		self.rb02RotCam = tf.transformations.quaternion_from_euler(np.pi, 
														-(np.pi)/2, 
														0.0)

		self.rb01PosQuad = np.array([0.0, -0.03, 0.0])
		self.rb01RotQuad = np.array([0.0, 0.0, 0.0, 1.0])

		self.cameraMatrix = rospy.get_param('~camera_matrix/data')

		self.posePublisher = rospy.Publisher('quad_pose', Float32MultiArray, queue_size=10)

		self.restrictPixel = -1.0

	def controller(self):

		rate = rospy.Rate(120)

		while not rospy.is_shutdown():

			if self.restrictPixel > 0.0:
				if self.pixelsPosQuad.data[0] > 0 and self.pixelsPosQuad.data[0] < 640:
					if self.pixelsPosQuad.data[1] > 0 and self.pixelsPosQuad.data[1] < 480:
						self.pubPixels.publish(self.pixelsPosQuad)

			rate.sleep()

	def handle_rb01_CB(self, data):

		### MEASUREMENTS ###

		self.rb01.pose.position.x = data.pose.position.x
		self.rb01.pose.position.y = data.pose.position.y
		self.rb01.pose.position.z = data.pose.position.z

		self.rb01.pose.orientation.x = data.pose.orientation.x
		self.rb01.pose.orientation.y = data.pose.orientation.y
		self.rb01.pose.orientation.z = data.pose.orientation.z
		self.rb01.pose.orientation.w = data.pose.orientation.w

		worldPosrb01 = np.array([self.rb01.pose.position.x,
						self.rb01.pose.position.y,
						self.rb01.pose.position.z])

		worldRotrb01 = np.array([self.rb01.pose.orientation.x,
						self.rb01.pose.orientation.y,
						self.rb01.pose.orientation.z,
						self.rb01.pose.orientation.w])

		worldPosrb02 = np.array([self.rb02.pose.position.x,
						self.rb02.pose.position.y,
						self.rb02.pose.position.z])		

		worldRotrb02 = np.array([self.rb02.pose.orientation.x,
						self.rb02.pose.orientation.y,
						self.rb02.pose.orientation.z,
						self.rb02.pose.orientation.w])

		### STEP 1 ###

		camRotWorld = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(self.rb02RotCam),
															 tf.transformations.quaternion_inverse(worldRotrb02))

		### STEP 2 ###

		worldPosQuad = self.qv_multi(worldRotrb01, self.rb01PosQuad) + worldPosrb01

		### STEP 3 ###

		worldPosCam = self.qv_multi(worldRotrb02, self.rb02PosCam) + worldPosrb02

		### STEP 4 ###

		camPosQuad = self.qv_multi(camRotWorld, (worldPosQuad - worldPosCam))

		### STEP 5 ###

		self.restrictPixel = camPosQuad[2]

		self.pixelsPosQuad.data = np.matmul(np.reshape(self.cameraMatrix, (3,3)),
								  			camPosQuad)

		self.pixelsPosQuad.data = np.divide(self.pixelsPosQuad.data, camPosQuad[2])

		### PUBLISH ###

		toPublish = Float32MultiArray()
		toPublish.data = camPosQuad
		self.posePublisher.publish(toPublish)

	def handle_rb02_CB(self, data):

		self.rb02.pose.position.x = data.pose.position.x
		self.rb02.pose.position.y = data.pose.position.y
		self.rb02.pose.position.z = data.pose.position.z

		self.rb02.pose.orientation.x = data.pose.orientation.x
		self.rb02.pose.orientation.y = data.pose.orientation.y
		self.rb02.pose.orientation.z = data.pose.orientation.z
		self.rb02.pose.orientation.w = data.pose.orientation.w

	def qv_multi(self, q1, v1):
		norm = np.linalg.norm(v1)
		v1 = tf.transformations.unit_vector(v1)
		q2 = list(v1)
		q2.append(0.0)
		vector = tf.transformations.quaternion_multiply(
			tf.transformations.quaternion_multiply(q1, q2),
			tf.transformations.quaternion_conjugate(q1)
		)[:3]
		print(type(vector))
		return (vector*norm)

if __name__ == '__main__':
	cameraTransform = cameraTransform()
	cameraTransform.controller()