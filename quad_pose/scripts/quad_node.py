#!/usr/bin/env python
'''
This node is a subscriber to the VRPN client node, broadcasts to TF, and publushes to RB1/pose. 
The launch file for this node opens the node itself along with rviz, which helps vizualize the 
optitrack data. The launch file also contains a static link transform. 
Different users can create their own config.yaml files for different camera parameters, etc. If 
you create your own config file, remember to change the name in the launch file too.
'''

import rospy
import numpy as np
#import math
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import PoseStamped

class vrpnHandler():

	def __init__(self):

		rospy.init_node('quad_node', anonymous=True)

		self.p = PoseStamped() # To publish

		self.br = tf2_ros.TransformBroadcaster()
		self.world_n = tf2_ros.TransformStamped() # new frame specified by config.yaml
		# self.quadT = tf2_ros.TransformStamped() # position relative to world_n frame

		'''
		SETTING 'world_n'
		'''
		self.world_n.header.frame_id = 'world'
		self.world_n.child_frame_id = 'world_new'

		self.world_n.transform.translation.x = rospy.get_param('~x')
		self.world_n.transform.translation.y = rospy.get_param('~y')
		self.world_n.transform.translation.z = rospy.get_param('~z')

		self.transVector = np.array([self.world_n.transform.translation.x,
									 self.world_n.transform.translation.y,
									 self.world_n.transform.translation.z])

		self.theta = rospy.get_param('~roll')
		self.phi = rospy.get_param('~pitch')
		self.psi = rospy.get_param('~yaw')

		self.q = tf.transformations.quaternion_from_euler(self.theta, 
														self.phi, 
														self.psi)

		self.world_n.transform.rotation.x = self.q[0]
		self.world_n.transform.rotation.y = self.q[1]
		self.world_n.transform.rotation.z = self.q[2]
		self.world_n.transform.rotation.w = self.q[3]

		'''
		SETTING PUBLISHER AND SUBSCRIBER + LISTENER FOR TF
		'''
		self.p.header.frame_id = 'world_new'

		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
	
		self.pubRb1 = rospy.Publisher('RB1/pose', PoseStamped, queue_size=10)

		rospy.Subscriber('vrpn_client_node/RigidBody01/pose',
                     PoseStamped,
                     self.handle_pose_CB)
	
	def controller(self):
		
		rate = rospy.Rate(10)
		
		while not rospy.is_shutdown():

			if not self.p:
				rate.sleep()
				continue
			
			self.br.sendTransform(self.world_n)

			self.pubRb1.publish(self.p)
			rospy.loginfo(self.p)
			rate.sleep()

	def handle_pose_CB(self, msg):
		
		self.world_n.header.stamp = rospy.Time.now()
		
		# self.quadT.header.stamp = rospy.Time.now()
		# self.quadT.header.frame_id = 'world_new'
		# self.quadT.child_frame_id = 'quad'

		posV = np.array([msg.pose.position.x,
						 msg.pose.position.y,
						 msg.pose.position.z])

		quat =  np.array([msg.pose.orientation.x,
						msg.pose.orientation.y,
						msg.pose.orientation.z,
						msg.pose.orientation.w])

		posF, rotF = self.applyTransform(posV, quat)

		self.p.pose.position.x = posF[0]
		self.p.pose.position.y = posF[1]
		self.p.pose.position.z = posF[2]

		self.p.pose.orientation.x = rotF[0]
		self.p.pose.orientation.y = rotF[1]
		self.p.pose.orientation.z = rotF[2]
		self.p.pose.orientation.w = rotF[3]

		# self.quadT.transform.translation.x = posF[0]
		# self.quadT.transform.translation.y = posF[1]
		# self.quadT.transform.translation.z = posF[2]

		# self.quadT.transform.rotation.x = rotF[0]
		# self.quadT.transform.rotation.y = rotF[1]
		# self.quadT.transform.rotation.z = rotF[2]
		# self.quadT.transform.rotation.w = rotF[3]

		# self.br.sendTransform(self.quadT)

	def applyTransform(self, posVector, quat):
		
		posT = self.qv_multi(self.q, posVector) #+ self.transVector
			# Multiplies the Euler angles (that get converted to quaternions) from the .yaml file 
			# by the position vector of the input data. Then it adds the xyz translation from the 
			# .yaml file
		rotT = tf.transformations.quaternion_multiply(self.q, quat)
			# Multiplies the quaternion from the .yaml file by the quaternion of the input data 

		return(posT, rotT)

	'''
	MULTIPLIES QUATERNION BY VECTOR
	'''
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

	'''
	ROTATION MATRICES
	'''
	# def eulerToRot(localTheta, localPhi, localPsi):

	# 	rtheta = np.array([1, 0, 0], 
	# 		[0, math.cos(localTheta), math.sin(localTheta)],
	# 		[0, -1*math.sin(localTheta), math.cos(localTheta)])
		
	# 	rphi = np.array([math.cos(localPhi), 0, -1*math.sin(localPhi)],
	# 		[0, 1, 0],
	# 		[math.sin(localPhi), 0, math.cos(localPhi)])

	# 	rpsi = np.array([math.cos(localPsi), math.sin(localPsi), 0],
	# 		[-1*math.sin(localPsi), math.cos(localPsi), 0],
	# 		[0, 0, 1])

	# 	p1 = rtheta*rphi
	# 	final = p1*rpsi

	# 	return final

if __name__ == '__main__':	
	
	vrpn = vrpnHandler()
	vrpn.controller()