import numpy as np
from utils import *
class Leg:
	def __init__(self, thigh_segment, shin_segment, hip_joint, knee_joint, leg_name):
		self.thigh_segment = thigh_segment
		self.shin_segment = shin_segment
		self.hip_joint = hip_joint
		self.knee_joint = knee_joint

		self.leg_name = leg_name

		if self.leg_name == "hind":
			self.leg_index = 0

		else:
			self.leg_index = 2

		self.m1 = self.thigh_segment.body.mass
		self.m2 = self.shin_segment.body.mass

		self.l1 = self.thigh_segment.height
		self.l2 = self.shin_segment.height


	def GetMassMatrix(self, state):
		theta1 = state.joint_theta[self.leg_index + 0]
		theta2 = state.joint_theta[self.leg_index + 1]

		# theta1 = self.joint_1.GetAngle()
		# theta2 = self.joint_2.GetAngle()

		r11 = self.m2*self.l2**2 + 2*self.l1*self.l2*self.m2*np.cos(theta2) + (self.l1**2)*(self.m2 + self.m1)
		r12 = self.m2*self.l2**2 +   self.l1*self.l2*self.m2*np.cos(theta2)
		r21 = r12
		r22 = self.m2*self.l2**2
		M = np.array([
						[r11, r12],
						[r21, r22]
					 ])

		return M

	def GetCoriolisMatrix(self, state):
		theta1 = state.joint_theta[self.leg_index + 0]
		theta2 = state.joint_theta[self.leg_index + 1]

		theta1_dot = state.joint_theta_dot[self.leg_index + 0]
		theta2_dot = state.joint_theta_dot[self.leg_index + 1]

		# theta1 = self.joint_1.GetAngle()
		# theta2 = self.joint_2.GetAngle()

		# theta1_dot = self.joint_1.GetVelocity()
		# theta2_dot = self.joint_2.GetVelocity()


		a = -  self.l1*self.l2*self.m2*np.sin(theta2)*theta2_dot**2
		b = -2*self.l1*self.l2*self.m2*np.sin(theta2)*theta2_dot*theta1_dot
		r11 = a + b
		r21 =  self.l1*self.l2*self.m2*np.sin(theta2)*theta1_dot**2
		C = np.array([
						[r11],
						[r21]
					 ])

		return C

	def GetGravityMatrix(self, state):
		theta1 = state.joint_theta[self.leg_index + 0]
		theta2 = state.joint_theta[self.leg_index + 1]

		theta1_dot = state.joint_theta_dot[self.leg_index + 0]
		theta2_dot = state.joint_theta_dot[self.leg_index + 1]

		# theta1 = self.joint_1.GetAngle()
		# theta2 = self.joint_2.GetAngle()

		# theta1_dot = self.joint_1.GetVelocity()
		# theta2_dot = self.joint_2.GetVelocity()

		a = self.l2*self.m2*gravity*np.cos(theta1 + theta2)
		b = (self.m2 + self.m1)*self.l1*gravity*np.cos(theta1)
		r11 = a + b
		r12 = self.m2*self.l2*gravity*np.cos(theta1 + theta2)
		G = np.array([
						[-r11],
						[-r12]
					 ])

		return G