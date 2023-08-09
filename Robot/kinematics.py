import numpy as np
from utils import GetTransformationMatrix, GetInverseMatrix

class LegKinematics:
	def __init__(self, l1, l2):
		self.l1 = l1
		self.l2 = l2

	def FK(self, theta_1, theta_2):
		theta_1 = theta_1 - np.pi/2
		x = self.l2*np.cos(theta_1 + theta_2) + self.l1*np.cos(theta_1)
		y = self.l2*np.sin(theta_1 + theta_2) + self.l1*np.sin(theta_1)

		return x, y


	def IK(self, position):
		x = position[0]
		y = position[1]

		temp = (x**2 + y**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)

		if temp>1:
			temp = 1
		if temp<-1:
			temp = -1

		theta_2 = np.arccos(temp)


		theta_1 = np.arctan2(y, x) - np.arctan2((self.l2*np.sin(theta_2)), (self.l1 + self.l2*np.cos(theta_2)))

		return np.pi/2 + theta_1, theta_2


class BodyKinematics:
	def __init__(self, leg_hind_pos, leg_front_pos):
		self.leg_hind_pos = leg_hind_pos
		self.leg_front_pos = leg_front_pos

	def IK(self, positions, theta):
		i = 0
		for position in positions:
			if i == 0:
				x, y = self.leg_hind_pos
				p_leg = np.array([[x, y, 1]])
			else:
				x, y = self.leg_front_pos
				p_leg = np.array([[x, y, 1]])

			T_body_leg = GetTransformationMatrix(theta, x, y)
			R_body = GetTransformationMatrix(theta, 0, 0)

			p_leg = R_body@p_leg.T



			position = T_body_leg@position.T
			position = position - p_leg.flatten()


			positions[i, 0] = position[0]
			positions[i, 1] = position[1]
			i += 1

		return positions