import numpy as np

class Kinematics:
	def __init__(self, l1, l2):
		self.l1 = l1
		self.l2 = l2

	def FK(self):
		pass


	def IK(self, x, y):
		temp = (x**2 + y**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)

		if temp>1:
			temp = 1
		if temp<-1:
			temp = -1

		theta_2 = np.arccos(temp)


		theta_1 = np.arctan2(y, x) - np.arctan2((self.l2*np.sin(theta_2)), (self.l1 + self.l2*np.cos(theta_2)))

		return np.pi/2 + theta_1, theta_2