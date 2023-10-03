import numpy as np
from utils import *
class Leg:
	def __init__(self, thigh_segment, shin_segment, hip_joint, knee_joint):
		self.thigh_segment = thigh_segment
		self.shin_segment = shin_segment
		self.hip_joint = hip_joint
		self.knee_joint = knee_joint

	def GetMassMatrix(self):
		r11 = self.shin_segment.body.mass*self.shin_segment.height**2 + 2*self.thigh_segment.height*self.shin_segment.height*self.shin_segment.body.mass*np.cos(self.knee_joint.angle) + self.thigh_segment.width*(self.shin_segment.body.mass + self.thigh_segment.body.mass)
		r12 = self.shin_segment.body.mass*self.shin_segment.height**2 +   self.thigh_segment.height*self.shin_segment.height*self.shin_segment.body.mass*np.cos(self.knee_joint.angle)
		r21 = self.shin_segment.body.mass*self.shin_segment.height**2 +   self.thigh_segment.height*self.shin_segment.height*self.shin_segment.body.mass*np.cos(self.knee_joint.angle)
		r22 = self.shin_segment.body.mass*self.shin_segment.height**2
		M = np.array([
						[r11, r12],
						[r21, r22]
					 ])

		return M

	def GetCoriolisMatrix(self):
		a = -self.thigh_segment.height*self.shin_segment.height*self.shin_segment.body.mass*np.sin(self.knee_joint.angle)*self.knee_joint.motorSpeed**2
		b = -2*self.thigh_segment.height*self.shin_segment.height*self.shin_segment.body.mass*np.sin(self.knee_joint.angle)*self.knee_joint.motorSpeed*self.hip_joint.motorSpeed
		r11 = a + b
		r21 = self.thigh_segment.height*self.shin_segment.height*self.shin_segment.body.mass*np.sin(self.knee_joint.angle)*self.hip_joint.motorSpeed**2
		C = np.array([
						[r11],
						[r21]
					 ])

		return C

	def GetGravityMatrix(self):
		a = self.shin_segment.height*self.shin_segment.body.mass*gravity*np.cos(self.hip_joint.angle)*np.cos(self.knee_joint.angle)
		b = (self.shin_segment.body.mass + self.thigh_segment.body.mass)* self.thigh_segment.height*gravity*np.cos(self.hip_joint.angle)
		r11 = a + b
		r12 = self.shin_segment.body.mass*self.shin_segment.body.mass*gravity*np.cos(self.hip_joint.angle)*np.cos(self.knee_joint.angle)
		G = np.array([
						[r11],
						[r12]
					 ])

		return G