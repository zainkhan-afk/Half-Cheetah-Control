class Leg:
	def __init__(self, thigh_segment, shin_segment, hip_joint, knee_joint):
		self.thigh_segment = thigh_segment
		self.shin_segment = shin_segment
		self.hip_joint = hip_joint
		self.knee_joint = knee_joint

	def GetMassMatrix(self):
		r11 = self.shin_segment.body.mass*self.shin_segment.length**2 + 2*self.thigh_segment.length*self.shin_segment.length*self.shin_segment.body.mass*np.cos(self.knee_joint.theta) + self.thigh_segment.length*(self.shin_segment.body.mass + self.thigh_segment.body.mass)
		r12 = self.shin_segment.body.mass*self.shin_segment.length**2 +   self.thigh_segment.length*self.shin_segment.length*self.shin_segment.body.mass*np.cos(self.knee_joint.theta)
		r21 = self.shin_segment.body.mass*self.shin_segment.length**2 +   self.thigh_segment.length*self.shin_segment.length*self.shin_segment.body.mass*np.cos(self.knee_joint.theta)
		r22 = self.shin_segment.body.mass*self.shin_segment.length**2
		M = np.array([
						[r11, r12],
						[r21, r22]
					 ])


	def GetCoriolisMatrix(self):
		a = -self.thigh_segment.length*self.shin_segment.length*self.shin_segment.body.mass*np.sin(self.knee_joint.theta)*self.knee_joint.theta_dot**2
		b = -2*self.thigh_segment.length*self.shin_segment.length*self.shin_segment.body.mass*np.sin(self.knee_joint.theta)*self.knee_joint.theta_dot*self.shin_joint.theta_dot
		r11 = a + b
		r21 = self.thigh_segment.length*self.shin_segment.length*self.shin_segment.body.mass*np.sin(self.knee_joint.theta)*self.shin_joint.theta_dot**2
		C = np.array([
						[r11],
						[r21]
					 ])

	def GetGravityMatrix(self):
		pass