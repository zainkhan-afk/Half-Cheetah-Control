class JointController:
	def __init__(self, joint):
		self.joint = joint
		self.current_angle = self.joint.angle 

		self.P = 0.2
		self.D = 0.05

		self.prev_error = 0

	def SetAngle(self, desired_angle):
		self.current_angle = self.joint.angle
		error = desired_angle - self.current_angle

		val = self.P*error + self.D*(error - self.prev_error)

		self.joint.motorSpeed = val

		self.prev_error = error