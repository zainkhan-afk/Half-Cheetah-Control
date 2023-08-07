class JointController:
	def __init__(self, joint):
		self.joint = joint
		self.current_angle = self.joint.angle 

		self.P = 1
		self.D = 0.1

		self.prev_error = 0

	def MoveTo(self, desired_angle):
		self.current_angle = self.joint.angle
		error = desired_angle - self.current_angle

		val = self.P*error + self.D*(self.prev_error - error)

		self.joint.motorSpeed = val

		self.prev_error = error