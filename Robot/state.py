from utils import *

class State:
	def __init__(self, position, velocity, acceleration, 
				body_theta, body_theta_dot, body_theta_double_dot,
				joint_theta, joint_theta_dot, joint_theta_double_dot):
		self.position = position
		self.velocity = velocity
		self.acceleration = acceleration

		self.body_theta = body_theta
		self.body_theta_dot = body_theta_dot
		self.body_theta_double_dot = body_theta_double_dot

		self.joint_theta = joint_theta
		self.joint_theta_dot = joint_theta_dot
		self.joint_theta_double_dot = joint_theta_double_dot

	def SetPosition(self, position):
		self.position = position

	def SetJointTheta(self, joint_theta):
		self.joint_theta = joint_theta

	def UpdateUsingPosition(self, position):
		new_velocity = (position - self.position)/TIME_STEP
		new_acceleration = (new_velocity - self.velocity)/TIME_STEP

		return State(position, new_velocity, new_acceleration, 
			self.body_theta, self.body_theta_dot, self.body_theta_double_dot,
			self.joint_theta, self.joint_theta_dot, self.joint_theta_double_dot)

	def UpdateUsingVelocity(self, velocity):
		new_position = self.position + velocity*TIME_STEP
		new_acceleration = (velocity - self.velocity)/TIME_STEP

		return State(new_position, velocity, new_acceleration, 
			self.body_theta, self.body_theta_dot, self.body_theta_double_dot,
			self.joint_theta, self.joint_theta_dot, self.joint_theta_double_dot)

	def UpdateUsingAcceleration(self, acceleration):
		new_velocity = self.velocity + acceleration*TIME_STEP
		new_position = self.position + new_velocity*TIME_STEP

		return State(new_position, new_velocity, acceleration,
			self.body_theta, self.body_theta_dot, self.body_theta_double_dot,
			self.joint_theta, self.joint_theta_dot, self.joint_theta_double_dot)

	def UpdateUsingBodyTheta(self, body_theta):
		new_body_theta_dot = (body_theta - self.body_theta) / TIME_STEP
		new_body_theta_double_dot = (new_body_theta_dot - self.body_theta_dot) / TIME_STEP

		return State(self.position, self.velocity, self.acceleration, 
					body_theta, new_body_theta_dot, new_body_theta_double_dot,
					self.joint_theta, self.joint_theta_dot, self.joint_theta_double_dot)

	def UpdateUsingBodyThetaDot(self, body_theta_dot):
		new_body_theta = self.body_theta + body_theta_dot*TIME_STEP
		new_body_theta_double_dot = (body_theta_dot - self.body_theta_dot) / TIME_STEP

		return State(self.position, self.velocity, self.acceleration, 
			new_body_theta, body_theta_dot, new_body_theta_double_dot,
			self.joint_theta, self.joint_theta_dot, self.joint_theta_double_dot)

	def UpdateUsingBodyThetaDoubleDot(self, body_theta_double_dot):
		new_body_theta_dot = self.body_theta_dot + body_theta_double_dot*TIME_STEP
		new_body_theta = self.body_theta + new_body_theta_dot*TIME_STEP

		return State(self.position, self.velocity, self.acceleration, 
			new_body_theta, new_body_theta_dot, body_theta_double_dot,
			self.joint_theta, self.joint_theta_dot, self.joint_theta_double_dot) 

	def UpdateUsingJointTheta(self, joint_theta):
		new_joint_theta_dot = (joint_theta - self.joint_theta) / TIME_STEP
		new_joint_theta_double_dot = (new_joint_theta_dot - self.joint_theta_dot) / TIME_STEP

		return State(self.position, self.velocity, self.acceleration, 
			self.body_theta, self.body_theta_dot, self.body_theta_double_dot,
			joint_theta, new_joint_theta_dot, new_joint_theta_double_dot)

	def UpdateUsingJointThetaDot(self, joint_theta_dot):
		new_joint_theta = self.joint_theta + joint_theta_dot*TIME_STEP
		new_joint_theta_double_dot = (joint_theta_dot - self.joint_theta_dot) / TIME_STEP

		return State(self.position, self.velocity, self.acceleration, 
			self.body_theta, self.body_theta_dot, self.body_theta_double_dot,
			new_joint_theta, joint_theta_dot, new_joint_theta_double_dot)

	def UpdateUsingJointThetaDoubleDot(self, joint_theta_double_dot):
		new_joint_theta_dot = self.joint_theta_dot + joint_theta_double_dot*TIME_STEP
		new_joint_theta = self.joint_theta + new_joint_theta_dot*TIME_STEP

		return State(self.position, self.velocity, self.acceleration, 
			self.body_theta, self.body_theta_dot, self.body_theta_double_dot,
			new_joint_theta, new_joint_theta_dot, joint_theta_double_dot) 


	def Update(self, position, velocity, acceleration, body_theta, body_theta_dot, body_theta_double_dot, joint_theta, joint_theta_dot, joint_theta_double_dot):
		return State(position, velocity, acceleration, body_theta, body_theta_dot, body_theta_double_dot, joint_theta, joint_theta_dot, joint_theta_double_dot)


	def __str__(self):
		return f'''
		Body:
			Translation:
				Position: {self.position[0], self.position[1]}
				Velocity: {self.velocity[0], self.velocity[1]}
				Acceleration: {self.acceleration[0], self.acceleration[1]}

			Rotation:
				Body Theta: {self.body_theta}
				Body Theta Dot: {self.body_theta_dot}
				Body Theta Double Dot: {self.body_theta_double_dot}

		Joints:
			Leg Hind:
				Joint Theta: {self.joint_theta[0], self.joint_theta[1]}
				Joint Theta Dot: {self.joint_theta_dot[0], self.joint_theta_dot[1]}
				Joint Theta Double Dot: {self.joint_theta_double_dot[0], self.joint_theta_double_dot[1]}

			Leg Front:
				Joint Theta: {self.joint_theta[2], self.joint_theta[3]}
				Joint Theta Dot: {self.joint_theta_dot[2], self.joint_theta_dot[3]}
				Joint Theta Double Dot: {self.joint_theta_double_dot[2], self.joint_theta_double_dot[3]}
		'''