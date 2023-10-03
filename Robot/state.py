# from utils import *

class State:
	def __init__(self, position, velocity, acceleration, theta, theta_dot, theta_double_dot):
		self.position = position
		self.velocity = velocity
		self.acceleration = acceleration

		self.theta = theta
		self.theta_dot = theta_dot
		self.theta_double_dot = theta_double_dot

	def SetPosition(self, position):
		self.position = position

	def SetTheta(self, theta):
		self.theta = theta

	def UpdateUsingPosition(self, position):
		new_velocity = (position - self.position)/TIME_STEP
		new_acceleration = (new_velocity - self.velocity)/TIME_STEP

		return State(position, new_velocity, new_acceleration, self.theta, self.theta_dot. self.theta_double_dot)

	def UpdateUsingVelocity(self, velocity):
		new_position = self.position + velocity*TIME_STEP
		new_acceleration = (velocity - self.velocity)/TIME_STEP

		return State(new_position, velocity, new_acceleration, self.theta, self.theta_dot. self.theta_double_dot)

	def UpdateUsingAcceleration(self, acceleration):
		new_velocity = self.velocity + acceleration*TIME_STEP
		new_position = self.position + new_velocity*TIME_STEP

		return State(new_position, new_velocity, acceleration, self.theta, self.theta_dot. self.theta_double_dot)

	def UpdateUsingTheta(self, theta):
		new_theta_dot = (theta - self.theta) / TIME_STEP
		new_theta_double_dot = (new_theta_dot - self.theta_dot) / TIME_STEP

		return State(self.position, self.velocity, self.acceleration, theta, new_theta_dot, new_theta_double_dot)

	def UpdateUsingThetaDot(self, theta_dot):
		new_theta = self.theta + theta_dot*TIME_STEP
		new_theta_double_dot = (theta_dot - self.theta_dot) / TIME_STEP

		return State(self.position, self.velocity, self.acceleration, new_theta, theta_dot, new_theta_double_dot)

	def UpdateUsingThetaDoubleDot(self, theta_double_dot):
		new_theta_dot = self.theta_dot + theta_double_dot*TIME_STEP
		new_theta = self.theta + new_theta_dot*TIME_STEP

		return State(self.position, self.velocity, self.acceleration, new_theta, new_theta_dot, theta_double_dot) 


	def Update(self, position, velocity, acceleration, theta, theta_dot, theta_double_dot):
		return State(position, velocity, acceleration, theta, theta_dot, theta_double_dot)


	def __str__(self):
		return f'''
		Body:
			Position: {self.position[0], self.position[1]}
			Velocity: {self.velocity[0], self.velocity[1]}
			Acceleration: {self.acceleration[0], self.acceleration[1]}

		Joints:
			Leg 1:
				Theta: {self.theta[0], self.theta[1]}
				Theta Dot: {self.theta_dot[0], self.theta_dot[1]}
				Theta Double Dot: {self.theta_double_dot[0], self.theta_double_dot[1]}

			Leg 2:
				Theta: {self.theta[2], self.theta[3]}
				Theta Dot: {self.theta_dot[2], self.theta_dot[3]}
				Theta Double Dot: {self.theta_double_dot[2], self.theta_double_dot[3]}
		'''

if __name__ == "__main__":
	import numpy as np
	
	pos = np.array([0, 0])
	vel = np.array([0, 0])
	acc = np.array([0, 0])

	theta = np.array([0, 0, 0, 0])
	theta_dot = np.array([0, 0, 0, 0])
	theta_double_dot = np.array([0, 0, 0, 0])

	s = State(pos, vel, acc, theta, theta_dot, theta_double_dot)

	print(s)