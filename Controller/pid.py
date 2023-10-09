import numpy as np
from Robot.Dynamics.simulateDynamics import SimulateDynamics
from utils import GetTransformationMatrix

class PID:
	def __init__(self, dynamicsModel, hind_hip_pos, front_hip_pos, P = 300, I = 0, D = 50):
		self.dynamicsSimulator = SimulateDynamics(dynamicsModel)
		self.hind_hip_pos, self.front_hip_pos = hind_hip_pos, front_hip_pos
		self.P = P;
		self.I = I;
		self.D = D;

		self.prev_error = np.array([0, 0, 0, 0]).astype("float32")
		self.error_sum = np.array([0, 0, 0, 0]).astype("float32")

		self.prev_goal_pos = None

	def GetHipPosition(self, position, theta, hip_position):
		world_T_robot = GetTransformationMatrix(0, position[0], position[1])
		robot_T_hip   = GetTransformationMatrix(theta, 0, 0)

		hip_position_world = world_T_robot@robot_T_hip@hip_position.T

		return [hip_position_world[0, 0], hip_position_world[1, 0]]


	def Solve(self, current_state, J, current_pos, current_body_theta, goal_pos, goal_body_theta):
		hind_hip_position_current  = self.GetHipPosition(current_pos, current_body_theta,
									hip_position = np.array([[self.hind_hip_pos[0], self.hind_hip_pos[1], 1]]))
		front_hip_position_current = self.GetHipPosition(current_pos,  current_body_theta,
									hip_position = np.array([[self.front_hip_pos[0], self.front_hip_pos[1], 1]]))

		hind_hip_position_goal  = self.GetHipPosition(goal_pos, goal_body_theta,
									hip_position = np.array([[self.hind_hip_pos[0], self.hind_hip_pos[1], 1]]))
		front_hip_position_goal = self.GetHipPosition(goal_pos,  goal_body_theta,
									hip_position = np.array([[self.front_hip_pos[0], self.front_hip_pos[1], 1]]))

		current_pos = np.array(hind_hip_position_current + front_hip_position_current)
		goal_pos    = np.array(hind_hip_position_goal + front_hip_position_goal)

		error = goal_pos - current_pos
		print()
		print("Current:", current_pos)
		print("Goal:   ", goal_pos)
		print("Error:  ", error)
		force = self.P*error + self.D*(self.prev_error - error) + self.I*self.error_sum

		new_state = self.dynamicsSimulator.GoToNextStateFD(force.reshape(len(force), 1), J, current_state)

		self.prev_error = error
		self.error_sum += error

		return new_state