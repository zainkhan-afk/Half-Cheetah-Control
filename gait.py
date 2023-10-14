import numpy as np
class Gait:
	def __init__(self, Tg, step_height = 0.05, max_step_size = 0.25, gait_type = "trot"):
		'''
		Tg - Gait time
		gait_type - Name of the gait 

		A gait matrix will be created according to the gait type chosen. The gait matrix will contain the flight and stance 
		sequence and the length of each sequence for each of the robot legs.
		'''
		self.Tg = Tg
		self.gait_type = gait_type
		self.step_height = step_height
		self.max_step_size = max_step_size

		self.gait_matrix = np.array([
										[0, 0.5],
										[0.5, 1.0]
								   ])

		# self.gait_matrix = np.array([
		# 								[0, 0.5],
		# 								[0, 0.5]
		# 						   ])

		self.m = 0
		self.l = 0

		self.last_leg_pos_hind = None
		self.last_leg_pos_front = None

	def FindLegPos(self, t_norm, step_size, leg_index, height_offset):
		leg_pos = None
		if t_norm < self.gait_matrix[leg_index, 0]:
			leg_pos = None

		elif self.gait_matrix[leg_index, 0] < t_norm < self.gait_matrix[leg_index, 1]:
			t_norm = (t_norm - self.gait_matrix[leg_index, 0]) / (self.gait_matrix[leg_index, 1] - self.gait_matrix[leg_index, 0])
			x = (t_norm - 0.5)*step_size
			# x = t_norm/2 * step_size
			y = np.sqrt((1 - (x**2)/((step_size/2)**2))*self.step_height**2)
			leg_pos = [x, y + height_offset]

			if leg_index == 0:
				self.last_leg_pos_hind = leg_pos
			else:
				self.last_leg_pos_front = leg_pos

		elif t_norm >	self.gait_matrix[leg_index, 1]:
			leg_pos = None

		return leg_pos

	def GetLegPosition(self, t, leg_positions, height_offset = -0.2):
		gait_cycle_t = t - self.Tg*int(t/self.Tg)
		t_norm = gait_cycle_t / self.Tg
		step_size = self.max_step_size

		hind_leg_pos = self.FindLegPos(t_norm, step_size, 0, height_offset)
		front_leg_pos = self.FindLegPos(t_norm, step_size, 1, height_offset)


		return hind_leg_pos, front_leg_pos