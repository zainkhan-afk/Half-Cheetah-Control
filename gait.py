class Gait:
	def __init__(self, Tg, step_height = 0.01, max_step_size = 0.5, gait_type = "trot"):
		'''
		Tg - Gait time
		gait_type - Name of the gait 

		A gait matrix will be created according to the gait type chosen. The gait matrix will contain the flight and stance 
		sequence and the length of each sequence for each of the robot legs.
		The gait matrix will have 2 variables per leg, the start of swing phase and the end of swing phase, the rest of the
		times the robot leg will be in stance phase.
		'''
		self.Tg = Tg
		self.gait_type = gait_type

		if self.gait_type == "trot":
			self.gait_matrix = [
									[0, 0.5],
									[0.5, 1]
							   ]

	def GetLegPosition(self, t, leg_positions):
		gait_cycle_t = t - self.Tg*int(t/self.Tg)