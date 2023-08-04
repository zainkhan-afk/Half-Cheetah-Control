import numpy as np
class Physics:
	def __init__(self, delta_T):
		self.gravity = (0, 0, -9.8)
		self.delta_T = delta_T

	def Step(self, entities):
		pass