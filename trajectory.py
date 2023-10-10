class Trajectory:
	def __init__(self, size):
		self.path = []
		self.size = size

	def AddPathPoint(self, path_point):
		self.path.append(path_point)

		while len(self.path) >= self.size:
			self.path.reverse()
			self.path.pop()
			self.path.reverse()

	