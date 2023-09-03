class Link:
	def __init__(self, mass, length, height):
		self.mass = mass
		self.length = length
		self.height = height
		self.I = []

	def CalculateMassMatrix(self):
		Iz = 1/3 * self.mass * (self.length**2 + self.height**2)
		Ixx = 1/12 * self.mass * (self.length**2 + self.height**2)
		Iyy = Ixx

		self.I = np.array([
						[Ixx,   0,  0],
						[  0, Iyy,  0],
						[  0,   0, Iz]
						])