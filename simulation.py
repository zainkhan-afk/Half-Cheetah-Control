from renderer import Renderer

class Simulation:
	def __init__(self, width = 600, height = 400, delta_T = 0.1):
		self.entities = []
		
		self.width = width
		self.height = height
		self.delta_T = delta_T

		self.renderer = Renderer(width, height)

	def AddEntity(self, entity):
		self.entities.append(entity)

	def Render(self):
		simulation_running = self.renderer.Poll()

		if simulation_running:
			self.renderer.Clear()
			for entity in self.entities:
				self.renderer.Render(entity)


		return simulation_running

	def Step(self):
		pass