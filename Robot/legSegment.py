import pygame
class LegSegment:
	def __init__(self, sim_handle, position, angle, width, height):
		self.body = sim_handle.world.CreateDynamicBody(position=position, angle=angle)
		box = self.body.CreatePolygonFixture(box=(width, height), density=1, friction=0.3)

		self.color = (200, 200, 255)

	def Render(self, screen, PPM):
		H = screen.get_height()
		for fixture in self.body.fixtures:
			shape = fixture.shape
			vertices = [(self.body.transform * v) * PPM for v in shape.vertices]
			vertices = [(v[0], H - v[1]) for v in vertices]
			pygame.draw.polygon(screen, self.color, vertices)