import pygame
from Box2D import b2Filter
class LegSegment:
	def __init__(self, sim_handle, position, angle, width, height, group_index):
		self.body = sim_handle.world.CreateDynamicBody(position=position, angle=angle)
		box = self.body.CreatePolygonFixture(box=(width, height), density=2.5, friction=0.3, filter = b2Filter(groupIndex=group_index))
		print(self.body.mass)
		
		self.color = (200, 200, 255)

	def Render(self, screen, PPM):
		H = screen.get_height()
		for fixture in self.body.fixtures:
			shape = fixture.shape
			vertices = [(self.body.transform * v) * PPM for v in shape.vertices]
			vertices = [(v[0], H - v[1]) for v in vertices]
			pygame.draw.polygon(screen, self.color, vertices)