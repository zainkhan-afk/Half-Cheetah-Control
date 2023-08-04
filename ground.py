from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)
class Ground:
	def __init__(self, sim_handle):
		self.body = sim_handle.world.CreateStaticBody(
						    position=(0, 1),
						    shapes=polygonShape(box=(50, 1)),
						)

		self.color = (125, 125, 125)