from floatingBase import FloatingBase
from link import Link

class Quadruped:
	def __init__(self, floating_base_mass, floating_base_length, floating_base_height):
		self.floating_base = FloatingBase(floating_base_mass, floating_base_length, floating_base_height)
		self.links = []

	def AddLink(self, link_mass, link_length, link_height):
		link = Link(link_mass, link_length, link_height)
		self.links.appends(link)