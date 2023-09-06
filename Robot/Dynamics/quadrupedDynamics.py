import numpy as np

from .floatingBase import FloatingBase
from .legSegment import LegSegment
from .leg import Leg

class QuadrupedDynamics:
	def __init__(self, floating_base):
		self.floating_base = FloatingBase(floating_base)
		self.links = []
		self.links.append(self.floating_base)
		self.legs = {}
		self.composite_inertia = None


	def AddLeg(self, name, thigh_link, shin_link):
		self.legs[name] = {"thigh": thigh_link, "shin": shin_link}
		self.AddLegSegment(thigh_link)
		self.AddLegSegment(shin_link)

	def AddLegSegment(self, link_object):
		link = LegSegment(link_object)
		self.links.append(link)

	def CalculateCompositeRigidBodyInertia(self):
		self.composite_inertia = np.zeros((3, 3))
		
		for link in self.links:
			self.composite_inertia += link.GetTransform()@link.GetSpatialInertia()@link.GetTransform().T

	def GetCompositeInertia(self):
		return self.composite_inertia

	def GetMassMatrix(self):
		self.M = np.zeros((7, 7))

		self.M[:3, :3] = self.composite_inertia

		print(self.M)

		return self.M