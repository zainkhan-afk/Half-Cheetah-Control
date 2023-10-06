import numpy as np

from .floatingBase import FloatingBase
from .legSegment import LegSegment
from .leg import Leg

from utils import GetInverseMatrix

class QuadrupedDynamics:
	def __init__(self, floating_base):
		self.floating_base = FloatingBase(floating_base)
		self.links = []
		self.links.append(self.floating_base)
		self.legs = {}
		self.composite_inertia = None


	def AddLeg(self, name, thigh_link, shin_link, hip_joint, knee_joint):
		# print(hip_joint)
		self.legs[name] = Leg(thigh_link, shin_link, hip_joint, knee_joint, leg_name = name)
		self.AddLegSegment(thigh_link)
		self.AddLegSegment(shin_link)

	def AddLegSegment(self, link_object):
		link = LegSegment(link_object)
		self.links.append(link)

	def CalculateCompositeRigidBodyInertia(self):
		self.composite_inertia = np.zeros((3, 3))
		
		for link in self.links:
			self.composite_inertia += link.GetTransform()@link.GetSpatialInertia()@link.GetTransform().T

		# print(self.composite_inertia)

	def GetCompositeInertia(self):
		return self.composite_inertia

	def ForwardDynamics(self, forces, jacobian, state, old_torques = np.zeros((4, 1))):
		M = self.GetMassMatrix(state)
		C = self.GetCoriolisMatrix(state)
		G = self.GetGravityMatrix(state)

		selection_vector = np.zeros((7, 1))
		selection_vector[3: ] = old_torques

		EE_front = np.array([[jacobian[2, 0], jacobian[2, 1]]])
		EE_hind  = np.array([[jacobian[2, 2], jacobian[2, 3]]])

		f_front = np.array([[forces[0, 0], forces[1, 0]]]).T
		f_hind  = np.array([[forces[2, 0], forces[3, 0]]]).T

		torque_from_front = EE_front@f_front
		torque_from_hind  = EE_hind@f_hind

		jacobian_force_result       = jacobian@forces
		jacobian_force_result[2, 0] = torque_from_front - torque_from_hind

		joint_torques = jacobian_force_result[3:, :]

		M_inv = GetInverseMatrix(M)

		theta_double_dot = M_inv@(selection_vector + jacobian_force_result - C - G)

		# print(theta_double_dot)
		# print(selection_vector)

		return theta_double_dot, joint_torques


	def GetMassMatrix(self, state):
		M = np.zeros((7, 7))

		M[:3, :3] = self.composite_inertia
		M[3:5, 3:5] = self.legs["hind"].GetMassMatrix(state)
		M[5:, 5:] = self.legs["front"].GetMassMatrix(state)
		# print()
		# for r in range(M.shape[0]):
		# 	for c in range(M.shape[1]):
		# 		if abs(M[r, c])>0:
		# 			val = round(M[r, c], 5)
		# 		else:
		# 			val = "0.0000"
		# 		print(f"{val}", end=" ")
		# 	print()

		return M


	def GetCoriolisMatrix(self, state):
		C = np.zeros((7, 1))
		C[3:5] = self.legs["hind"].GetCoriolisMatrix(state)
		C[5: ] = self.legs["front"].GetCoriolisMatrix(state)

		# print()
		# for r in range(C.shape[0]):
		# 	for c in range(C.shape[1]):
		# 		if abs(C[r, c])>0:
		# 			val = round(C[r, c], 10)
		# 		else:
		# 			val = "0.0000"
		# 		print(f"{val}", end=" ")
		# 	print()

		return C

	def GetGravityMatrix(self, state):
		G = np.zeros((7, 1))
		G[ :3] = self.floating_base.GetGravityMatrix()
		G[3:5] = self.legs["hind"].GetGravityMatrix(state)
		G[5: ] = self.legs["front"].GetGravityMatrix(state)

		# print()
		# for r in range(G.shape[0]):
		# 	for c in range(G.shape[1]):
		# 		if abs(G[r, c])>0:
		# 			val = round(G[r, c], 10)
		# 		else:
		# 			val = "0.0000"
		# 		print(f"{val}", end=" ")
		# 	print()

		return G