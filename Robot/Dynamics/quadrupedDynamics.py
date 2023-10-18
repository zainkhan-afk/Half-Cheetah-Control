import numpy as np

from .floatingBase import FloatingBase
from .legSegment import LegSegment
from .leg import Leg

from utils import GetInverseMatrix, GetAngle

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

	def CalculateCompositeRigidBodyInertiaWRTWorld(self):
		self.composite_inertia = np.zeros((3, 3))
		
		for link in self.links:
			world_T_link = link.GetTransform()
			self.composite_inertia += world_T_link@link.GetSpatialInertia()@(world_T_link.T)

	def CalculateCompositeRigidBodyInertiaWRTFloatingBase(self):
		world_T_FB = self.links[0].GetTransform()
		FB_T_world = GetInverseMatrix(world_T_FB)
		
		self.composite_inertia = np.zeros((3, 3))
		
		for link in self.links:
			world_T_link = link.GetTransform()
			FB_T_link = FB_T_world@world_T_link
			self.composite_inertia += FB_T_link@link.GetSpatialInertia()@(FB_T_link.T)

	def GetCompositeInertia(self):
		return self.composite_inertia

	def ForwardDynamics(self, forces, jacobian, state, old_torques = np.zeros((4, 1))):
		M = self.GetMassMatrix(state)
		C = self.GetCoriolisMatrix(state)
		G = self.GetGravityMatrix(state)

		# if forces[1, 0] > 0:
		# 	forces = np.array([[0, 0, 0, 0]]).T

		# if forces[3, 0] > 0:
		# 	forces = np.array([[0, 0, 0, 0]]).T

		selection_vector = np.zeros((7, 1))
		selection_vector[3: ] = old_torques

		jacobian_body = jacobian[:3, :]
		jacobian_legs = jacobian[3:, :]

		EE_hind = np.array([[jacobian_body[2, 0], jacobian_body[2, 1]]])
		EE_front  = np.array([[jacobian_body[2, 2], jacobian_body[2, 3]]])
		
		f_hind = np.array([[forces[0, 0], forces[1, 0]]])
		f_front  = np.array([[forces[2, 0], forces[3, 0]]])


		force_angle_hind  = GetAngle(f_hind.ravel(), np.array([0, 0]))
		force_angle_front = GetAngle(f_front.ravel(), np.array([0, 0]))

		torque_from_hind = np.cross(EE_hind, f_hind)[0]
		torque_from_front = np.cross(EE_front, f_front)[0]

		forces_body = jacobian_body@forces
		torque_legs = jacobian_legs.T@forces

		resultant_torques = np.zeros((7, 1))
		resultant_torques[:3, :] = forces_body
		resultant_torques[2, 0] = torque_from_front + torque_from_hind
		resultant_torques[3:, :] = torque_legs

		joint_torques = resultant_torques[3:, :]

		M_inv = GetInverseMatrix(M)

		theta_double_dot = M_inv@(resultant_torques - C - G)

		return theta_double_dot, joint_torques


	def GetMassMatrix(self, state):
		M = np.zeros((7, 7))

		M[:3, :3] = self.composite_inertia
		M[3:5, 3:5] = self.legs["hind"].GetMassMatrix(state)
		M[5:, 5:] = self.legs["front"].GetMassMatrix(state)

		return M


	def GetCoriolisMatrix(self, state):
		C = np.zeros((7, 1))
		C[3:5] = self.legs["hind"].GetCoriolisMatrix(state)
		C[5: ] = self.legs["front"].GetCoriolisMatrix(state)

		return C

	def GetGravityMatrix(self, state):
		G = np.zeros((7, 1))
		G[ :3] = self.floating_base.GetGravityMatrix()
		G[3:5] = self.legs["hind"].GetGravityMatrix(state)
		G[5: ] = self.legs["front"].GetGravityMatrix(state)

		return G