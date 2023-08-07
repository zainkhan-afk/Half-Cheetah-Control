import pygame
import numpy as np
from .torso import Torso
from .legSegment import LegSegment
from .kinematics import Kinematics
from .jointController import JointController

class Cheetah:
	def __init__(self, sim_handle, position = (0, 0), angle = 0):

		self.torso_width = 0.5
		self.torso_height = 0.1

		self.leg_width = 0.01
		self.leg_height = 0.2

		self.front_leg_thigh_joint_pos = (position[0] + self.torso_width, position[1])
		self.hind_leg_thigh_joint_pos  = (position[0] - self.torso_width, position[1])
		self.front_leg_shin_joint_pos = (position[0] + self.torso_width, position[1] - 2*self.leg_height)
		self.hind_leg_shin_joint_pos  = (position[0] - self.torso_width, position[1] - 2*self.leg_height)

		self.front_leg_thigh_pos = (position[0] + self.torso_width, position[1] - self.leg_height)
		self.hind_leg_thigh_pos  = (position[0] - self.torso_width, position[1] - self.leg_height)
		self.front_leg_shin_pos = (position[0] + self.torso_width, position[1] - 3*self.leg_height)
		self.hind_leg_shin_pos  = (position[0] - self.torso_width, position[1] - 3*self.leg_height)

		self.torso = Torso(sim_handle, position, angle, self.torso_width, self.torso_height, group_index = -1)
		
		self.thigh_front = LegSegment(sim_handle, self.front_leg_thigh_pos, angle, self.leg_width, self.leg_height, group_index = -1)
		self.thigh_hind = LegSegment(sim_handle, self.hind_leg_thigh_pos, angle, self.leg_width, self.leg_height, group_index = -1)

		self.shin_front = LegSegment(sim_handle, self.front_leg_shin_pos, angle, self.leg_width, self.leg_height, group_index = -1)
		self.shin_hind = LegSegment(sim_handle, self.hind_leg_shin_pos, angle, self.leg_width, self.leg_height, group_index = -1)

		self.front_thigh_joint = sim_handle.world.CreateRevoluteJoint(
									bodyA = self.torso.body,
									bodyB = self.thigh_front.body,
									anchor = self.front_leg_thigh_joint_pos,
									maxMotorTorque = 10000.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)


		self.hind_thigh_joint = sim_handle.world.CreateRevoluteJoint(
									bodyA = self.torso.body,
									bodyB = self.thigh_hind.body,
									anchor = self.hind_leg_thigh_joint_pos,
									maxMotorTorque = 10000.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)

		self.front_shin_joint = sim_handle.world.CreateRevoluteJoint(
									bodyA = self.thigh_front.body,
									bodyB = self.shin_front.body,
									anchor = self.front_leg_shin_joint_pos,
									maxMotorTorque = 10000.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)

		self.hind_shin_joint = sim_handle.world.CreateRevoluteJoint(
									bodyA = self.thigh_hind.body,
									bodyB = self.shin_hind.body,
									anchor = self.hind_leg_shin_joint_pos,
									maxMotorTorque = 10000.0,
									motorSpeed = 0.0,
									enableMotor = True,
									)

		self.front_thigh_joint_controller = JointController(self.front_thigh_joint)
		self.hind_thigh_joint_controller = JointController(self.hind_thigh_joint)
		self.front_shin_joint_controller = JointController(self.front_shin_joint)
		self.hind_shin_joint_controller = JointController(self.hind_shin_joint)

		self.kine_model = Kinematics(self.leg_height, self.leg_height)

	def StandUp(self, height = 0.3):

		theta_1, theta_2 = self.kine_model.IK(x = 0, y = -height)

		# theta_1 = 10/180*np.pi
		# theta_2 = 10/180*np.pi

		self.front_thigh_joint_controller.MoveTo(theta_1)
		self.hind_thigh_joint_controller.MoveTo(theta_1)
		self.front_shin_joint_controller.MoveTo(theta_2)
		self.hind_shin_joint_controller.MoveTo(theta_2)

	def Render(self, screen, PPM):
		self.torso.Render(screen, PPM)
		
		self.thigh_front.Render(screen, PPM)
		self.thigh_hind.Render(screen, PPM)

		self.shin_front.Render(screen, PPM)
		self.shin_hind.Render(screen, PPM)