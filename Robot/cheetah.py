import pygame
import numpy as np
from .torso import Torso
from .legSegment import LegSegment
from .leg import Leg

class Cheetah:
	def __init__(self, sim_handle, position = (0, 0), angle = 0):

		self.torso_width = 0.5
		self.torso_height = 0.1

		self.leg_width = 0.01
		self.leg_segment_length = 0.2

		self.front_leg_thigh_joint_pos = (position[0] + self.torso_width, position[1])
		self.hind_leg_thigh_joint_pos  = (position[0] - self.torso_width, position[1])
		self.front_leg_shin_joint_pos = (position[0] + self.torso_width, position[1] - 2*self.leg_segment_length)
		self.hind_leg_shin_joint_pos  = (position[0] - self.torso_width, position[1] - 2*self.leg_segment_length)

		self.front_leg_thigh_pos = (position[0] + self.torso_width, position[1] - self.leg_segment_length)
		self.hind_leg_thigh_pos  = (position[0] - self.torso_width, position[1] - self.leg_segment_length)
		self.front_leg_shin_pos = (position[0] + self.torso_width, position[1] - 3*self.leg_segment_length)
		self.hind_leg_shin_pos  = (position[0] - self.torso_width, position[1] - 3*self.leg_segment_length)

		self.torso = Torso(sim_handle, position, angle, self.torso_width, self.torso_height, group_index = -1)
		
		self.thigh_front = LegSegment(sim_handle, self.front_leg_thigh_pos, angle, self.leg_width, self.leg_segment_length, group_index = -1)
		self.thigh_hind = LegSegment(sim_handle, self.hind_leg_thigh_pos, angle, self.leg_width, self.leg_segment_length, group_index = -1)

		self.shin_front = LegSegment(sim_handle, self.front_leg_shin_pos, angle, self.leg_width, self.leg_segment_length, group_index = -1)
		self.shin_hind = LegSegment(sim_handle, self.hind_leg_shin_pos, angle, self.leg_width, self.leg_segment_length, group_index = -1)

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

		self.leg_front = Leg(self.front_thigh_joint, self.front_shin_joint, self.leg_segment_length)
		self.leg_hind = Leg(self.hind_thigh_joint, self.hind_shin_joint, self.leg_segment_length)

	def StandUp(self, height = 0.3):
		self.leg_front.MoveTo(x = 0, y = -height)
		self.leg_hind.MoveTo(x = 0, y = -height)

	def Render(self, screen, PPM):
		self.torso.Render(screen, PPM)
		
		self.thigh_front.Render(screen, PPM)
		self.thigh_hind.Render(screen, PPM)

		self.shin_front.Render(screen, PPM)
		self.shin_hind.Render(screen, PPM)