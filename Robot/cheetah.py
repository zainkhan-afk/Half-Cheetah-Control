import pygame
import numpy as np
from utils import AlmostEqual

from .torso import Torso
from .legSegment import LegSegment
from .leg import Leg
from .kinematics import BodyKinematics
from .state import State

from .Dynamics import QuadrupedDynamics


class Cheetah:
	def __init__(self, sim_handle, position = np.array([0, 0]), angle = 0):
		self.torso_width = 0.5
		self.torso_height = 0.1

		self.leg_width = 0.01
		self.leg_segment_length = 0.2

		self.body_angle = 0

		self.leg_hind_pos = (- self.torso_width, 0)
		self.leg_front_pos = ( self.torso_width, 0)

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

		self.body_kine_model = BodyKinematics(self.leg_hind_pos, self.leg_front_pos)
		self.dynamics = self.SetUpRobotDynamics()

		self.dynamics.CalculateCompositeRigidBodyInertia()

		self.old_torques = np.zeros((4, 1))


		hind_theta_thigh, hind_theta_shin = self.leg_hind.GetAngles()
		front_theta_thigh, front_theta_shin = self.leg_front.GetAngles()
		
		self.state = State(position, np.array([0, 0]), np.array([0, 0]),
							angle, 0, 0,
							np.array([hind_theta_thigh, hind_theta_shin, front_theta_thigh, front_theta_shin]), 
							np.array([0, 0, 0, 0]), np.array([0, 0, 0, 0]))


	def SetUpRobotDynamics(self):
		floating_body_pos = np.array([self.torso.body.position[0], self.torso.body.position[1]])

		dynamics = QuadrupedDynamics(self.torso)
		dynamics.AddLeg("hind", self.thigh_hind, self.shin_hind, self.hind_thigh_joint, self.hind_shin_joint)
		dynamics.AddLeg("front", self.thigh_front, self.shin_front, self.front_thigh_joint, self.front_shin_joint)

		return dynamics

	def StandUp(self, height = 0.3):
		pass

	def Rest(self, height = 0.2):
		positions = np.array([
								[0, -height, 1],
								[0, -height, 1]
							])

		positions = self.body_kine_model.IK(positions, self.body_angle)
		
		self.leg_front.MoveTo(positions[0, :])
		self.leg_hind.MoveTo( positions[1, :])

		desired_pos_leg_front = np.array([[positions[0, 0], positions[0, 1]]]).T

		self.leg_hind.MoveTo( positions[0, :])
		self.leg_front.MoveTo(positions[1, :])

		hind_theta_thigh, hind_theta_shin = self.leg_hind.GetAngles()
		front_theta_thigh, front_theta_shin = self.leg_front.GetAngles()

		body_position = self.torso.GetPosition()
		body_angle = self.torso.body.angle

		body_position = np.array([body_position[0], body_position[1]])

		self.state = self.state.UpdateUsingJointTheta(np.array([hind_theta_thigh, hind_theta_shin, front_theta_thigh, front_theta_shin]))
		self.state = self.state.UpdateUsingPosition(body_position)
		self.state = self.state.UpdateUsingBodyTheta(body_angle)

		return AlmostEqual(desired_pos_leg_front, self.leg_front.GetEEFKPosition())

	def UpdateState(self):
		hind_theta_thigh, hind_theta_shin = self.leg_hind.GetAngles()
		front_theta_thigh, front_theta_shin = self.leg_front.GetAngles()

		body_position = self.torso.GetPosition()
		body_angle = self.torso.body.angle

		body_position = np.array([body_position[0], body_position[1]])

		self.state = self.state.UpdateUsingJointTheta(np.array([hind_theta_thigh, hind_theta_shin, front_theta_thigh, front_theta_shin]))
		self.state = self.state.UpdateUsingPosition(body_position)
		self.state = self.state.UpdateUsingBodyTheta(body_angle)

		print(self.state)

	def ApplyForceToLegs(self, forces):
		self.dynamics.CalculateCompositeRigidBodyInertia()

		hind_theta_thigh, hind_theta_shin = self.leg_hind.GetAngles()
		front_theta_thigh, front_theta_shin = self.leg_front.GetAngles()

		body_position = self.torso.GetPosition()
		body_angle = self.torso.body.angle

		body_position = np.array([body_position[0], body_position[1]])

		self.state = self.state.UpdateUsingJointTheta(np.array([hind_theta_thigh, hind_theta_shin, front_theta_thigh, front_theta_shin]))
		self.state = self.state.UpdateUsingPosition(body_position)
		self.state = self.state.UpdateUsingBodyTheta(body_angle)


		jacobian = np.zeros((7, 4))

		hind_ee_pos  = self.leg_hind.GetEEFKPosition()
		front_ee_pos = self.leg_front.GetEEFKPosition()

		hind_J  = self.leg_hind.GetJacobian()
		front_J = self.leg_front.GetJacobian()

		body_J  = self.body_kine_model.GetJacobian(hind_ee_pos, front_ee_pos)

		jacobian[:3,   :]   = body_J
		jacobian[3:5, :2]   = hind_J
		jacobian[5: , 2:]   = front_J

		theta_double_dot, self.old_torques = self.dynamics.ForwardDynamics(forces, jacobian, self.state)


		body_acc = theta_double_dot[:2, 0]
		body_theta_double_dot = theta_double_dot[2, 0]
		joint_theta_double_dot = theta_double_dot[3:, 0]

		self.state = self.state.UpdateUsingJointThetaDoubleDot(joint_theta_double_dot)
		self.state = self.state.UpdateUsingAcceleration(body_acc)
		self.state = self.state.UpdateUsingBodyThetaDoubleDot(body_theta_double_dot)


		hind_leg_theta = self.state.joint_theta[:2]
		front_leg_theta = self.state.joint_theta[2:]

		self.leg_hind.SetAngles(hind_leg_theta)
		self.leg_front.SetAngles(front_leg_theta)

		print(self.state)



	def Render(self, screen, PPM):
		self.torso.Render(screen, PPM)
		
		self.thigh_front.Render(screen, PPM)
		self.thigh_hind.Render(screen, PPM)

		self.shin_front.Render(screen, PPM)
		self.shin_hind.Render(screen, PPM)