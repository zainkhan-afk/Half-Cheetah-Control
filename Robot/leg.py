from .kinematics import LegKinematics
from .jointController import JointController
from utils import *

class Leg:
	def __init__(self, thigh_joint, shin_joint, leg_segment_length):
		self.thigh_joint_controller = JointController(thigh_joint, name = "thigh joint")
		self.shin_joint_controller = JointController(shin_joint, name = "shin joint")

		self.leg_kine_model = LegKinematics(leg_segment_length, leg_segment_length)

	def GetAngles(self):
		return self.thigh_joint_controller.GetAngle(), self.shin_joint_controller.GetAngle()

	def GetJacobian(self, theta_1, theta_2):
		return self.leg_kine_model.GetJacobian(theta_1, theta_2)

	def GetEEFKPosition(self):
		return self.leg_kine_model.FK(self.thigh_joint_controller.GetAngle(), self.shin_joint_controller.GetAngle())

	def SetAngles(self, angles):
		self.thigh_joint_controller.SetAngle(angles[0])
		self.shin_joint_controller.SetAngle(angles[1])

	def MoveTo(self, position):
		theta_1, theta_2 = self.leg_kine_model.IK(position)
		self.thigh_joint_controller.SetAngle(theta_1)
		self.shin_joint_controller.SetAngle(theta_2)

	# def MoveToAcc(self, acc):
	# 	speed_1 = self.thigh_joint.speed + acc[0]*TIME_STEP
	# 	speed_2 = self.shin_joint.speed  + acc[1]*TIME_STEP

	# 	theta_1 = self.thigh_joint.angle + speed_1*TIME_STEP
	# 	theta_2 = self.shin_joint.angle  + speed_2*TIME_STEP

	# 	# theta_1, theta_2 = self.leg_kine_model.IK(position)
	# 	self.thigh_joint_controller.SetAngle(theta_1)
	# 	self.shin_joint_controller.SetAngle(theta_2)