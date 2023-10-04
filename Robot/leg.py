from .kinematics import LegKinematics
from .jointController import JointController
from utils import *

class Leg:
	def __init__(self, thigh_joint, shin_joint, leg_segment_length):
		self.thigh_joint = thigh_joint
		self.shin_joint = shin_joint

		self.thigh_joint_controller = JointController(self.thigh_joint)
		self.shin_joint_controller = JointController(self.shin_joint)

		self.leg_kine_model = LegKinematics(leg_segment_length, leg_segment_length)

	def GetAngles(self):
		return self.thigh_joint.angle, self.shin_joint.angle

	def GetJacobian(self):
		return self.leg_kine_model.GetJacobian(self.thigh_joint.angle, self.shin_joint.angle)

	def GetEEFKPosition(self):
		return self.leg_kine_model.FK(self.thigh_joint.angle, self.shin_joint.angle)

	def MoveTo(self, position):
		theta_1, theta_2 = self.leg_kine_model.IK(position)
		self.thigh_joint_controller.SetAngle(theta_1)
		self.shin_joint_controller.SetAngle(theta_2)


	def MoveToAcc(self, acc):
		speed_1 = self.thigh_joint.speed + acc[0]*TIME_STEP
		speed_2 = self.shin_joint.speed  + acc[1]*TIME_STEP

		theta_1 = self.thigh_joint.angle + speed_1*TIME_STEP
		theta_2 = self.shin_joint.angle  + speed_2*TIME_STEP

		# theta_1, theta_2 = self.leg_kine_model.IK(position)
		self.thigh_joint_controller.SetAngle(theta_1)
		self.shin_joint_controller.SetAngle(theta_2)