from .kinematics import LegKinematics
from .jointController import JointController

class Leg:
	def __init__(self, thigh_joint, shin_joint, leg_segment_length):
		self.thigh_joint = thigh_joint
		self.shin_joint = shin_joint

		self.thigh_joint_controller = JointController(self.thigh_joint)
		self.shin_joint_controller = JointController(self.shin_joint)

		self.leg_kine_model = LegKinematics(leg_segment_length, leg_segment_length)

	def MoveTo(self, position):
		theta_1, theta_2 = self.leg_kine_model.IK(position)
		self.thigh_joint_controller.SetAngle(theta_1)
		self.shin_joint_controller.SetAngle(theta_2)