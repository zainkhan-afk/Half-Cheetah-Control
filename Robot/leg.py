from .kinematics import Kinematics
from .jointController import JointController

class Leg:
	def __init__(self, thigh_joint, shin_joint, leg_segment_length):
		self.thigh_joint = thigh_joint
		self.shin_joint = shin_joint

		self.thigh_joint_controller = JointController(self.thigh_joint)
		self.shin_joint_controller = JointController(self.shin_joint)

		self.kine_model = Kinematics(leg_segment_length, leg_segment_length)

	def MoveTo(self, x, y):
		theta_1, theta_2 = self.kine_model.IK(x, y)
		self.thigh_joint_controller.SetAngle(theta_1)
		self.shin_joint_controller.SetAngle(theta_2)