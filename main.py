import sys
import numpy as  np

from simulation import Simulation
from Robot import Cheetah
from ground import Ground

from utils import *
from Controller import PID 
from trajectory import Trajectory

PPM = 50.0  # pixels per meter
TARGET_FPS = 60
# TIME_STEP = 0.001
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
ground = Ground(sim)
cheetah = Cheetah(sim, ground, position = np.array([1, 0.8]), angle  = 0)
traj = Trajectory(size = 500)

pid_controller = PID(cheetah.dynamicsModel, cheetah.leg_hind_pos, cheetah.leg_front_pos, P = 500, I = 0.001, D = 10)


num_pts = 500
for i in range(num_pts):
	point = np.array([1 + i/num_pts * 5, 0.55])
	traj.AddPoint(point)

sim.AddEntity(cheetah)
sim.AddEntity(ground)
sim.AddEntity(traj)

traj.MakePathRenderPts(sim.renderer.screen, sim.PPM)

ang = 0

robot_placed = False


print("Going to Home Position")
while not robot_placed:
	robot_placed = cheetah.Rest()
	ret = sim.Step()
	if not ret:
		sys.exit()

print("Reached Rest Position")
cheetah.UpdateState()
cheetah.UpdateState()
cheetah.UpdateState()
print("State Updated")

ang = 0
new_state = None
x = 0.87
y = 0.53007615

# x = 1.0
# y = 0.8
t = 0
while True:
	cheetah.UpdateState()

	if new_state is not None:
		error_state = cheetah.CalculateStateError(new_state)
		# error_state = current_state - new_state
		print("Difference between the predicted state and the actual state")
		print(error_state)

	# cheetah.Walk(t)
	current_state = cheetah.GetState()
	J = cheetah.GetJacobian()

	current_pos = current_state.position
	current_body_theta = current_state.body_theta
	goal_pos = np.array([x + 0.1*np.cos(ang), y + 0.1*np.sin(ang)])
	# goal_pos = np.array([x, y])
	goal_body_theta = current_body_theta
	# goal_body_theta = np.pi/18*np.sin(ang)

	print()
	print("Current State")
	print(current_state)
	print("New State")
	print(new_state)


	new_state = pid_controller.Solve(current_state, J, current_pos, current_body_theta, goal_pos, goal_body_theta)
	cheetah.ApplyState(new_state)

	ret = sim.Step()
	ang += 0.0025
	t += TIME_STEP
	if not ret:
		sys.exit()