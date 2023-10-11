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

pid_controller = PID(cheetah.dynamicsModel, cheetah.leg_hind_pos, cheetah.leg_front_pos, P = 250, I = 1, D = 10)


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
y = 2.41
x = 2.51
while True:
	cheetah.UpdateState()
	current_state = cheetah.GetState()
	J = cheetah.GetJacobian()

	current_pos = current_state.position
	# print(current_pos)
	current_body_theta = current_state.body_theta
	goal_pos = np.array([x + 0.05*np.cos(ang), y + 0.05*np.sin(ang)])
	goal_body_theta = current_body_theta
	goal_pos = current_pos

	print(current_pos)

	new_state = pid_controller.Solve(current_state, J, current_pos, current_body_theta, goal_pos, goal_body_theta)
	cheetah.ApplyState(new_state)

	ret = sim.Step()
	ang += 0.0025
	if not ret:
		sys.exit()