import sys
import numpy as  np

from simulation import Simulation
from Robot import Cheetah
from ground import Ground

from utils import *
from Controller import PID 


PPM = 50.0  # pixels per meter
TARGET_FPS = 60
# TIME_STEP = 0.001
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
ground = Ground(sim)
cheetah = Cheetah(sim, ground, position = np.array([2.5, 3.0]), angle  = 0)

pid_controller = PID(cheetah.dynamicsModel, cheetah.leg_hind_pos, cheetah.leg_front_pos, P = 250, I = 1, D = 10)

sim.AddEntity(cheetah)
sim.AddEntity(ground)

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
x = 2.22
while True:
	cheetah.UpdateState()
	current_state = cheetah.GetState()
	J = cheetah.GetJacobian()

	current_pos = current_state.position
	current_body_theta = current_state.body_theta
	goal_pos = np.array([x + 0.05*np.cos(ang), y + 0.05*np.sin(ang)])
	goal_body_theta = current_body_theta

	new_state = pid_controller.Solve(current_state, J, current_pos, current_body_theta, goal_pos, goal_body_theta)
	cheetah.ApplyState(new_state)

	ret = sim.Step()
	ang += 0.0025
	if not ret:
		sys.exit()