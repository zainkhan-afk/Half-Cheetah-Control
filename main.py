import sys
import numpy as  np

from simulation import Simulation
from Robot import Cheetah
from ground import Ground

from utils import *



PPM = 50.0  # pixels per meter
TARGET_FPS = 60
# TIME_STEP = 0.001
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
cheetah = Cheetah(sim, position = (2.5, 3.0), angle  = 0)
ground = Ground(sim)

sim.AddEntity(cheetah)
sim.AddEntity(ground)

ang = 0


robot_placed = False
while not robot_placed:
	robot_placed = cheetah.Rest()
	ret = sim.Step()
	if not ret:
		sys.exit()

while True:
	cheetah.StandUp()
	# cheetah.body_angle = np.pi/36*np.sin(ang)


	ret = sim.Step()
	if not ret:
		sys.exit()