import sys

from simulation import Simulation
from cheetah import Cheetah
from ground import Ground



PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
cheetah = Cheetah(sim, position = (2.5, 5), angle  = 0)
ground = Ground(sim)

sim.AddEntity(cheetah)
sim.AddEntity(ground)

while True:
	# ret = sim.Render()

	ret = sim.Step()
	if not ret:
		sys.exit()