import sys

from simulation import Simulation
from Robot import Cheetah
from ground import Ground



PPM = 50.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 0.01
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
cheetah = Cheetah(sim, position = (2.5, 2.5), angle  = 0)
ground = Ground(sim)

sim.AddEntity(cheetah)
sim.AddEntity(ground)

while True:
	# ret = sim.Render()

	cheetah.StandUp()
	ret = sim.Step()
	if not ret:
		sys.exit()