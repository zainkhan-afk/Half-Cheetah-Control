import sys

from simulation import Simulation
from cheetah import Cheetah

cheetah = Cheetah(position = (100, 100), rotation  = 0)
sim = Simulation()

sim.AddEntity(cheetah)

while True:
	ret = sim.Render()
	if not ret:
		sys.exit()

	sim.step()