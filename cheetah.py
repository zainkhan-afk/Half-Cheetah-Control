import numpy as np
import pygame
from utils import *

class Cheetah:
	def __init__(self, sim_handle, position = (0, 0), angle = 0):
		self.body = sim_handle.world.CreateDynamicBody(position=position, angle=angle)
		box = self.body.CreatePolygonFixture(box=(2, 1), density=1, friction=0.3)

		self.color = (200, 200, 255)
