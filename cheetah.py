import numpy as np
import pygame
from utils import *

class Cheetah:
	def __init__(self, position = (0, 0), rotation = 0):
		self.points =   np.array([
									[-50, -20],
									[ 50, -20],
									[ 50,  20],
									[-50,  20]
								]).T

		R = GetRotationMatrix(rotation)
		self.points = R@self.points + np.array([[position[0], position[1]]]).T

		self.color = (0, 0, 255)

	def Render(self, screen):
		pygame.draw.lines(screen, self.color, True, self.points.T)
