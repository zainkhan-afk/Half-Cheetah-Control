from link import Link
import numpy as np


class FloatingBase(Link):
	def __init__(self, mass, length, height):
		super.__init__(mass, length, height)