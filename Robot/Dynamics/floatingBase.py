from .link import Link
import numpy as np


class FloatingBase(Link):
	def __init__(self, link_object):
		super().__init__(link_object)