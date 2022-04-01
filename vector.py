""""
Class for making use of vectors
"""
from cmath import sqrt
from random import random, seed
from secrets import randbelow

class Vector:

    def __init__(self, x, y, z) -> None:
        self.x = x
        self.y = y
        self.z = z

    # Norms vector
    def __norm__(self) -> int:
        return sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    # Returns normed vector
    def get_normed_vector(self):
        norm = self.__norm__()
        return Vector(self.x / norm, self.y / norm, self.z / norm)

    # Returns vector values as list
    def get_params_as_list(self):
        return [self.x, self.y, self.z]

    # method returns random vector
    def get_rando_vector(self, random_range: int = 1):
        seed()
        x = random() * random_range
        y = random() * random_range
        z = random() * random_range
        return Vector(x,y,z)