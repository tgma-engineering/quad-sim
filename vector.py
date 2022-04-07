""""
Class for making use of vectors
"""
from cmath import sqrt
from random import random, seed
from secrets import randbelow
from typing import List

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
        if norm != 0:
            return Vector(self.x / norm, self.y / norm, self.z / norm)
        else:
            return Vector(0, 0, 0)

    # Returns vector values as list
    def get_params_as_list(self):
        return [self.x, self.y, self.z]

    # Returns random vector with int values
    def get_random_vector(self, random_range: int = 1):
        seed()
        x = int(random() * random_range)
        y = int(random() * random_range)
        z = int(random() * random_range)
        v = Vector(x,y,z)
        return v

    def multiply_vector(self, coefficent: float):
        x = self.x * coefficent
        y = self.y * coefficent
        z = self.z * coefficent
        return Vector(x, y, z)

    def add_vector_list(self, vectors: List):
        x,y,z = 0
        for i in range(0, len(vectors)):
            x = vectors[i].x 
        
        return Vector(x,y,z)