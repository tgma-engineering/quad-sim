""""
Class for making use of vectors
"""
from cmath import sqrt

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