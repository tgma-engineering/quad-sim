"""
Class makes use of quaternions to rate a given vector
"""
import numpy as np
from math import atan2, cos, asin, sin, sqrt, pi
from random import random, seed
from vector import Vector

# Class for representing Quaternion
class Quaternion:

    # initializes vector  
    def __init__(self, w, x, y, z) -> None:
        self.w = w
        if self.w < 0.00001 and self.w > -0.00001:
            self.w = 0
        self.x = x
        self.y = y
        self.z = z

    # norms quaternion
    def norm_quaternion(self):
        norm = sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if norm != 0:
            self.w = self.w / norm
            self.x = self.x / norm
            self.y = self.y / norm
            self.z = self.z / norm

    # Method returns inverse Quaternion
    def get_conjugate(self): 
        w = self.w
        x = 0 if -self.x == 0 else -self.x
        y = 0 if -self.y == 0 else -self.y
        z = 0 if -self.z == 0 else -self.z
        return Quaternion(w, x, y, z)

    # get Quaternion from 3 dimensional vector and given angle
    def get_quaternion_from_angle(self, angle: float, vector: Vector, degree: bool = False):
        # convert degree to rad
        if degree:
            angle = angle % 360
            angle = pi * angle / 180

        half_angle = angle / 2

        normed_vector = vector.get_normed_vector()
        print(normed_vector.get_params_as_list())
        real_part = cos(half_angle)
        print('real part' + str(real_part))
        i_part = sin(half_angle) * normed_vector.x
        j_part = sin(half_angle) * normed_vector.y
        k_part = sin(half_angle) * normed_vector.z

        return Quaternion(real_part.real, i_part.real, j_part.real, k_part.real)

    # Method returns inverse directly from angle
    def get_conjugate_from_angle(self, angle: float):
        return Quaternion(cos(-angle), sin(-angle) * self.x, sin(-angle) * self.y, sin(-angle) * self.z)

    def get_params_as_list(self):
        return [self.w, self.x, self.y, self.z]

    def get_random_quaternion(self):
        seed()
        w = random() * 10
        x = random() * 10
        y = random() * 10
        z = random() * 10
        q = Quaternion(w,x,y,z)
        q.norm_quaternion()
        return q
        
    def get_imaginary_part_as_vector(self) -> Vector:
        return Vector(self.x, self.y, self.z)

class QuaternionCalculation:

    # Method returns quaternions multiplicated
    def __quaternion_multiplication__(self, quaternion1: Quaternion, quaternion2: Quaternion) -> Quaternion:
        real_part = quaternion1.w * quaternion2.w - quaternion1.x * quaternion2.x - quaternion1.y * quaternion2.y - quaternion1.z * quaternion2.z
        i_part = quaternion1.w * quaternion2.x + quaternion1.x * quaternion2.w + quaternion1.y * quaternion2.z - quaternion1.z * quaternion2.y
        j_part = quaternion1.w * quaternion2.y + quaternion1.y * quaternion2.w + quaternion1.z * quaternion2.x - quaternion1.x * quaternion2.z
        k_part = quaternion1.w * quaternion2.z + quaternion1.z * quaternion2.w + quaternion1.x * quaternion2.y - quaternion1.y * quaternion2.x

        return Quaternion(real_part, i_part, j_part, k_part)

    # Calculates and returns rotation around axis vector 
    def calculate_rotation(self, angle: float, vector: Vector, rotation_axis_vector: Vector) -> Quaternion:
        quaternion_vector = Quaternion(0, vector.x, vector.y, vector.z)
        quaternion_rotation_axis = Quaternion.get_quaternion_from_angle(Quaternion, angle, rotation_axis_vector, True)
        quaternion_rotation_axis_inverse = Quaternion.get_conjugate(quaternion_rotation_axis)

        print("Vector " + str(quaternion_vector.get_params_as_list()))
        print("Rotation Axis " + str(quaternion_rotation_axis.get_params_as_list()))
        print("Rotation Axis Inverse " + str(quaternion_rotation_axis_inverse.get_params_as_list()))

        q1 = QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation ,quaternion_rotation_axis, quaternion_vector)
        return QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, q1, quaternion_rotation_axis_inverse)
        
    def calculate_rotation_from_given_quaternion(rotation_quaternion: Quaternion, vector: Vector) -> Quaternion:
        rotation_quaternion_inverse = Quaternion.get_conjugate(rotation_quaternion)
        quaternion_vector = Quaternion(0, vector.x, vector.y, vector.z)

        q1 = QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation ,rotation_quaternion, quaternion_vector)
        return QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, q1, rotation_quaternion_inverse)

    # transform euler to quaternion
    def euler_to_quaternion(self, phi, theta, psi) -> Quaternion:
        qw = cos(phi/2) *cos(theta/2) * cos(psi/2) + sin(phi/2) * sin(theta/2) * sin(psi/2)
        qx = sin(phi/2) *cos(theta/2) * cos(psi/2) - cos(phi/2) * sin(theta/2) * sin(psi/2)
        qy = cos(phi/2) *sin(theta/2) * cos(psi/2) + sin(phi/2) * cos(theta/2) * sin(psi/2)
        qz = cos(phi/2) *cos(theta/2) * sin(psi/2) - sin(phi/2) * sin(theta/2) * cos(psi/2)

        return Quaternion(qw, qx, qy, qz)

    # transformes quaternion to euler
    def quaternion_to_euler(self, quaternion: Quaternion) -> Vector:
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z

        t0 = 2 * (w * x + y * z)
        t1 = 1 - 2 * (x * x + y * y)
        X = atan2(t0, t1)

        t2 = 2 * (w * y - z * x)
        t2 = 1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        Y = asin(t2)

        t3 = 2 * (w * z + x * y)
        t4 = 1 - 2 * (y * y + z * z)
        Z = atan2(t3, t4)

        return Vector(X,Y,Z)
