"""
Class for testing correct implementation of classes
"""

from traceback import print_tb
from quaternion import Quaternion, QuaternionCalculation
from vector import Vector
from math import *

print("started")
xyz = ["w= ", "x = ", "y= ", "z= "]
"""
vector = Vector(1,0,0)
phi = pi/2
theta = pi/4
psi = pi/2
q = QuaternionCalculation.euler_to_quaternion(QuaternionCalculation,phi, theta, psi)
"""

"""
Testing Quaternion
"""
def print_list(list):
    for i in range(0, len(list)):
        print(list[i])

v1 = Vector(1,0,0)
v2 = Vector(0,1,0)
v3 = Vector(0,0,1)

angle = 90
rotation_axis_x = Vector(1,0,0)
rotation_axis_y = Vector(0,1,0)
rotation_axis_z = Vector(0,0,1)

q1 = Quaternion(0,0,0.6,0.8)
q1.norm_quaternion()
print_list(q1.get_params_as_list())
print_list(q1.get_conjugate().get_params_as_list())
print('-'*40)

print_list(Quaternion.get_quaternion_from_angle(Quaternion, angle=90, vector=rotation_axis_x).get_params_as_list())
print_list(Quaternion.get_quaternion_from_angle(Quaternion, angle=90, vector=rotation_axis_x).get_conjugate().get_params_as_list())

# testing for correct multiplication calculation
q2 = Quaternion(1,2,3,4)
q3 = Quaternion(4,88,6,7)

print('-'*10)
print_list(QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, q2, q3).get_params_as_list())
# multiplication is done correctly

# checking for correct rotation calculation 
# rotation about x-axis
result = QuaternionCalculation.calculate_rotation(QuaternionCalculation, 90, v3, rotation_axis_x)
print_list(result.get_params_as_list())

print(QuaternionCalculation.quaternion_to_euler(QuaternionCalculation, result).get_params_as_list())