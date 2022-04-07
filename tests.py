"""
Class for testing correct implementation of classes
"""
from random import random, seed
import numpy as np
import numpy.testing
from quaternion import Quaternion, QuaternionCalculation
from vector import Vector

#print("started")
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
def print_list(list, vector = False):
    for i in range(0, len(list)):
        print(list[i])

"""
v1 = Vector(1,0,0)
v2 = Vector(0,1,0)
v3 = Vector(0,0,1)

angle = 90
rotation_axis_x = Vector(1,0,0)
rotation_axis_y = Vector(0,1,0)
rotation_axis_z = Vector(0,0,1)

q1 = Quaternion(0,0,0.6,0.8)
q1.norm_quaternion()
#print_list(q1.get_params_as_list())
#print_list(q1.get_conjugate().get_params_as_list())
#print('-'*40)

#print_list(Quaternion.get_quaternion_from_angle(Quaternion, angle=90, vector=rotation_axis_x).get_params_as_list())
#print_list(Quaternion.get_quaternion_from_angle(Quaternion, angle=90, vector=rotation_axis_x).get_conjugate().get_params_as_list())

# testing for correct multiplication calculation
q2 = Quaternion(1,2,3,4)
q3 = Quaternion(4,88,6,7)

#print('-'*10)
#print_list(QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, q2, q3).get_params_as_list())
# multiplication is done correctly

# checking for correct rotation calculation
# rotation about x-axis
result = QuaternionCalculation.calculate_rotation(QuaternionCalculation, 90, v3, rotation_axis_x)
#print_list(result.get_params_as_list())

#print(QuaternionCalculation.quaternion_to_euler(QuaternionCalculation, result).get_params_as_list())


seed()


"""
"""
for i in range(0, 10):
    print('--'*50)
    angle = int(random() * 360)
    vector = Vector.get_random_vector(Vector, 10)
    rotation_axis_vector = Vector.get_random_vector(Vector, 10);
    rotation_axis_vector = Vector(9,5,3)
    rotated_quaternion = QuaternionCalculation.calculate_rotation(QuaternionCalculation, angle, vector, rotation_axis_vector)    
    print('(' + str(vector.x) + ', ' + str(vector.y) + ', ' + str(vector.z) + ')' + '  ', end='')
    print('(' + str(rotation_axis_vector.x) + ', ' + str(rotation_axis_vector.y) + ', ' + str(rotation_axis_vector.z) + ')' + '  angle=' + str(angle) + '°  ', end='')
    print('(' + str(rotated_quaternion.w) + ', ' + str(rotated_quaternion.x) + ', ' + str(rotated_quaternion.y) + ', ' + str(rotated_quaternion.z) + ')' + '  ')  

print('--'*50)
angle = 45
vector = Vector(0,0,1)
rotation_axis_vector = Vector(1,0,0)
rotated_quaternion = QuaternionCalculation.calculate_rotation(QuaternionCalculation, angle, vector, rotation_axis_vector)    
print('(' + str(vector.x) + ', ' + str(vector.y) + ', ' + str(vector.z) + ')' + '  ', end='')
print('(' + str(rotation_axis_vector.x) + ', ' + str(rotation_axis_vector.y) + ', ' + str(rotation_axis_vector.z) + ')' + '  angle=' + str(angle) + '°  ', end='')
print('(' + str(rotated_quaternion.w) + ', ' + str(rotated_quaternion.x) + ', ' + str(rotated_quaternion.y) + ', ' + str(rotated_quaternion.z) + ')' + '  ')  

qua = Quaternion(1,0,0,0)
angle = 45
vector = Vector(0,0,1)
rotation_axis_vector = Vector(1,0,0)
rotated_quaternion = QuaternionCalculation.calculate_rotation(QuaternionCalculation, angle, vector, rotation_axis_vector)    
rotated_quaternion = QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, rotated_quaternion, qua)
print('(' + str(vector.x) + ', ' + str(vector.y) + ', ' + str(vector.z) + ')' + '  ', end='')
print('(' + str(rotation_axis_vector.x) + ', ' + str(rotation_axis_vector.y) + ', ' + str(rotation_axis_vector.z) + ')' + '  angle=' + str(angle) + '°  ', end='')
print('(' + str(rotated_quaternion.w) + ', ' + str(rotated_quaternion.x) + ', ' + str(rotated_quaternion.y) + ', ' + str(rotated_quaternion.z) + ')' + '  ')  

"""
if __name__ == '__main__':
    vector = np.array([1,1,1])
    print(vector)
    q = Quaternion.get_quaternion_from_angle(Quaternion, 0.5, vector)
    print(q.get_params_as_list())