"""
input range?
k needs to be put into thrust equation
"""
from vector import Vector
from quaternion import Quaternion, QuaternionCalculation
import numpy as np

class Propellor:

    def __init__(self, name) -> None:
        self.name = name
        self.speed = 0
        self.thrust = 0

    def set_speed(self, speed) -> None:
        self.speed = speed

    # implement current input
    def calculate_thrust(self, k = 1) -> float:
        self.thrust = self.speed * self.speed * k
        return self.thrust

    # method for setting speed with current parameters
    def set_current_flow(self) -> float:
        pass

# how to save the rotations of drone? 
class Drone:    

    def __init__(self, gravity = 9.81, arm_length = 1, mass = 1, inertia = np.array([0.1,0.1,0.1]) ) -> None:
        self.propellors = [Propellor("One"), Propellor("Two"), Propellor("Three"), Propellor("Four")]    
        """drone param"""
        self.gravity = gravity
        empty_list = [0, 0, 0]
        self.positionVector = np.array(empty_list)
        self.velocityVector = np.array(empty_list) # Unit is m/s
        self.yMotorAxisVector = np.array([0, 1, 0])
        self.xMotorAxisVector = np.array([1, 0, 0])
        """ nach drone param auslegen """
        self.inertia = inertia
        L = arm_length
        self.x1 = (-L, 0, -0.1 + self.positionVector[2])
        self.x2 =  (L, 0, -0.1 + self.positionVector[2])
        self.x3 = (L, 0, 0.1 + self.positionVector[2])
        self.x4 = (-L, 0, 0.1 + self.positionVector[2])
        self.vertices_xAxis = [[self.x1, self.x2, self.x3, self.x4]]
        self.y1 = (0, -L, -0.1 + self.positionVector[2])
        self.y2 = (0, L, -0.1 + self.positionVector[2])
        self.y3 = (0, L, 0.1 + self.positionVector[2])
        self.y4 = (0, -L, 0.1 + self.positionVector[2])
        self.vertices_yAxis = [[self.y1, self.y2, self.y3, self.y4]]

        """drone param"""
        self.arm_length = arm_length # unit is in m
        """normierung nicht vergessen von quaternion"""
        self.rotationQuaternion = Quaternion(1, 0, 0, 0) # w : cos(0°) = 1, i,j,k: sin(0°)=0
        self.angularVelocity = Quaternion(1,0,0,0)
        """drone param"""
        self.mass = mass

    def updatePosition(self, traveled_distance: np.array) -> None:
        #self.positionVector = traveled_distance

        if self.positionVector[2] < 0 or traveled_distance[2] < 0:
            self.positionVector[2] = 0
            self.update_vertices()
            self.velocityVector = np.array([0,0,0], dtype=np.float64)
            #self.set_motor_speed(0,0,0,0)
        else:
            self.positionVector = traveled_distance
            self.update_vertices()

    def update_velocity(self, vector: np.array):
        self.velocityVector = vector

    # methods sets motor speed
    def set_motor_speed(self, speed1: int, speed2: int, speed3: int, speed4: int):
        self.propellors[0].set_speed(speed1)
        self.propellors[1].set_speed(speed2)
        self.propellors[2].set_speed(speed3)
        self.propellors[3].set_speed(speed4)

    # calculating thrust in z-direction in the body frame
    def calculate_thrust(self) -> np.array:
        thrust = 0
        for i in range(0, len(self.propellors)):
            thrust = thrust + self.propellors[i].calculate_thrust()

        return np.array([0, 0, thrust])

    def update_rotation(self, arr: np.array):
        self.rotationQuaternion = Quaternion.get_from_numpy_arr(Quaternion, arr)

    def update_angular_velocity(self, arr: np.array):
        self.angularVelocity = Quaternion.get_from_numpy_arr(Quaternion, arr)

    """Constan b ist auch für torque"""
    # constant b needs to be implemented here for torque z
    def caculate_body_frame_torque(self) -> np.array:
        torque_x = self.arm_length * (self.propellors[0].calculate_thrust() - self.propellors[2].calculate_thrust())
        torque_y = self.arm_length * (self.propellors[1].calculate_thrust() - self.propellors[3].calculate_thrust())
        torque_z = self.arm_length * (self.propellors[0].calculate_thrust() - self.propellors[1].calculate_thrust() + self.propellors[2].calculate_thrust() - self.propellors[3].calculate_thrust())
        
        return np.array([torque_x, torque_y, torque_z]) / self.inertia

    def update_vertices(self):
        x1 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.x1).get_imaginary_part_as_vector()
        x1 = self.positionVector + x1
        x1 = tuple(x1.tolist())
        x2 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.x2).get_imaginary_part_as_vector()
        x2 = self.positionVector + x2
        x2 = tuple(x2.tolist())
        x3 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.x3).get_imaginary_part_as_vector()
        x3 = self.positionVector + x3
        x3 = tuple(x3.tolist())
        x4 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.x4).get_imaginary_part_as_vector()
        x4 = self.positionVector + x4
        x4 = tuple(x4.tolist())

        y1 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.y1).get_imaginary_part_as_vector()
        y1 = self.positionVector + y1
        y1 = tuple(y1.tolist())
        y2 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.y2).get_imaginary_part_as_vector()
        y2 = self.positionVector + y2
        y2 = tuple(y2.tolist())
        y3 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.y3).get_imaginary_part_as_vector()
        y3 = self.positionVector + y3
        y3 = tuple(y3.tolist())
        y4 = QuaternionCalculation.calculate_rotation_from_given_quaternion(self.rotationQuaternion, self.y4).get_imaginary_part_as_vector()
        y4 = self.positionVector + y4
        y4 = tuple(y4.tolist())

        self.vertices_xAxis = [[x1, x2, x3, x4]]
        self.vertices_yAxis = [[y1, y2, y3, y4]]