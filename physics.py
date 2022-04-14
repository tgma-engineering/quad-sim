import numpy as np

from drone import Drone
from quaternion import Quaternion
from quaternion import QuaternionCalculation

class Physics:

    def __init__(self, drone: Drone, drag: float = 0.1):
        self.drone = drone
        self.drag = drag

    def calculatePosition(self, delta_time: float):
        
        x = np.concatenate((self.drone.positionVector, self.drone.velocityVector, self.drone.rotationQuaternion.get_as_numpy_arr(), self.drone.angularVelocity.get_as_numpy_arr()))
        new_postion_velocity_vector = Physics.rk4(self.x_dot, x, 0, delta_time)
        """bis hierhin ist alles korrekt"""

        distance_added = new_postion_velocity_vector[:3]
        velocity_added = new_postion_velocity_vector[3:6]

        self.drone.update_rotation(new_postion_velocity_vector[6:10])
        self.drone.update_angular_velocity(new_postion_velocity_vector[10:])

        self.drone.update_velocity(velocity_added)
        self.drone.updatePosition(distance_added)

    def x_dot(self, x, t):
        position = x[:3]
        velocity = x[3:6]
        rotation = x[6:10]
        angular_velocity = x[10:]

        x_dot = np.zeros(14, dtype=np.float64)
        x_dot[:3] = velocity
        x_dot[3:6] = self.__calculate_accleration__()
        sqrt = np.sqrt(x[11] * x[11] + x[12] * x[12] + x[13] * x[13])
        if not sqrt == 0:
            v1 = x[11] / sqrt
            v2 = x[12] / sqrt
            v3 = x[13] / sqrt
        else:
            v1, v2, v3 = 0,0,0
        alpha = 2 * np.arctan2(sqrt, x[10])
        x_dot[6] = -v1 * alpha * x[7] * 0.5 - v2 * alpha * x[8] * 0.5 - v3 * alpha * x[9] * 0.5
        x_dot[7] = v1 * alpha * x[6] * 0.5 + v2 * alpha * x[9] * 0.5 - v3 * alpha * x[8] * 0.5
        x_dot[8] = -v1 * alpha * x[9] * 0.5 + v2 * alpha * x[6] * 0.5 + v3 * alpha * x[7] * 0.5
        x_dot[9] = v1 * alpha * x[8] * 0.5 - v2 * alpha * x[7] * 0.5 + v3 * alpha * x[6] * 0.5

        # Rotation auf den Torque muss noch angwendet werden
        angular_accleration = Physics.__calculate_angular_acceleration__(self.drone.caculate_body_frame_torque(), self.drone.inertia)
        angular_accleration_norm = np.linalg.norm(angular_accleration)
        angular_accleration_quaternion = Quaternion.get_quaternion_from_angle(Quaternion, angular_accleration_norm, angular_accleration)

        angular_accleration = QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, Quaternion(x[6], x[7], x[8], x[9]), angular_accleration_quaternion)
        angular_accleration = QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, angular_accleration, Quaternion(x[6], x[7], x[8], x[9]).get_conjugate())
        angular_accleration_quaternion = angular_accleration

        sqrt = np.sqrt(angular_accleration_quaternion.x**2 + angular_accleration_quaternion.y**2 + angular_accleration_quaternion.z**2)
        if not sqrt == 0:
            v1 = angular_accleration_quaternion.x / sqrt
            v2 = angular_accleration_quaternion.y / sqrt
            v3 = angular_accleration_quaternion.z / sqrt
        else:
            v1, v2, v3 = 0,0,0

        alpha = 2 * np.arctan2(sqrt, angular_accleration_quaternion.w)
        x_dot[10] = -v1 * alpha * x[11] * 0.5 - v2 * alpha * x[12] * 0.5 - v3 * alpha * x[13] * 0.5
        x_dot[11] = v1 * alpha * x[10] * 0.5 + v2 * alpha * x[13] * 0.5 - v3 * alpha * x[12] * 0.5
        x_dot[12] = -v1 * alpha * x[13] * 0.5 + v2 * alpha * x[10] * 0.5 + v3 * alpha * x[11] * 0.5
        x_dot[13] = v1 * alpha * x[12] * 0.5 - v2 * alpha * x[11] * 0.5 + v3 * alpha * x[10] * 0.5

        return x_dot

    def __calculate_accleration__(self) -> np.array:
        thrustVector = self.drone.calculate_thrust()
        # real orientation of thrust vector
        rotationQuaternion = self.drone.rotationQuaternion
        thrustVectorRotated = QuaternionCalculation.calculate_rotation_from_given_quaternion(rotationQuaternion, thrustVector).get_imaginary_part_as_vector()
        # inverse of drone mass
        mass_inverse = 1 / self.drone.mass
        thrustVector = mass_inverse * thrustVectorRotated
        """noch als param auslagern"""
        gravityVector = np.array([0, 0, -self.drone.gravity])
        # Same drag for x,y,z
        dragVector = -1 * self.drag

        accleration_vector = thrustVector + gravityVector + dragVector
        return accleration_vector

    def __calculate_distance_from_velocity_vector__(velocity: np.array, delta_time: float):
        distance = velocity * delta_time

        return distance

    def __calculate_velocity_from_accleration_vector__(accleration: np.array, delta_time: float):
        velocity = accleration * delta_time

        return velocity

    def __calculate_angular_acceleration__(torque: np.array, inertia: np.array):
        return torque / inertia

    """
    f: funktion
    x: punkt an dem ich gerade bin
    t: momentane Zeit
    dt: deltatime
    """
    # Classical Runge-Kutta Integration
    # x durch thrust etc geändert
    def rk4(f, x, t, dt: float):
        k1 = f(x, t)
        k2 = f(x + dt/2 * k1, t + dt/2)
        k3 = f(x + dt/2 * k2, t + dt/2)
        k4 = f(x + dt * k3, t + dt)
        return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)