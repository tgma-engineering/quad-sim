"""
input range?
k needs to be put into thrust equation
"""

from vector import Vector

class Propellor:

    def __init__(self, name) -> None:
        self.name = name
        self.speed = 0
        self.thrust = 0

    def set_speed(self, speed) -> None:
        self.speed = speed

    # implement current input
    def calculate_thrust(self, k = 1) -> float:
        return self.speed * self.speed * k

# how to save the rotations of drone? 
class Drone:    
    
    def __init__(self, gravity = 9.81, propellor_distance_from_center = 1) -> None:
        self.propellors = [Propellor("One"), Propellor("Two"), Propellor("Three"), Propellor("Four")]    
        self.gravity = gravity
        self.propellor_distance_from_center = propellor_distance_from_center
        self.positionVector = Vector(0,0,0)

    # calculating thrust in z-direction in the body frame
    def calculate_thrust(self) -> Vector:
        thrust = 0
        for i in range(0, len(self.propellors)):
            thrust = thrust + self.propellors[i].calculate_thrust()

        return Vector(0, 0, thrust)

    # constant b needs to be implemented here for torque z
    def caculate_body_frame_torque(self) -> Vector:
        torque_x = self.propellor_distance_from_center * (self.propellors[0].calculate_thrust() - self.propellors[2].calculate_thrust()) 
        torque_y = self.propellor_distance_from_center * (self.propellors[1].calculate_thrust() - self.propellors[3].calculate_thrust()) 
        torque_z = self.propellor_distance_from_center * (self.propellors[0].calculate_thrust() - self.propellors[1].calculate_thrust() + self.propellors[2].calculate_thrust() - self.propellors[3].calculate_thrust()) 
        
        return Vector(torque_x, torque_y, torque_z)

    def calculate_accleration():
        pass

    def calculate_angular_accleration():
        pass