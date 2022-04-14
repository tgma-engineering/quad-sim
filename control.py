import numpy as np
from quaternion import Quaternion
from quaternion import QuaternionCalculation
from lqr import stbl, ctrb, lqr


class Controller:
    def __init__(self):
        # Constants
        k = 1.
        m = 1.
        inertia = .1 * np.ones(3, dtype=np.float64)
        arm_length = 1.

        x_dot_const = lambda x, u : self.x_dot(x, u, k, m, inertia, arm_length)

        # Equilibrium point around which system dynamics are linearized
        x_eq = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0], dtype=np.float64)
        eq_thrust = np.sqrt(9.81 * m / (4 * k))
        self.u_eq = eq_thrust * np.ones(4, dtype=np.float64)

        jacobian_x = self.jacobian_x(x_dot_const, x_eq, self.u_eq)
        jacobian_u = self.jacobian_u(x_dot_const, x_eq, self.u_eq)

        # Remove real parts of quaternion rotation and angular velocity because its uncontrollable
        # and can be reconstructed from the other three parts and the fact that rotation quaternions have norm 1.

        A = np.empty((12, 12), dtype=np.float64)
        B = np.empty((12, 4), dtype=np.float64)

        A = np.delete(np.delete(jacobian_x, [6, 10], 0), [6, 10], 1)
        B = np.delete(jacobian_u, [6, 10], 0)

        Q = np.eye(12, dtype=np.float64)
        Q[0:3, 0:3] *= 0.5
        R = 0.1 * np.eye(4, dtype=np.float64)

        self.K = lqr(A, B, Q, R)  # LQR Gain Matrix
    
    def control(self, x, x_r):
        """
        x is the current system state, x_r is the desired reference state.
        """
        x_e = np.zeros(12, dtype=np.float64)  # Position error vector
        x_e[:6] = x[:6] - x_r[:6]  # Translation errors are simple differences

        # The quaternion error is the result of a quaternion multiplication
        rot_err = QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, Quaternion(x_r[6], x_r[7], x_r[8], x_r[9]), Quaternion(x[6], x[7], x[8], x[9]).get_conjugate())
        x_e[6:9] = rot_err.get_as_numpy_arr()[1:4]  # Controller only uses the Quaternions's Vector
        ang_vel_err = QuaternionCalculation.__quaternion_multiplication__(QuaternionCalculation, Quaternion(x_r[10], x_r[11], x_r[12], x_r[13]), Quaternion(x[10], x[11], x[12], x[13]).get_conjugate())
        x_e[9:12] = ang_vel_err.get_as_numpy_arr()[1:4]

        u = self.K @ x_e + self.u_eq
        return u


    def x_dot(self, x, u, k, m, inertia, arm_length):
        k = k
        m = m
        inertia = inertia
        arm_length = arm_length

        w1 = u[0]
        w2 = u[1]
        w3 = u[2]
        w4 = u[3]

        position = x[:3]
        velocity = x[3:6]
        rotation = x[6:10]
        angular_velocity = x[10:]

        x_dot = np.zeros(14, dtype=np.float64)
        x_dot[:3] = velocity


        thrustVector = k/m * np.array([0, 0, np.sum(u**2)], dtype=np.float64)
        # real orientation of thrust vector
        rotationQuaternion = Quaternion(x[6], x[7], x[8], x[9])
        thrustVectorRotated = QuaternionCalculation.calculate_rotation_from_given_quaternion(rotationQuaternion, thrustVector).get_imaginary_part_as_vector()
        gravityVector = np.array([0, 0, -9.81], dtype=np.float64)
        # Same drag for x,y,z
        dragVector = -1 * k * velocity

        accleration_vector = thrustVectorRotated + gravityVector + dragVector


        x_dot[3:6] = accleration_vector
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

        torque_x = arm_length * k * (w1**2 - w3**2)
        torque_y = arm_length * k * (w2**2 - w4**2)
        torque_z = arm_length * k * (w1**2 - w2**2 + w3**2 - w4**2)
        
        torqueVector = np.array([torque_x, torque_y, torque_z], np.float64)
        angular_accleration = torqueVector / inertia


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


    def jacobian_x(self, x_dot, x, u, eps=1e-4):
        """
        2-Point finite difference numerical differentiation.
        """
        n = len(x)
        jacobian = np.zeros((n, n), dtype=np.float64)

        for i in range(n):
            x_minus_eps = x.copy()
            x_minus_eps[i] -= eps
            x_plus_eps = x.copy()
            x_plus_eps[i] += eps

            x_dot_minus_eps = x_dot(x_minus_eps, u)
            x_dot_plus_eps = x_dot(x_plus_eps, u)

            dx = (x_dot_plus_eps - x_dot_minus_eps) / (2 * eps)
            jacobian[:, i] = dx

        return jacobian

    def jacobian_u(self, x_dot, x, u, eps=1e-4):
        """
        2-Point finite difference numerical differentiation.
        """
        n = len(x)
        m = len(u)
        jacobian = np.zeros((n, m), dtype=np.float64)

        for i in range(m):
            u_minus_eps = u.copy()
            u_minus_eps[i] -= eps
            u_plus_eps = u.copy()
            u_plus_eps[i] += eps

            x_dot_minus_eps = x_dot(x, u_minus_eps)
            x_dot_plus_eps = x_dot(x, u_plus_eps)

            dx = (x_dot_plus_eps - x_dot_minus_eps) / (2 * eps)
            jacobian[:, i] = dx

        return jacobian


# Usage example
if __name__ == '__main__':
    x = np.array([1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0], dtype=np.float64)  # Actual state, depends on simulation
    x_r = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0], dtype=np.float64)  # Desired state, this one is a good start

    ctrl = Controller()  # Init controller object
    actuation = ctrl.control(x, x_r)  # Returns array of optimal motor angular velocities

    print(actuation)