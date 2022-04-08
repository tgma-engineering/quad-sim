"""
Author: Marvin Ahlborn

Simple Optimal Control and Optimal Estimation functions
for linear, time-invariant Systems.

References:
'Optimal Control Theory: An Introduction' by Donald E. Kirk
'Applied Optimal Estimation' by Arthur Gelb
"""

import numpy as np

def stbl(A):
    """
    A is time-invariant (numpy) matrix of system x_dot = Ax.
    Returns boolean stating if system is stable.
    """
    eigs = np.linalg.eigvals(A)
    return np.real(eigs).max() < 0.0

def ctrb(A, B):
    """
    A and B are time-invariant (numpy) matricies of system x_dot = Ax + Bu.
    Returns boolean stating if system is controllable.
    """
    n = len(A)
    E = B.copy()

    for i in range(1, n):
        D = np.linalg.matrix_power(A, i)
        E = np.concatenate((E, D @ B), axis=1)
    
    return np.linalg.matrix_rank(E) == n

def obsv(F, H):
    """
    F and H are time-invariant (numpy) matricies of system
    x_dot = Fx,
    z = Hx.
    Returns boolean stating if system is observable.
    """
    n = len(F)
    O = H.T.copy()

    for i in range(1, n):
        N = np.linalg.matrix_power(F.T, i)
        O = np.concatenate((O, N @ H.T), axis=1)
    
    return np.linalg.matrix_rank(O) == n

def lqr(A, B, Q, R, dt=1e-2, eps=1e-3, max_steps=100000):
    """
    Computes control law F of Linear Quadratic Regulator u = Fx.
    A and B are time-invariant (numpy) matricies of system x_dot = Ax + Bu.
    LQR minimizes cost function 0.5 * (x.T @ Q @ x + u.T @ R @ u).
    Q is a time-invariant, symmetric and positive semi-definite (numpy) matrix,
    R is a time-invariant, symmetric and positive definite (numpy) matrix.
    """
    n = len(A)
    R_inv = np.linalg.inv(R)
    K = np.zeros((n, n), dtype=np.float32)

    for _ in range(max_steps):
        # The Ricatti Equation
        dK = -K @ A - A.T @ K - Q + K @ B @ R_inv @ B.T @ K
        K -= dK * dt  # Integration goes backwards

        # Integrate until steady state is reached
        if np.abs(dK).max() < eps:
            return -R_inv @ B.T @ K
    
    return None  # Solution doesn't converge in time

def kalman(F, H, G, Q, R, dt=1e-2, eps=1e-3, max_steps=100000):
    """
    Computes Gain Matrix K of Kalman Filter
    x_est_dot = F @ x_est + L @ u + K @ (z - H @ x_est)
    with x_est being the state estimate, control u and measurement z of
    the time-invariant system
    x_dot = Fx + Lu + Gw,
    z = Hx + v
    with the uncorrelated Gaussian White Noise Processes
    w ~ N(0, Q),
    v ~ N(0, R)
    where Q and R are the noise's Spectral Densities.
    """
    n = len(A)
    R_inv = np.linalg.inv(R)
    P = np.zeros((n, n), dtype=np.float32)

    for _ in range(max_steps):
        # The Ricatti Equation
        dP = F @ P + P @ F.T + G @ Q @ G.T - P @ H.T @ R_inv @ H @ P
        P += dP * dt

        # Integrate until steady state is reached
        if np.abs(dP).max() < eps:
            return P @ H.T @ R_inv
        
    return None  # Solution doesn't converge in time


"""
Control Example:
"""
if __name__ == '__main__':
    A = np.array([[0, 1], [2, -1]], dtype=np.float32)
    B = np.array([[0], [1]], dtype=np.float32)
    Q = np.array([[2, 0], [0, 1]], dtype=np.float32)
    R = np.array([[0.5]], dtype=np.float32)

    print(stbl(A))
    print(ctrb(A, B))
    print(lqr(A, B, Q, R))