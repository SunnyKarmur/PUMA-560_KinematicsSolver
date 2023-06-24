"""

Sunny Karmur PUMA_560_KINEMATICS_MOTION_PLANNER


"""

import math
import numpy as np
from PUMA_560_Source.utils import unit_vector

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0


def t_from_T(matrix):
    """Return translation vector from transformation matrix.
    """
    return np.array(matrix, copy=False)[:3, 3].copy()


def rotation_matrix(angle, direction):
    """Return matrix to rotate about axis defined by point and direction.
    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.diag([cosa, cosa, cosa])
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array([[0.0,  -direction[2],  direction[1]],
                      [direction[2],  0.0,  -direction[0]],
                      [-direction[1],  direction[0],  0.0]])
    return R


def get_transform(R, t):
    """Return matrix to rotate about axis defined by point and direction.
    """
    T = np.identity(4)
    T[:3, :3] = R
    t = np.array(t[:3], dtype=np.float64, copy=False)
    T[:3, 3] = t
    return T


def quaternion_slerp(quat0, quat1, levels=5):
    """Return spherical linear interpolation between two quaternions.
    """
    q0 = unit_vector(quat0[:4])
    q1 = unit_vector(quat1[:4])
    d = np.dot(q0, q1)
    angle = math.acos(d)
    if abs(angle) < _EPS:
        return q0

    all_q = [q0] # the first one

    # your code: find all intermediate quaternions

    mu = np.linspace(0, 1, levels)

    for i in range(1, levels-1):

        qi = np.sin((1-mu[i])*angle)*q0/(np.sin(angle)) + np.sin(mu[i]*angle)/(np.sin(angle))*q1
        all_q.append(qi)

    all_q.append(q1)  # the last one
    return all_q


def quaternion_about_axis(angle, axis):
    """Return quaternion for rotation about axis.
    """
    q = np.array([0.0, axis[0], axis[1], axis[2]])
    qlen = np.linalg.norm(q)
    if qlen > _EPS:
        q *= math.sin(angle/2.0) / qlen
    q[0] = math.cos(angle/2.0)
    return q


def R_from_quaternion(quaternion):
    """Return homogeneous rotation matrix from quaternion.
    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)

    if n < _EPS:
        R = np.identity(3)
    else:
        R = np.array([[1-2*q[1]**2-2*q[2]**2, 2*(q[0]*q[1]-q[2]*q[3]), 2*(q[0]*q[2]+q[1]*q[3])], [2*(q[0]*q[1]+q[2]*q[3]), 1-(2*q[0]**2)-(2*q[2]**2), 2*(q[1]*q[2] - q[0]*q[3])], [2*(q[0]*q[2] - q[1]*q[3]), 2*(q[1]*q[2] + q[0]*q[3]), 1-(2*q[0]**2)-(2*q[1]**2)]])
    return R


def quaternion_from_R(matrix):
    """Return quaternion from rotation matrix.
    """
    M = np.array(matrix, dtype=np.float64, copy=False)
    m00 = M[0, 0]
    m01 = M[0, 1]
    m02 = M[0, 2]
    m10 = M[1, 0]
    m11 = M[1, 1]
    m12 = M[1, 2]
    m20 = M[2, 0]
    m21 = M[2, 1]
    m22 = M[2, 2]

    # your code: perform the calculation for q

    s = math.sqrt(m00+m11+m22+1)*2
    q4 = s/4
    q1 = (m21 - m12) / s
    q2 = (m02 - m20) / s
    q3 = (m10 - m01) / s

    q_mod = math.sqrt(q1**2 + q2**2 + q3**2 + q4**2)

    q1 = q1/q_mod
    q2 = q2 / q_mod
    q3 = q3 / q_mod
    q4 = q4 / q_mod

    q = np.array([q1, q2, q3, q4])

    # negativity check
    if q[0] < 0.0:
        np.negative(q, q)
    return q


def compund_transform(*matrices):
    """Return concatenation of series of transformation matrices.
    """
    T = np.identity(4)
    for M in matrices:
        T = np.dot(T, M)
    return T
