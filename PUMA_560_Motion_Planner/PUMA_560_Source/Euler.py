"""

Sunny Karmur PUMA_560_KINEMATICS_MOTION_PLANNER


"""



import numpy as np


def Fixed_Angle(alpha, beta, gamma, order='xyz'):
    """
    input
        alpha, beta, gamma = rotation angles in rotation order (degrees)
    output
        3x3 rotation matrix (numpy array) based on Fixed-Angle (original static axis)
    """
    c_alpha = np.cos(np.radians(alpha))
    s_alpha = np.sin(np.radians(alpha))

    c_beta = np.cos(np.radians(beta))
    s_beta = np.sin(np.radians(beta))

    c_gamma = np.cos(np.radians(gamma))
    s_gamma = np.sin(np.radians(gamma))

    R_Z = np.array([[c_alpha, -s_alpha, 0],
                    [s_alpha, c_alpha, 0],
                    [0, 0, 1]]
                   )

    R_Y = np.array([[c_beta, 0, s_beta],
                    [0, 1, 0],
                    [-s_beta, 0, c_beta]]
                   )

    R_X = np.array([[1, 0, 0],
                    [0, c_gamma, -s_gamma],
                    [0, s_gamma, c_gamma]]
                   )

    if order == 'xyz':
        matrix = R_Z @ R_Y @ R_X
    elif order == 'zyx':
        matrix = R_X @ R_Y @ R_Z
    elif order == 'yzx':
        matrix = R_X @ R_Z @ R_Y
    elif order == 'xyx':
        matrix = R_X @ R_Y @ R_X
    elif order == 'xzx':
        matrix = R_X @ R_Z @ R_X
    elif order == 'yxy':
        matrix = R_Y @ R_X @ R_Y
    elif order == 'yzy':
        matrix = R_Y @ R_Z @ R_Y
    elif order == 'zxz':
        matrix = R_Z @ R_X @ R_Z
    elif order == 'zyz':
        matrix = R_Z @ R_Y @ R_Z
    elif order == 'zxy':
        matrix = R_Y @ R_X @ R_Z
    elif order == 'yxz':
        matrix = R_Z @ R_X @ R_Y
    elif order == 'xzy':
        matrix = R_Y @ R_Z @ R_X

    else:
        matrix = np.identity(3)

    return matrix


def Euler_Angle(alpha, beta, gamma, order='xyz'):
    """
    input
        alpha, beta, gamma = rotation angles in rotation order (degrees)
    output
        3x3 rotation matrix (numpy array) based on Euler-Angle (moving axis)
    """
    c_alpha = np.cos(np.radians(alpha))
    s_alpha = np.sin(np.radians(alpha))

    c_beta = np.cos(np.radians(beta))
    s_beta = np.sin(np.radians(beta))

    c_gamma = np.cos(np.radians(gamma))
    s_gamma = np.sin(np.radians(gamma))

    R_Z = np.array([[c_alpha, -s_alpha, 0],
                    [s_alpha, c_alpha, 0],
                    [0, 0, 1]]
                   )

    R_Y = np.array([[c_beta, 0, s_beta],
                    [0, 1, 0],
                    [-s_beta, 0, c_beta]]
                   )

    R_X = np.array([[1, 0, 0],
                    [0, c_gamma, -s_gamma],
                    [0, s_gamma, c_gamma]]
                   )

    if order == 'xyz':
        matrix = R_X @ R_Y @ R_Z
    elif order == 'zyx':
        matrix = R_Z @ R_Y @ R_X
    elif order == 'yzx':
        matrix = R_Y @ R_Z @ R_X
    elif order == 'xyx':
        matrix = R_X @ R_Y @ R_X
    elif order == 'xzx':
        matrix = R_X @ R_Z @ R_X
    elif order == 'yxy':
        matrix = R_Y @ R_X @ R_Y
    elif order == 'yzy':
        matrix = R_Y @ R_Z @ R_Y
    elif order == 'zxz':
        matrix = R_Z @ R_X @ R_Z
    elif order == 'zyz':
        matrix = R_Z @ R_Y @ R_Z
    elif order == 'zxy':
        matrix = R_Z @ R_X @ R_Y
    elif order == 'yxz':
        matrix = R_Y @ R_X @ R_Z
    elif order == 'xzy':
        matrix = R_X @ R_Z @ R_Y

    else:
        matrix = np.identity(3)

    return matrix


def Euler_Angles_from_R(R, order='xyz'):
    """
    input
        3x3 rotation matrix (numpy array) based on Euler-Angle (moving axis)
    output
        return alpha, beta, gamma in degrees (for all 6 orders)
    """
    if order == 'xzy':

        alpha = -np.arcsin(R[0, 2])
        gamma = np.arctan2(R[1, 2] / np.cos(alpha), R[2, 2] / np.cos(alpha))
        beta = np.arctan2(R[0, 1] / np.cos(alpha), R[0, 0] / np.cos(alpha))

    elif order == 'xyz':

        beta = -np.arcsin(R[2, 0])
        gamma = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
        alpha = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))

    elif order == 'yxz':

        alpha = -np.arcsin(R[1, 0])
        beta = np.arctan2(R[1, 2] / np.cos(alpha), R[1, 1] / np.cos(alpha))
        gamma = np.arctan2(R[2, 0] / np.cos(alpha), R[0, 0] / np.cos(alpha))

    elif order == 'yzx':

        alpha = np.arcsin(R[1, 2])
        gamma = np.arctan2(R[0, 2] / np.cos(alpha), R[2, 2] / np.cos(alpha))
        beta = np.arctan2(R[1, 0] / np.cos(alpha), R[1, 1] / np.cos(alpha))

    elif order == 'zyx':

        beta = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
        alpha = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
        gamma = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))

    elif order == 'zxy':

        alpha = -np.arcsin(R[1, 2])
        gamma = np.arctan2(R[0, 2] / np.cos(alpha), R[2, 2] / np.cos(alpha))
        beta = np.arctan2(R[1, 0] / np.cos(alpha), R[1, 1] / np.cos(alpha))

    else:
        alpha, beta, gamma = 0, 0, 0

    return np.degrees(alpha), np.degrees(beta), np.degrees(gamma)


