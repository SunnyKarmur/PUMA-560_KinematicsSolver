"""

Sunny Karmur PUMA_560_KINEMATICS_MOTION_PLANNER


"""




import numpy as np

class Puma_FK:
    def __init__(self, a2, a3, d3, d4):
        # all alpha_{i-1}

        self.alpha = [0, -np.pi / 2, 0, -np.pi / 2, np.pi / 2, -np.pi / 2]

        # all a_{i-1}
        self.a = [0, 0, a2, a3, 0, 0]

        # all d_iA
        self.d = [0, 0, d3, d4, 0, 0]


    def get_Ti(self, i):
        # return the transformation {i-1}T{i}: formula 3.6 (Craig's book)

        # Defining the variables

        T_i = np.matrix([[np.cos(self.theta[i-1]), -np.sin(self.theta[i-1]), 0, self.a[i-1]],
                          [np.sin(self.theta[i-1]) * np.cos(self.alpha[i-1]), np.cos(self.theta[i-1]) * np.cos(self.alpha[i-1]), -np.sin(self.alpha[i-1]),
                           -np.sin(self.alpha[i-1]) * self.d[i-1]],
                          [np.sin(self.theta[i-1]) * np.sin(self.alpha[i-1]), np.cos(self.theta[i-1]) * np.sin(self.alpha[i-1]), np.cos(self.alpha[i-1]),
                           np.cos(self.alpha[i-1]) * self.d[i-1]],
                          [0, 0, 0, 1]])
        return T_i


