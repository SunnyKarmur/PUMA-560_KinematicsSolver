"""

Sunny Karmur PUMA_560_KINEMATICS_MOTION_PLANNER


"""
import numpy as np
from math import radians
from PUMA_560_Source.PUMA import Puma_FK

######## Part C #######################
# PUMA robot: all joint angles
theta1 = radians(10)
theta2 = radians(20)
theta3 = radians(30)
theta4 = radians(45)
theta5 = radians(50)
theta6 = radians(60)

a2 = 5.
a3 = 0.5
d3 = 1.
d4 = 10.

puma = Puma_FK(a2, a3, d3, d4)

puma.theta = [10, 20, 30, 45, 50, 60]

T_10 = puma.get_Ti(i=1)
T_21 = puma.get_Ti(i=2)
T_32 = puma.get_Ti(i=3)
T_43 = puma.get_Ti(i=4)
T_54 = puma.get_Ti(i=5)
T_65 = puma.get_Ti(i=6)


# 4T6 = 4T5 * 5T6

T_64 = T_54 @ T_65
print ("transformation form frame 6 to frame 4:")
print(T_64)

# Calculate results for 0T6 and 6T0

T_60 = T_10 @ T_21 @ T_32 @ T_43 @ T_54 @ T_65

print ("transformation form frame 6 to frame 0:")
print(T_60)

print ("transformation form frame 0 to frame 6:")
T_06 = np.linalg.inv(T_60)

print(T_06)

Point_0 = np.matrix([5, 5, 5, 1])   # 0_P (point at {0} frame)

# Calculate Point_6 (point at {6} frame)

Point_6 = T_06 @ Point_0.transpose()
print ("Coordinates of point in frame 6")
print(Point_6[:3, 0])