"""

Sunny Karmur PUMA_560_KINEMATICS_MOTION_PLANNER


"""



import numpy as np
from PUMA_560_Source.Euler import Euler_Angle, Euler_Angles_from_R

######## Part A #######################
R_XYZ = Euler_Angle(60, 30, 60, 'yxz')
print ("Rotation Matrix R_XYZ = ")
print (R_XYZ)
"""
Test your code
Euler_Angle(30, 45, 60, 'xyz') = 
[[ 0.61237244 -0.35355339  0.70710678]
 [ 0.78033009  0.12682648 -0.61237244]
 [ 0.12682648  0.9267767   0.35355339]]
 Try other examples
"""

## check your code
alpha, beta, gamma = Euler_Angles_from_R(R_XYZ)
print ("alpha, beta, gamma = ")
print (alpha, beta, gamma)

## Find the axis of rotation
Eval, Evec = np.linalg.eig(R_XYZ)
print ("\nEigen Values = ")
print (Eval)
print ("\nEigen Vectors = ")
print (Evec)


