'''
a3_sample.py
This script is a sample code for verification of Assignment 3 on forward kinematics.
'''

import modern_robotics as mr
import numpy as np

np.set_printoptions(suppress=True, precision=4, linewidth=150)

# Sample input
M = np.array([[-1, 0,  0, 0],
              [ 0, 1,  0, 6],
              [ 0, 0, -1, 2],
              [ 0, 0,  0, 1]])

Slist = np.array([[0, 0,  1,  4, 0,    0],
                  [0, 0,  0,  0, 1,    0],
                  [0, 0, -1, -6, 0, -0.1]]).T

thetalist = np.array([np.pi / 2.0, 3, np.pi])

fk = mr.FKinSpace(M, Slist, thetalist)

print(fk)