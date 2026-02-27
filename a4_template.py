"""
a4_template.py

This script is a template to be used on assignment 4
Here you should define the Puma robot following A4 design
Following the textbook, write your solution for the analytical inverse position and inverse orientation
Check your answers with modern_robotics foward kinematics
Use the modern_robotics library numerical IK method and see if they agree with your analytical
"""

import numpy as np
import modern_robotics as mr
from math import atan2, sqrt, cos, sin, pi, radians
 
np.set_printoptions(precision=3, suppress=True)

# PUMA parameters, following textbook
# a1 = 0.623
d1 = 0.140
a2 = 0.450
a3 = 0.490
# a4 = 0.050 

# Define below the values for M, omega_s, q_s, v_s, and Slist for A4 PUMA robot 

M = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

omega_s = np.array([[0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]])

q_s = np.array([[0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]])

v_s = np.zeros((6, 3))
for i in range(6):
    v_s[i] = np.cross(-omega_s[i], q_s[i])

# PUMA Screw axis Space
Slist = np.hstack((omega_s, v_s)).T

def PumaInversePosition(wrist_center):
    """
    Create the inverse position for the PUMA robot following the textbook Section 6.1.1 6R PUMA-Type Arm
    """
    # Extract wrist position
    px, py, pz = wrist_center

    # Hint: use the book's second solution without pi for theta1

    theta1, theta2, theta3 = 0, 0, 0 # Just a placeholder
    return theta1, theta2, theta3

def ExtractYXYAngles(R):
    """
    The PUMA wrist design on A4 can be defined as a YXY rotation matrix
    This part is slightly different from the textbook wrist
    Read Section B.1.2 Other Euler Angle Representations
    You should extract YXY Euler angles (alpha, beta, gamma) from a 3x3 rotation matrix
    Assumes the matrix follows R = Ry(alpha) * Rx(beta) * Ry(gamma)
    """

    alpha, beta, gamma = 0, 0, 0 # Just a placeholder
    return alpha, beta, gamma

def PumaInverseOrientation(X, theta1, theta2, theta3):
    """
    For the inverse orientation
    (1) Find the rotation matrix R from the right side of Eq. 6.2 of the textbook
    (2) Use your ExtractYXYAngles(R) to get theta4, theta5 and theta6.
    """

    R = np.array([[1, 0, 0, 0], # Just a placeholder
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    theta4, theta5, theta6 = ExtractYXYAngles(R)
    return theta4, theta5, theta6

def PumaAnalyticalIK(X):
    """
    This function calls the inverse position and inverse orientation for the PUMA
    """
    target_position = X[:3, -1]

    theta1, theta2, theta3 = PumaInversePosition(target_position)
    theta4, theta5, theta6 = PumaInverseOrientation(X, theta1, theta2, theta3)

    return np.array([theta1, theta2, theta3, theta4, theta5, theta6])

if __name__ == "__main__":
    #  Target pose X
    X = np.array([[1, 0, 0, 0.4],
                  [0, 1, 0, 0.2],
                  [0, 0, 1, 0.1], 
                  [0, 0, 0, 1.0]])
    
    # Create your own target pose
    # Keep in mind the robots workspace
    # and the wrist orthogonality
    # X = np.array([[1, 0, 0, 0.4],
    #               [0, -1, 0, 0.2],
    #               [0, 0, -1, 0.1],
    #               [0, 0, 0,   1]])
    print(f"X:\n{X}")

    thetalist_a = PumaAnalyticalIK(X)
    print(f"thetalist analytical:\n{thetalist_a}")
    
    # You can verify your solution by using mr.FKinSpace()
    # Check if T_a matches the provided target pose X
    T_a = mr.FKinSpace(M, Slist, thetalist_a)
    print(f"T_a:\n{T_a}") 

    # Use the mr.IKinSpace() to find a IK solution
    # Remember that numerical methods need a thetalist0 close to the target
    # Otherwise the method may find solutions outside of joint range
    # Also, the numerical methods may not converge at all
    # The numerical solution may be slightly diferent from the analytical
    # Specially for the wrist where multiple solutions are possible

    # You can use thetalist analytical as a initial guess
    # This may be specially usefull to check your wrist solution
    # thetalist0 = thetalist_a
    thetalist0 = np.array([-0.6, -0.85, 2.0, 0.2, 0.65, -1.0])
    # thetalist0 = np.array([-0.3, -0.4, 1.0, 0.2, 0.65, -1.0])

    eomg = 0.001
    ev = 0.0001
    
    thetalist_n, success = mr.IKinSpace(Slist, M, X, thetalist0, eomg, ev)
    if not success:
        print("No numerical solution found!")
    else:
        print(f"thetalist numerical:\n{thetalist_n}")
        
        # You can use the FK to verify the numerical solution too
        # Check if T_n matches the provided target pose X
        T_n = mr.FKinSpace(M, Slist, thetalist_n)
        print(f"T_n:\n{T_n}")