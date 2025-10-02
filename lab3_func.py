#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	q1 = np.array([0,0,0.152])
	q2 = np.array([0,0.12,0])
	q3 = np.array([0, 0.093, 0])
	q4 = np.array([0, 0.083, 0])
	q5 = np.array([0.083, 0, 0])
	q6 = np.array([0, 0.082, 0])

	omega1 = np.array([0, 0, 1])
	omega2 = np.array([0, 1, 0])
	omega3 = np.array([0, 1, 0])
	omega4 = np.array([0, 1, 0])
	omega5 = np.array([1, 0, 0])
	omega6 = np.array([0, 1, 0])

	v1 = -np.cross(omega1, q1)
	v2 = -np.cross(omega2, q2)
	v3 = -np.cross(omega3, q3)
	v4 = -np.cross(omega4, q4)
	v5 = -np.cross(omega5, q5)
	v6 = -np.cross(omega6, q6)

	S1 = np.concatenate([omega1, v1])
	S2 = np.concatenate([omega2, v2])
	S3 = np.concatenate([omega3, v3])
	S4 = np.concatenate([omega4, v4])
	S5 = np.concatenate([omega5, v5])
	S6 = np.concatenate([omega6, v6])

	S = np.array([S1, S2, S3, S4, S5, S6])

	M = np.array([
		[0, -1, 0, 0.54],
		[0, 0, -1, 0.192],
		[1, 0, 0, 0.152], 
		[0, 0, 0, 1]
	])

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	M, S = Get_MS()

	T = np.eye(4)
	thetas = [theta1*np.pi/180, theta2*np.pi/180, theta3*np.pi/180, theta4*np.pi/180, theta5*np.pi/180, theta6*np.pi/180]

	for i in range(6):
		omega = S[i][0:3]
		v = S[i][3:6]

		skew = np.array([
			[0, -omega[2], omega[1]],
			[omega[2], 0, -omega[0]],
			[-omega[1], omega[0], 0]
		])

		S_matrix = np.zeros((4, 4))
		S_matrix[0:3, 0:3] = skew
		S_matrix[0:3, 3] = v

		exponentials = expm(S_matrix * thetas[i])

		T = T @ exponentials
	
	T = T @ M

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
