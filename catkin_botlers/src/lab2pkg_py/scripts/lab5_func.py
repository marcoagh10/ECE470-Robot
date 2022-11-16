#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def SM(w, v):
    return np.array(
		[
      	[0, -w[2], w[1], v[0]],
   		[w[2], 0, -w[0], v[1]],
   		[-w[1], w[0], 0, v[2]],
   		[0, 0, 0, 0],]
	)

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	w1 = np.array([0,0,1])
	w2 = np.array([0,1,0])
	w3 = np.array([0,1,0])
	w4 = np.array([0,1,0])
	w5 = np.array([1,0,0])
	w6 = np.array([0,1,0])

	q1 = np.array([-150, 150, 10+152])
	q2 = np.array([-150, 150, 10+152])
	q3 = np.array([-150 + 244, 150, 10+152])
	q4 = np.array([-150 + 244 + 213, 150, 10+152])
	q5 = np.array([-150 + 244 + 213, 150 + 120 -93 + 83, 10+152])
	q6 = np.array([-150 + 244 + 213 + 83, 150 + 120 -93 + 83, 10 + 152])
 
	v1 = -np.cross(w1, q1)
	v2 = -np.cross(w2, q2)
	v3 = -np.cross(w3, q3)
	v4 = -np.cross(w4, q4)
	v5 = -np.cross(w5, q5)
	v6 = -np.cross(w6, q6)
 
	S1 = SM(w1, v1)
	S2 = SM(w2, v2)
	S3 = SM(w3, v3)
	S4 = SM(w4, v4)
	S5 = SM(w5, v5)
	S6 = SM(w6, v6)
 

	M = np.array(
		[[0, -1,  0, -150 + 244 + 213 + 83       ],
   		 [0,  0, -1, 150 + 120 -93 + 83 + 82 + 59],
         [1,  0,  0, 10 + 152 + 53.5             ],
     	 [0,  0,  0, 1                           ],
     ]
	)
 
	S = [S1, S2, S3, S4, S5, S6]

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
 
	print(np.degrees(theta))
 
	T = np.eye(4)

	M, S = Get_MS()

	angles = [theta1, theta2, theta3, theta4, theta5, theta6]
 
	T = np.eye(4)
 
	for theta, Si in zip(angles, S):
		transform = expm(theta * Si)
		T = T @ transform

	T = T @ M
	# print(str(T) + "\n")

	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	print(xWgrip, yWgrip, zWgrip, yaw_WgripDegree)

	yaw_WgripDegree = np.radians(yaw_WgripDegree)

	L1 = 152
	L2 = 120
	L3 = 244
	L4 = 93
	L5 = 213
	L6 = 83
	L7 = 83
	L8 = 82
	L9 = 53.5
	L10 = 59
	E = 27

	# xGrip = xWgrip + 150
	# yGrip = yWgrip - 150
	# zGrip = zWgrip - 10

	xGrip = xWgrip
	yGrip = yWgrip
	zGrip = zWgrip

	xCenter = xGrip - L9 * np.cos(yaw_WgripDegree)
	yCenter = yGrip - L9 * np.sin(yaw_WgripDegree)
	zCenter = zGrip
 
	h = np.sqrt(xCenter **2 + yCenter ** 2)
	thetaX = np.arcsin((E + L6) / h)
 

	theta1 = np.arctan2(yCenter, xCenter) - thetaX
 
	theta6 = theta1 + np.radians(90) - yaw_WgripDegree
 
	xT = E + L6
	yT = L7
 
	x3end = xCenter - yT * np.cos(theta1) + xT * np.sin(theta1)
	y3end = yCenter - yT * np.sin(theta1) - xT * np.cos(theta1)
	z3end = zCenter + L10 + L8

	Q = np.sqrt(x3end**2 + y3end**2 + (z3end - L1)**2)
	F = np.sqrt(x3end**2 + y3end**2 )
	R = z3end - L1
 
 
	negTheta2Bottom = np.arctan2(R, F)
	negTheta2Top = np.arccos((L5**2 - L3 **2 - Q**2)/(-2 * L3 * Q))
 
	theta2 = -(negTheta2Bottom + negTheta2Top)
 
	theta3 = np.radians(180) - np.arccos((Q**2 - L3 **2 - L5 **2)/(-2 * L3 * L5)) 
	theta4 = -theta2 - theta3

	theta5 = np.radians(-90)
 
	# ==============================================================#
	return lab_fk(theta1,theta2,theta3,theta4,theta5,theta6)
