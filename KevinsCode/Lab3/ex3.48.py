import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

transMatrix = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0 ,3], [0, 0, 0, 1]])
e = 10**-6
q = np.array([0, 2, 0])
s = np.array([0, 0, 1])
h = 2
th = pi

def NearZero(z):
	return abs(z) < 1e-6

def TransToRp(T):
	R = [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]]
	return R, [T[0][3], T[1][3], T[2][3]]

def MatrixLog3(R):
	if NearZero(np.linalg.norm(R - np.eye(3))):
		return np.zeros(3,3)
	elif NearZero(np.trace(R) + 1):
		if not NearZero(1 + R[2][2]):
			omg = (1.0 / sqrt(2 * (1 + R[2][2]))) * np.array([R[0][2], R[1][2], 1 + R[2][2]])
		elif not NearZero(1 + R[1][1]): 
			omg = (1.0 / sqrt(2 * (1 + R[1][1]))) * np.array([R[0][1], 1 + R[1][1], R[2][1]])
		else:
			omg = (1.0 / sqrt(2 * (1 + R[0][0]))) * np.array([1 + R[0][0], R[1][0], R[2][0]])
		return VecToso3(pi*omg)
	else:
		acosinput = (np.trace(R) - 1) / 2.0
		if acosinput > 1:
			acosinput = 1
		elif acosinput < -1:
			acosinput = -1		
		theta = acos(acosinput)
		return theta / 2.0 / sin(theta) * (R - np.array(R).T)

def MatrixLog6(T):
	R,p = TransToRp(T)
	if NearZero(np.linalg.norm(R - np.eye(3))):
		return np.r_[np.c_[np.zeros((3,3)), [T[0][3], T[1][3], T[2][3]]], [[0, 0, 0, 0]]]
	else: 
		acosinput = (np.trace(R) - 1) / 2.0
		if acosinput > 1:
			acosinput = 1
		elif acosinput < -1:
			acosinput = -1		
		theta = acos(acosinput)       
		omgmat = MatrixLog3(R) 
		return np.r_[np.c_[omgmat, np.dot(np.eye(3) - omgmat / 2.0 + (1.0 / theta - 1.0 / tan(theta / 2.0) / 2) * np.dot(omgmat, omgmat) / theta, [T[0][3], T[1][3], T[2][3]])], [[0, 0, 0, 0]]]

resultExponCoord = MatrixLog6(transMatrix)
print("EXPONENTIAL COORDINATES")
print(resultExponCoord)

def AxisAng6(expc6):
	theta = np.linalg.norm([expc6[0], expc6[1], expc6[2]])
	if NearZero(theta):
		theta = np.linalg.norm([expc6[3], expc6[4], expc6[5]])
	return (expc6 / theta,theta)

resultS = AxisAng6(resultExponCoord)
print("VALUE OF S")
print(resultS)