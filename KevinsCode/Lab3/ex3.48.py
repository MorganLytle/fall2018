import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

t = np.array([[1, 0, 0, 2], [0, 1, 0, 0], [0, 0, 1 ,0], [0, 0, 0, 1]])
q1 = np.array([0, 2, 0])
s1 = np.array([0, 0, 1])
h1 = 2
theta0 = 0
theta1 = pi/4
theta2 = pi/2
theta3 = 3*pi/4

def ScrewToAxis(q, s, h):
	return np.r_[s, np.cross(q, s) + np.dot(h, s)]

def VecToso3(omg):
	return [[0, -omg[2], omg[1]], [omg[2], 0, -omg[0]], [-omg[1], omg[0], 0]]

def RpToTrans(R, p):
	return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

def TransToRp(T):
	R = [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]]
	return R, [T[0][3], T[1][3], T[2][3]]

def RodForm(theta, w):
	I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
	output = np.multiply(I, theta)
	output2 = np.multiply(1 - cos(theta), newW)
	output3_1 = theta - sin(theta)
	output3_2 = np.matmul(newW, newW)
	output3 = np.multiply(output3_1, output3_2)
	R  = output + output2 + output3
	return R

s = ScrewToAxis(q1, s1, h1)
w = np.array([s[0], s[1], s[2]])
v = np.array([s[3], s[4], s[5]])

print("Angular velocity: ")
print(w)
print("Linear velocity: ")
print(v)

print(" ")

newW = VecToso3(w)
newV = VecToso3(v)

print("Skew w: ")
print(newW)
print("Skew v: ")
print(newV)

print(" ")

#Rodrigues' Formula
r0 = RodForm(theta0, newW)
r1 = RodForm(theta1, newW)
r2 = RodForm(theta2, newW)
r3 = RodForm(theta3, newW)

print("Rodriges' Formula 0: ")
print(r0)
print("Rodriges' Formula 1: ")
print(1)
print("Rodriges' Formula 2: ")
print(r2)
print("Rodriges' Formula 3: ")
print(r3)

print(" ")

def matrixTransform(rod):
	#colum vector of exponential matrix
	p = np.matmul(rod, v)

	matrixRT, matrixPT = TransToRp(t)

	rTPrime  = np.matmul(rod, matrixRT)
	pTPrime = np.matmul(rod, matrixPT) + p

	matrixTPrime = RpToTrans(rTPrime, pTPrime)

	print("Column vecotr of exponential matrix: ")
	print(p)

	print(" ")

	print("Value of R given matrix T: ")
	print(matrixRT)
	print("Value of p given matrix T: ")
	print(matrixPT)

	print(" ")


	return matrixTPrime

tPrime0 = matrixTransform(r0)
tPrime1 = matrixTransform(r1)
tPrime2 = matrixTransform(r2)
tPrime3 = matrixTransform(r3)


print("Value of T0: ")
print(tPrime0)

print(" ")

print("Value of T1: ")
print(tPrime1)

print(" ")

print("Value of T2: ")
print(tPrime2)

print(" ")

print("Value of T3: ")
print(tPrime3)
