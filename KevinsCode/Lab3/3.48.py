import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

def ScrewToAxis(q, s, h):
	return np.r_[s, np.cross(q,s) + np.dot(h,s)]

def VecToso3(omg):
	return [[0, -omg[2],  omg[1]], [omg[2], 0, -omg[0]], [-omg[1], omg[0], 0]]

def RpToTrans(R, p):
	return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]  

def TransToRp(T):
	R = [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]]
	return R, [T[0][3], T[1][3], T[2][3]]

def matrixTrans(S, theta, Tmat):
	I = [[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]]

	w = np.array([S[0], S[1], S[2]])
	v = np.array([S[3], S[4], S[5]])
	#print w,v

	skewW = VecToso3(w)
	skewV = VecToso3(v)

	#calculate terms of rodrigues form for exponential rotation matrix
	term1 = np.multiply(I,theta)
	term2 = np.multiply(1 - cos(theta), skewW)
	term3l = theta - sin(theta)
	term3r = np.matmul(skewW, skewW)
	term3 = np.multiply(term3l, term3r)

	R = term1 + term2 + term3 #Rotation matrix of exponential matrix
	#print R
	
	P = np.matmul(R,v) #column vector of exponential matrix
	#print P

	rT, pT = TransToRp(Tmat) # R and P of given matrix T
	
	#perform matrix transformation of form T_prime = ExpMatrix x T	
	rT_prime = np.matmul(R,rT)
	pT_prime = np.matmul(R,pT) + P
	
	return RpToTrans(rT_prime, pT_prime) #return transformation matrix of T prime



#begin of program
q = np.array([0,2,0])
s = np.array([0,0,1])
h = 2
THETA = pi
T = np.array([[1,0,0,2],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]])

S = ScrewToAxis(q,s,h)
#print S


print "\nwhen THETA = THETA/4"
T_prime_4 = matrixTrans(S, THETA * .25, T) # THETA = THETA/4
print T_prime_4

print "\nwhen THETA = THETA/2"
T_prime_2 = matrixTrans(S, THETA * .5, T) # THETA = THETA/2
print T_prime_2

print "\nwhen THETA = THETA*3/4"
T_prime_3_4 = matrixTrans(S, THETA * .75, T) # THETA = THETA*3/4
print T_prime_3_4

print "\nwhen THETA = THETA"
T_prime = matrixTrans(S, THETA, T) # THETA = THETA
print T_prime
