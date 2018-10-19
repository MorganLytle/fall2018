import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

#original matrix
m = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
#comparison matrix
i = np.array([[0, -1.20919958, 1.20919958], [1.20919958, 0, -1.20919958], [-1.20919958, 1.20919958, 0]])
#epsilon
e = 10**-6

def NearZero(z):
	return abs(z) < 1e-6

def MatrixLog3(R):
	if NearZero(np.linalg.norm(R - np.eye(3))):
		return np.zeros(3,3)
	elif NearZero(np.trace(R) + 1):
		if not NearZero(1 + R[2][2]):
			omg = (1.0 / sqrt(2 * (1 + R[2][2]))) \
				* np.array([R[0][2], R[1][2], 1 + R[2][2]])
		elif not NearZero(1 + R[1][1]): 
			omg = (1.0 / sqrt(2 * (1 + R[1][1]))) \
				* np.array([R[0][1], 1 + R[1][1], R[2][1]])
		else:
			omg = (1.0 / sqrt(2 * (1 + R[0][0]))) \
				* np.array([1 + R[0][0], R[1][0], R[2][0]])
		return VecToso3(pi*omg)
	else:
		acosinput = (np.trace(R) - 1) / 2.0
		if acosinput > 1:
			acosinput = 1
		elif acosinput < -1:
			acosinput = -1		
		theta = acos(acosinput)
		return theta / 2.0 / sin(theta) * (R - np.array(R).T)

#creating so(3) representation of expontential coordinates
so3 = MatrixLog3(m)
difResult = so3 - i

#comparing matrix
def compare(test):
	if(test.all() == m.all()):
		print("True: the matrix matches")
		print(test)
		return True
	else:
		print("False: the matrix does not match")
		print(test)
		return False

#comparing epsilon
def threshold(test2):
	if(test2.all() <= e):
		print("True: it is less than epsilon (10^-6)")
		print(test2)
		return True
	else:
		print("False: it is not less than epsilon (10^-6)")
		print(test2)
		return False

result = compare(so3)
epsilon = threshold(difResult)
