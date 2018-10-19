import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

#given matrix
matrix = np.array([[0, -3, 2, 4], [3, 0, -1, 5], [-2, 1, 0, 6], [0, 0, 0, 0]])
#value for epsilon
e = 10**-5

#takes 4x4 se(3) matrix and returns a 6-vector
def se3ToVec(se3mat):
	return np.r_[[se3mat[2][1], se3mat[0][2], se3mat[1][0]], [se3mat[0][3], se3mat[1][3], se3mat[2][3]]]

#creates 6-vector
resultMatrix = se3ToVec(matrix)
comEpsilon = resultMatrix - e

#comparing 6-vector to our original matrix
def compareMatrix(test):
	if(test.all() == matrix.all()):
		print("True: the matrix matches")
		print(test)
		return True
	else:
		print("False: the matrix does not match")
		print(test)
		return False

#comparing epsilon
def compareEpsilon(test2):
	if(test2.all() <= e):
		print("True: it is less than our value epsilon (10^-6)")
		print(test2)
		return True
	else:
		print("False: it is not less our value epsilon (10^-6)")
		print(test2)
		return False

output = compareMatrix(resultMatrix)
output2 = compareEpsilon(comEpsilon)
