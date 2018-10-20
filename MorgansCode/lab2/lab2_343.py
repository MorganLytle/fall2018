import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

def NearZero(z):
#Takes a scalar.
#Checks if the scalar is small enough to be neglected.
	'''
Example Input:
z = -1e-7
Output:
True
	'''
	return abs(z) < 1e-6

def RotInv(R):
#Takes a 3x3 rotation matrix.
#Returns the inverse (transpose).
	return np.array(R).T


def main():
	R = np.array([[0, 0, 1],[1, 0, 0], [0, 1, 0]])
	I = np.identity(3)
	Rt = RotInv(R)
	epsilon = 10**-6 #difference threshold
	result = np.dot(Rt,R)
	resultDiff = result - I
	rDeterminant = np.linalg.det(R)
	match = False #variable to check if both conditions are satisfied
	match1 = False #variable to check if Rt*R=I
	match2 = False #variable to check if det(R) = 1
	if(resultDiff.all() <= epsilon):
		match1 = True
	else: 
		match1 = False
	print("Input transpose multiplied by original input is the identity: ")
	print(match1)
	
	#check if determinant = 1
	if((rDeterminant - 1) <= epsilon):
		match2 = True
	else:
		match2 = False
	print("The determinant is equal to 1: ")
	print(match2)

	if((match1 == True) and (match2 == True)):
		match = True
	else:
		match = False
	
	print("The input matrix is a rotation matrix: ")
	print(match)
	
	return match

if __name__ == "__main__":
	main()
