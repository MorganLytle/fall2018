import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

def RpToTrans (R,p):
#Takes rotation matrix R and position p.
#Returns corresponding homogeneous transformation matrix T in SE(3).
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

def main():
	R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
	p = np.array([1,2,5])
	T = np.array([[1, 0, 0, 1], [0, 0, -1, 2], [0, 1, 0, 5], [0, 0, 0, 1]])
	match = False
	epsilon = 10**-6 #threshold
	result = RpToTrans(R,p)
	resultDiff = result - T
	if(resultDiff.all() <= epsilon):
		match = True
	else:
		match = False
	print("Input matches SE(3): ")
	print(match)
	return match 

if __name__ == "__main__":
        main()

