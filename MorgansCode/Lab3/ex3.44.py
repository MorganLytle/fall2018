import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

#matrix r
r = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
#position p
position = np.array([1, 2, 5])
#comparison matrix for SE(3)
i = np.array([[1, 0, 0, 1], [0, 0, -1, 2], [0, 1, 0, 5], [0, 0, 0, 1]])
#epsilon value
e = 10**(-6)

#returns corresponding homogeneous transformation matrix T in SE(3)
def RpToTrans(R, p):
	return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

#creating transformation matrix T in SE(3)
rt = RpToTrans(r, position)

#comparing if our created transformation matrix is the same as the matrix i
def compare(result):
	if(rt.all() == i.all()):
		output = True
		print(output)
		return output
	else:
		output = False
		print(output)
		return output

result = compare(rt)
