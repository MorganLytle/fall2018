import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

m = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
i = np.identity(3)
e = 10**(-6) 

#returns inverse (transpose)
def RotInv(R):
	
	return np.array(R).T

#checks if the scalar is small enough to be neglected
def NearZero(z):
	
	return abs(z) < 1*e-6

mt = RotInv(m)
identity = np.dot(m, mt)
result = identity - i
det = np.linalg.det(m)

#checking if identity equals the identity matrix
if(identity.all() == i.all()):
	print(i)
	print(identity)
	output = True
	return output
else:
	print(i)
	print(identiy)
	output = False
	return output
print("output")
print(output)

#checking if result is less than eps
if(result.all() < e):
	print(result)
	print(e)
	output2 = True
	return output2
else:
	print(result)
	print(e)
	output2 = False
	return output2
print("output2")
print(output2)

#checking if det is smaller than eps
if(det < e):
	output3 = True
	return output3
else:
	output3 = False
	return output3
print("output3")
print(output3)
