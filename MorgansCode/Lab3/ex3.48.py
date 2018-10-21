import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

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



def AxisAng6(expc6):
	theta = np.linalg.norm([expc6[0], expc6[1], expc6[2]])
	if NearZero(theta):
		theta = np.linalg.norm([expc6[3], expc6[4], expc6[5]])
	return (expc6 / theta,theta)

def TransToRp (T):
#Takes transformation matrix T in SE(3). 
#Returns R: The corresponding rotation matrix,
#        p: The corresponding position vector.
    R = [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    return R, [T[0][3], T[1][3], T[2][3]]

def RotInv(R):
#Takes a 3x3 rotation matrix.
#Returns the inverse (transpose).
    return np.array(R).T



def TransInv(T):
#Takes a transformation matrix T. 
#Returns its inverse.
#Uses the structure of transformation matrices to avoid taking a matrix
#inverse, for efficiency.
    R,p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]

def rotCheck(R):
	invR = RotInv(R)
	I = np.identity(3)
	RtR =np.dot(invR,R)
	if (abs(RtR.all()-I.all()) <= 10**-6):
		print("Rotation matrix is valid")
		check = 1
	else:
		print("Rotation matrix is NOT valid")
		check = 0
	return check

def transCheck(T):
	#Checks to see if the translation matrix is valid
	R,p = TransToRp(T)
	invT = TransInv(T)
	
	transR, negTransRp = TransToRp(invT)
	check1 = rotCheck(R)
	Rdiff = np.absolute(np.subtract(transR,R))
	pdiff = np.absolute(np.subtract(negTransRp,p))
	if ((Rdiff.all()<=10**-6) and (pdiff.all()<=10**-6) and (check1 == 1)):
		print("Transformation matrix is valid")
		check2 = 1
	else:
		print("Transformation matrix is NOT valid")	
		check2 = 0
	return check2

def se3ToVec(se3mat):
#Takes se3mat a 4x4 se(3) matrix.
#Returns the corresponding 6-vector (representing spatial velocity).
   
	 return np.r_[[se3mat[2][1], se3mat[0][2], se3mat[1][0]], [se3mat[0][3], se3mat[1][3], se3mat[2][3]]]

def TransToRp (T):
#Takes transformation matrix T in SE(3). 
#Returns R: The corresponding rotation matrix,
#        p: The corresponding position vector.
    R = [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    return R, [T[0][3], T[1][3], T[2][3]]


def Adjoint(T):
#Takes T a transformation matrix SE(3).
#Returns the corresponding 6x6 adjoint representation [AdT].
    R,p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3,3))],
                 np.c_[np.dot(VecToso3(p),R), R]]





def main():
	T = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0 ,3], [0, 0, 0, 1]])
	epsilon = 10**-6
	q = np.array([0, 2, 0])
	s = np.array([0, 0, 1])
	h = 2
	theta = np.array([0, pi/4, pi/2, 3*pi/4, pi])
	inputCheck =transCheck(T)
	if inputCheck != 1:
		return
	
	'''
	#Get se(3) representation of exponential coordinates
	resultExponCoord = MatrixLog6(T)
	print("EXPONENTIAL COORDINATES")
	print(resultExponCoord)

	#Convert the exponential coordinates to a 6-vector
	resultVector = se3ToVec(resultExponCoord)

	#Get S: The corresponding normalized screw axis
	resultS = AxisAng6(resultVector)
	print("VALUE OF S")
	print(resultS)

	#Find the Adjoint of S ([S])
	adS = Adjoint(resultS)
	print("[S] = ")
	print(adS)
	'''


	return
	
	

if __name__ == "__main__":
	main()
