#run gazebo
#roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch

from __future__ import division
from math import pi
import numpy as np
import math
from math import pi, cos, sin
from collections import namedtuple

position = namedtuple('position', ['x', 'y', 'z'])
jointangle = namedtuple('jointangle', ['theta1', 'theta2', 'theta3', 'theta4'])
makeVector = namedtuple('makeVector', ['x', 'y', 'z'])

class open_manipulator_kinematics(object):
        #q0 - initial positions of joints
        #origin - position of the base of the arm in carthesian space
    def __init__(self, link1=0.13, link2=0.124, link3=0.07, q0=jointangle(0,0,0,0), origin=makeVector(0,0,0)):
        self.link1 = link1
        self.link2 = link2
        self.link3 = link3
        self.lsq1 = link1 ** 2 + link2 ** 2
        self.joints = q0
        self.origin = origin
        self.end_effector = self.compute_end_effector()

    def forward_kinematics(self, input_joints):
        self.joints = input_joints
        self.end_effector = self.compute_end_effector()
        return self.end_effector

    def compute_end_effector(self):
        # the return of the function is the position of end_effector, start with self.joints.theta1, self.joints.theta2, self.joints.theta3 and self.joints.theta4
        ################################ Computes end_effector position knowing joint angles, your code goes between ##############################
	T01 = np.array([[cos(self.joints.theta1), -sin(self.joints.theta1), 0, 0], 
			[sin(self.joints.theta1), cos(self.joints.theta1), 0, 0], 
			[0, 0, 1, 0], 
			[0, 0, 0, 1]])

	T12 = np.array([[cos(self.joints.theta2), 0, sin(self.joints.theta2), self.link1], 
			[0, 1, 0 , 0], 
			[-sin(self.joints.theta2), 0, cos(self.joints.theta2), 0], 
			[0, 0, 0, 1]])

        T23 = np.array([[cos(self.joints.theta2), 0, sin(self.joints.theta2), self.link2],
                        [0, 1, 0 , 0], 
                        [-sin(self.joints.theta2), 0, cos(self.joints.theta2), 0], 
                        [0, 0, 0, 1]])

        T34 = np.array([[cos(self.joints.theta2), 0, sin(self.joints.theta2), self.link3],
                        [0, 1, 0 , 0], 
                        [-sin(self.joints.theta2), 0, cos(self.joints.theta2), 0], 
                        [0, 0, 0, 1]])

	T04 = T01*T12*T23*T34

	x = [T04[0][3]]
	y = [T04[1][3]]
	z = [T04[2][3]]

	print(x)
	print(y)
	print(z)
        ###########################################################################################################################################
        return position(x, y, z)

    def inverse_kinematics(self, input_ee):
        self.end_effector = input_ee
        ############################### check if the end effector position is reachable, your code goes below #####################################
        #in your code, please include 
        #raise ValueError('your words')
	xDistance = (0 - 0.218)**2
	yDistance = (0 - 0.203)**2
	disFormula = math.sqrt(xDistance + yDistance)
	linkLength = self.link1 + self.link2 + self.link3

	if(linkLength > disFormula):
		raise ValueError('END EFFECTOR POSTION NOT REACHABLE')
	else:
		print('Maximum length: ')
		print(disFormula)
		print(' ')
		print('Reach of the links: ')
		print(linkLength)
        ############################################################################################################################################
        self.joints = self.compute_joints()
        return self.joints

    def compute_joints(self):
        #the return of the function are angles of joints, which should stay between -pi and pi. Start with self.end_effector.x and self.end_effector.y.
        ################################# Computes joint angle knowing end effector position, your code goes below #################################
	self.end_effector.x
	self.end_effector.y
	
        ###########################################################################################################################################
        return jointangle(theta1, theta2, theta3, theta4)
