#!/usr/bin/env python

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
	
	###########################################################################################################################################
        return position(x, y, z)

    def inverse_kinematics(self, input_ee):
        self.end_effector = input_ee
	############################### check if the end effector position is reachable, your code goes below #####################################
	#in your code, please include 
	#raise ValueError('your words')

	############################################################################################################################################
        self.joints = self.compute_joints()
        return self.joints

    def compute_joints(self):
	#the return of the function are angles of joints, which should stay between -pi and pi. Start with self.end_effector.x and self.end_effector.y.
        ################################# Computes joint angle knowing end effector position, your code goes below #################################
	
	###########################################################################################################################################
        return jointangle(theta1, theta2, theta3, theta4)
