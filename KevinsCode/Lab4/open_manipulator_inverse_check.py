#!/usr/bin/env python

from __future__ import division
import rospy
import numpy as np
from std_msgs.msg import Float64
from collections import namedtuple
import open_manipulator_kinematics
from open_manipulator_kinematics import position, jointangle, makeVector
from math import pi, cos, sin
import unittest
gripperangle = namedtuple('gripperangle', ['theta1', 'theta2'])

link1 = 0.13
link2 = 0.124
link3 = 0.07
class open_manipulator_move():
    def __init__(self):
        rospy.init_node('open_manipulator_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.shutdown)
# Define publisher for each joint
        self.position_pub_joint1 = rospy.Publisher('/open_manipulator/joint1_position/command', Float64, queue_size=10)
	self.position_pub_joint2 = rospy.Publisher('/open_manipulator/joint2_position/command', Float64, queue_size=10)
	self.position_pub_joint3 = rospy.Publisher('/open_manipulator/joint3_position/command', Float64, queue_size=10)
	self.position_pub_joint4 = rospy.Publisher('/open_manipulator/joint4_position/command', Float64, queue_size=10)
	self.position_pub_grip1 = rospy.Publisher('/open_manipulator/grip_joint_position/command', Float64, queue_size=10)
	self.position_pub_grip2 = rospy.Publisher('/open_manipulator/grip_joint_sub_position/command', Float64, queue_size=10)

	end_effector = position(0.12, 0.2, 0.1)
	opm_kin = open_manipulator_kinematics.open_manipulator_kinematics(link1=link1, link2=link2, link3=link3)
	joints = opm_kin.inverse_kinematics(end_effector)
	grippers=gripperangle(0.01,0.01)
	rate = rospy.Rate(10)  # set spin rate for 10 Hz
	counter=0
	while not rospy.is_shutdown():
	    if counter > 10:   # hold for 1 second
            	self.position_pub_joint3.publish(np.float64(-0.8))
            if counter > 20:   # hold for 1 second
            	self.position_pub_joint1.publish(joints.theta1)
            if counter > 30:   # hold for 1 second
            	self.position_pub_joint2.publish(joints.theta2)
            if counter > 40:   # hold for 1 second
            	self.position_pub_joint3.publish(joints.theta3)
            if counter > 50:   # hold for 1 second
            	self.position_pub_joint4.publish(joints.theta4)
	    if counter > 60:   # hold for 1 second
	    	self.position_pub_grip1.publish(grippers.theta1)
            	self.position_pub_grip2.publish(grippers.theta2)
	    counter=counter+1
            rate.sleep()

    def shutdown(self):
	rate = rospy.Rate(10)  # set spin rate for 10 Hz
        rospy.loginfo("Return to the origin")
	for i in range(60):
            if i > 10:   # hold for 1 second
        	self.position_pub_joint2.publish(np.float64(0))
            if i > 20:   # hold for 1 second
		self.position_pub_joint3.publish(np.float64(0))
            if i > 30:   # hold for 1 second
		self.position_pub_joint1.publish(np.float64(0))
            if i > 40:   # hold for 1 second
		self.position_pub_joint4.publish(np.float64(0))
	    rate.sleep()
	self.position_pub_grip1.publish(np.float64(0))
	self.position_pub_grip2.publish(np.float64(0))
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        open_manipulator_move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
