#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class turtlebot_move():
	def __init__(self):
		rospy.init_node('turtlebot_move', anonymous=False)
		rospy.loginfo("Press CTRL + C to terminate")
		rospy.on_shutdown(self.shutdown)

		self.vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		vel = Twist()   # init a message of Twist datatype
		rate = rospy.Rate(10)  # set spin rate for 10 Hz ---> 10 Hz = 0.1 seconds
		counter = 0   # init a counter
		counter2 = 0
		while not rospy.is_shutdown():
#####################Start your code#######################################
			vel.linear.x = 0.5            
			
			vel.angular.z = 0 # example of how to set velocity and angular velocity
			vel.angular.y = 0
			vel.angular.x = 0

			if(counter == 100):
				vel.linear.x = 0
				vel.angular.z = pi/2
				counter2 = counter2 + 1
				if(counter2 == 10):
					counter = 0
					counter2 = 0
			else:
				counter = counter + 1
			
#####################End your code##########################################
			self.vel_pub.publish(vel)
			rate.sleep()

	def shutdown(self):
		rospy.loginfo("Stop Action")
		stop_vel = Twist()
		stop_vel.linear.x = 0
		stop_vel.angular.z = 0
		self.vel_pub.publish(stop_vel)
		rospy.sleep(1)

if __name__ == '__main__':
	try:
		turtlebot_move()
	except rospy.ROSInterruptException:
		rospy.loginfo("Action terminated.")
