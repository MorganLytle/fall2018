import rospy
from geometry_msgs.msg import Twist
from math import pi

class turtlebot_move():
	def __init__(self):
		rospy.init_node('turtlebot_move', anonymous=False)
		rospy.loginfo("Press CTRL + C to terminate")
		rospy.on_shutdown(self.shutdown)
        	vel = Twist()
		self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
#####################Start your code########################################
		anglePosition = 2*pi
		time = 30#seconds
		radius = 3
		angleVel = anglePosition/time
		velocity = angleVel*radius

		vel.linear.x = velocity #w=V/r
		vel.angular.z = angleVel # example of how to set velocity and angular velocity
#####################End your code##########################################
		rate = rospy.Rate(10);
		while not rospy.is_shutdown():
			self.set_velocity.publish(vel)
			rate.sleep()
                        
        
	def shutdown(self):

		rospy.loginfo("Stop Action")
		stop_vel = Twist()
		stop_vel.linear.x = 0
		stop_vel.angular.z = 0
		self.set_velocity.publish(stop_vel)

		rospy.sleep(1)
 
if __name__ == '__main__':
	try:
		turtlebot_move()
	except rospy.ROSInterruptException:
		rospy.loginfo("Action terminated.")

