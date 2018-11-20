#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from time import time
import geometry_msgs.msg
import tf
import math

class turtlebot_move():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)  		

	rospy.loginfo("Press CTRL + C to terminate")
 
        rospy.on_shutdown(self.shutdown)
        
        self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

	
	#reset odometry
	reset_odom = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10)
	#this message take a few iterations to get through
	timer=time()
	while time()-timer<1.0:
	      reset_odom.publish(Empty())
	#Initialize the tf listener
	tfListener = tf.TransformListener()

        vel = Twist()
        vel.linear.x = 0.5
	vel.angular.z = 0

	#START LAB6 CODE
	rate = rospy.Rate(100); 	#initialize clockspeed TO 1 hertz
	currT = rospy.Time(0); 	#obtain initial time
	diffT = 0 		#diffT tracks counter of 10 secs before turning
	prevT = 0 		#assign currT to prevT for next iteration

	desired_phi = 0 #phi star
	curr_phi = 0    #current orientation
	diff_phi = 0    #difference between desired and current orientation
	kp = 1		#P-controller constant

	#rospy.loginfo("position: "+str(position))
        #rospy.loginfo("orientation: "+str(orientation))

	
	
        while not rospy.is_shutdown():
	    currT = rospy.Time.now();	#retrieve updated time each cylce			
	    if prevT == 0:
		prevT = currT
	    diffT = diffT + currT.to_sec() - prevT.to_sec() #diff between clock cycles
	    try:
		(position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
	    except:
      		continue
	    orientation = tf.transformations.euler_from_quaternion(quaternion)
	    
	    #change quaternion to Euler angles since we have not learned quaternion in class. Orientation is in the form of [roll, pitch, yaw]. 

	    curr_phi = orientation[2]
	    diff_phi = desired_phi - curr_phi
		
	    omega = kp * diff_phi

	    if diffT >= 10:

		
		rospy.loginfo("Stop Action")
		stop_vel = Twist()
        	stop_vel.linear.x = 0
		stop_vel.angular.z = 0
        	self.set_velocity.publish(stop_vel)
        	rospy.sleep(1)

		diffT = 0
		vel.linear.x = 0
		vel.angular.z = math.pi/2

		if desired_phi == math.pi:
		    desired_phi = -math.pi/2
		else:
		    desired_phi = desired_phi + math.pi/2
	    else:
	    	vel.linear.x = .5
	    	vel.angular.z = omega

	    print(currT.to_sec(), diffT)
	    prevT = currT		
            self.set_velocity.publish(vel) #published command to robot
            rate.sleep()
	#END LAB6 CODE
                        
        
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
