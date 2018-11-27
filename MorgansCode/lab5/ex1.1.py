#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from time import time
import geometry_msgs.msg
import tf
import math
from math import pi

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
	
		#START LAB4 CODE
		
		rate = rospy.Rate(100);    #initialize clockspeed TO 100 hertz
		currT = rospy.Time(0);     #obtain initial time
		diffT = 0     #diffT tracks counter of 10 secs before turning
		prevT = 0     #assign currT to prevT for next iteration
		desired_phi = 0 #phi star
		curr_phi = 0    #current orientation
		diff_phi = 0    #difference between desired and current orientation
		prev_diff_phi = 0 #stores previous val for diff phi, for pd controller 
		compare_phi = 0 #stores difference (derivative) between both phi vals  
		kp = .01    #P-controller constant
		kd = .1          #D-controller constant
		kpTurn = .1
		kdTurn = .1
		#rospy.loginfo("position: "+str(position))
		#rospy.loginfo("orientation: "+str(orientation))
		case = 1 #0=stop/1=go/2=turn
		startTime = rospy.Time.now()
		elapsedTime = 0
		desiredTurnAngle = pi/2
		threshold = .1
		while not rospy.is_shutdown():
			currT = rospy.Time.now();  #retrieve updated time each cycle        
                        #print(case) 
                        if prevT == 0:
                                prevT = currT   # reset prevT to minimize jump at beginning of movement
                                diffT = diffT + currT.to_sec() - prevT.to_sec() #diff between clock cycles
                        try:    
                                (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0)) # get position and quaternion orientation of robot
                        except: 
                                continue
                        orientation = tf.transformations.euler_from_quaternion(quaternion)  # convert quaternion to Euler agle orientation in form of [roll, pitch, yaw]
                                
                        curr_phi = orientation[2]   # get current yaw orientation
			prev_diff_phi = diff_phi
                        diff_phi = desired_phi - curr_phi   # find difference for proportional term
                        compare_phi = diff_phi - prev_diff_phi  # find difference for derivative term
			elapsedTime = currT.to_sec() - startTime.to_sec() 
			#print(elapsedTime)	
			print(case)
			if (case == 0):
				#stop case
				#stop for 1 second
				# stop robot when reached corner to allow to change orientation
                                #rospy.loginfo("Stop Action")
                                #stop_vel = Twist()
                                #stop_vel.linear.x = 0
                                #stop_vel.angular.z = 0
				
				vel.linear.x = 0
				vel.angular.z = 0
                                self.set_velocity.publish(vel)
                                #rospy.sleep(1)
				#print(elapsedTime)
				if(elapsedTime >= 1):
					case = 2
					elapsedTime = 0
                                	startTime = rospy.Time.now()	
			elif(case == 1):
				#straight case
				#prevT = currT   # update prevT to be the currT
                                #prev_diff_phi = diff_phi   # update prev_diff_phi to be current diff_phi for derivative
                                #self.set_velocity.publish(vel) #published command to robot
				#compare_phi = diff_phi - prev_diff_phi  # find difference for derivative term
                        	prev_diff_phi = diff_phi
                        	diff_phi = desired_phi - curr_phi   # find difference for proportional term
                        	compare_phi = diff_phi - prev_diff_phi  # find difference for derivative term
				if(elapsedTime <= 10):
					omega = kp * diff_phi + kd * compare_phi    # adjust angular roation based on PD-controller
					print(diff_phi)
					vel.linear.x = .5
					vel.angular.z = omega
					self.set_velocity.publish(vel) #published command to robot
				else:
					case = 0
					startTime = rospy.Time.now()
					elapsedTime = 0
			elif(case == 2):
				#print(curr_phi)
				#turn case
				
				prev_diff_phi = diff_phi
                          	diff_phi = desired_phi - curr_phi + pi/2   # find difference for proportional term
                          	compare_phi = diff_phi - prev_diff_phi  # find difference for derivative term
				omega = kpTurn* diff_phi + kdTurn * compare_phi
				print(omega)
				if(desired_phi == pi/2):
					if((curr_phi <= (pi-threshold)) and(curr_phi >= -pi+threshold )):
						#print(curr_phi)
                                        	vel.linear.x = 0
                                        	vel.angular.z = omega
                                        	self.set_velocity.publish(vel)
					else:
                                        	vel.linear.x = 0
                                        	vel.angular.z = 0
                                        	self.set_velocity.publish(vel)

                                        	case = 1
                                        	elapsedTime = 0
                                        	startTime = rospy.Time.now()
<<<<<<< HEAD
						desired_phi = -pi
				else:
				'''
	
 				if(curr_phi <= desired_phi + pi/2):
					vel.linear.x = 0
                                       	vel.angular.z = pi/2
					self.set_velocity.publish(vel)
					
=======
						desired_phi = pi
				elif(desired_phi == pi):
					if((curr_phi <= (-pi/2 - threshold)) or (curr_phi >= -pi/2 + threshold )):
                                                #print(curr_phi)
                                                vel.linear.x = 0
                                                vel.angular.z = omega
                                                self.set_velocity.publish(vel)
                                        else:
                                                vel.linear.x = 0
                                                vel.angular.z = 0
                                                self.set_velocity.publish(vel)
 
                                                case = 1                                                 
						elapsedTime = 0
                                                startTime = rospy.Time.now()
						desired_phi = -pi/2

>>>>>>> 4db588c9cc33f89a4477ded718cd8038d6986dab
				else:
				
 					if(curr_phi <= desired_phi + pi/2 - threshold):
						vel.linear.x = 0
                                       		vel.angular.z = omega 
						self.set_velocity.publish(vel)
					else:
						vel.linear.x = 0
                                       		vel.angular.z = 0
						self.set_velocity.publish(vel)

						case = 1
						elapsedTime = 0
						startTime = rospy.Time.now()
						if (desired_phi == pi):
                                               		desired_phi = -pi/2
                                    		else:
                                               		desired_phi = desired_phi + pi/2



	
'''	
		while not rospy.is_shutdown():
			currT = rospy.Time.now();  #retrieve updated time each cylce        
			
			if prevT == 0:
				prevT = currT   # reset prevT to minimize jump at beginning of movement
				diffT = diffT + currT.to_sec() - prevT.to_sec() #diff between clock cycles
			try:
				(position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0)) # get position and quaternion orientation of robot
			except:
				continue
			orientation = tf.transformations.euler_from_quaternion(quaternion)  # convert quaternion to Euler agle orientation in form of [roll, pitch, yaw]
	
			curr_phi = orientation[2]   # get current yaw orientation
			diff_phi = desired_phi - curr_phi   # find difference for proportional term
			compare_phi = diff_phi - prev_diff_phi  # find difference for derivative term
			omega = kp * diff_phi + kd * compare_phi    # adjust angular roation based on PD-controller

			if (diffT >= 10):

      				# stop robot when reached corner to allow to change orientation
      				rospy.loginfo("Stop Action")
      				stop_vel = Twist()
				stop_vel.linear.x = 0
				stop_vel.angular.z = 0
				self.set_velocity.publish(stop_vel)
				rospy.sleep(1)
				turn_phi = pi/2
				while (diff_phi >= .1):
					# change orientation by pi/2 for corner
					diffT = 0
					vel.linear.x = 0
					vel.angular.z = math.pi/2
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					
					curr_phi = orientation[2]   # get current yaw orientation
                        		diff_phi = turn_phi - curr_phi   # find difference for proportional term			
					# change desired_phi to next angle; keep within range of [-pi, pi]
					if desired_phi == math.pi:
						desired_phi = -math.pi/2
					else:
						desired_phi = desired_phi + math.pi/2
		
			# move robot forward and rotate at a rate of omega to stay in a straight path
			else:
				vel.linear.x = 0.5
				vel.angular.z = omega

				prevT = currT   # update prevT to be the currT
				prev_diff_phi = diff_phi   # update prev_diff_phi to be current diff_phi for derivative
				self.set_velocity.publish(vel) #published command to robot
				rate.sleep()
		#END LAB4 CODE
		'''
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
