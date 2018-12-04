import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from time import time
import geometry_msgs.msg
import tf
import math
import matplotlib.pyplot as plt
import numpy as np


#these waypoints are given as list for convience, however, you can use any data type that you like
#i.e. np.array()
#as long as you use the same set of way points as below.
#These coordinates are in the "world" coordinate frame

# Point robot up
# [-0.71, 0.71] is the "origin" where the ball is relative to the robot
#waypoints = [[0,0],[0.3,0.5],[0.5,0.71],[0,0.71],[-0.5,0.71],[-0.71,0.71],[-1,0.71],[-1.3,0.71],[-1,0.5],[-0.5,0.2],[0,0]]

#position 3 waypoints
#waypoints = [[0,0],[0.5,0],[1,0],[1.2,0],[1.2,.5],[1.2,1],[1.2,1.5],[1.2,1.7]]

#position 2
#waypoints = [[0,0],[1,0],[1.8,0],[1.75,-.25],[1.2,-.25],[.8,-.25],[.3,-.25],[0,0]]

#waypoints = [[0,0],[1,0],[1.8,0],[1.2,.5],[.5,.5],[0,.5]]
waypoints = [[0,0],[1,0],[1.8,0]]

dthresh = 0.01 # threshold distance from waypoint
x_path = []
y_path = []

class turtlebot_move():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.shutdown)
        self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	
	# reset odometry
        reset_odom = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10)
        timer = time()
        while time()-timer <1.0:
            reset_odom.publish(Empty())

        tfListener = tf.TransformListener()

        vel = Twist()
        vel.linear.x = 0.5
	vel.angular.z = 0
	
	######################################################## Start Lab6 Code

        rate = rospy.Rate(100)

        desired_phi = 0 # desired orientation for robot to point at waypoint
        curr_phi = 0    # current orientation of robot
        diff_phi = 0    # the difference between orientations for P-controller
        kp = 2.5          # P-controller constant

	# lab 6 new vars
        i_way = 0   # index of waypoint

        while not rospy.is_shutdown():
            try:
                (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
            except:
                continue
	    
	    # get x and y position of the current waypoint
            x_way = waypoints[i_way][0]
            y_way = waypoints[i_way][1]

	    # get x and y position of robot
            x_pos = position[0]
            y_pos = position[1]
	    
            # find distance from waypoint
            dist = math.sqrt((x_way - x_pos)**2 + (y_way - y_pos)**2)

            orientation = tf.transformations.euler_from_quaternion(quaternion)
	    # determine which quadrant the waypoint is in with reference to the robot's frame
	    #  2 | 1
	    # -------
	    #  3 | 4
	    # assign temporary point make right triangle for trig calculations
            x_temp = x_way
            y_temp = y_pos
	
            curr_phi = orientation[2]   # get current yaw orientation

	    # check if waypoint and robot on same x
            if(x_pos == x_way):
	        # find angle
                if(y_pos < y_way): # waypoint is directly above robot
                    desired_phi = math.pi/2
                elif(y_pos > y_way): # waypoint is directly below robot
                    desired_phi = -math.pi/2

	    # check if waypoint and robot on same y
            elif(y_pos == y_way):
	        # find angle
                if(x_pos < x_way): # waypoint is directly right robot
                    desired_phi = 0
                elif(x_pos > x_way): # waypoint is directly left robot
                    desired_phi = -math.pi

	    # determine opposite side length of triangle
            else:
                # distance of opposite leg of triangle (for calculating theta)
                dist_tri = abs(y_way - y_temp)
		# find angle theta
                theta = math.asin(dist_tri/dist)
		
                # check which quadrant the way point is in in relation to robot
                # this will determine the desired orientation to reach waypoint
                if(x_way > x_pos):	# in either quadrant 1 or 4
                    if(y_way > y_pos):	# in quadrant 1
                        desired_phi = theta
                    elif(y_way < y_pos): # in quadrant 4
                        desired_phi = -theta
                elif(x_way < x_pos): # in either quadrant 2 or 3
                    if(y_way > y_pos):	# in quadrant 2
                        desired_phi = math.pi - theta
                    elif(y_way < y_pos): # in quadrant 3
                        desired_phi = -math.pi + theta

            diff_phi = desired_phi - curr_phi   # find difference for proportional term

	    if diff_phi < -math.pi:
		diff_phi = diff_phi + 2*math.pi
	    elif diff_phi > math.pi:
		diff_phi = diff_phi - 2*math.pi

            omega = kp * diff_phi   # adjust angular rotation based on P-controller	


            if dist < dthresh:	# if within distance
		# stop robot when reached corner to allow to change orientation
                rospy.loginfo("Stop Action")
                stop_vel = Twist()
                stop_vel.linear.x = 0
                stop_vel.angular.z = 0
                self.set_velocity.publish(stop_vel)
                rospy.sleep(1)

		
                # update waypoint index
                if i_way < len(waypoints)-1: # increment to next waypoint
                    i_way = i_way + 1
                else: # loop back to first waypoint
                    i_way = 0
                print(waypoints[i_way])
		
		#x_path.append(x_pos)
		#y_path.append(y_pos)
	    else:
                vel.linear.x = .1
		if omega > .7:                
		    vel.angular.z = .7
		elif omega < -.7:
		    vel.angular.z = -.7
		else:
		    vel.angular.z = omega
            
	    dPhi = diff_phi * 180 / math.pi
	    currPhiD = curr_phi * 180 / math.pi
	    desPhi = desired_phi * 180/ math.pi
	    print("at index %d, diff_phi = %d",i_way,  currPhiD, desPhi ) 
            self.set_velocity.publish(vel) #published command to robot
            rate.sleep()

	######################################################## End Lab6 Code
                        
        
    def shutdown(self):
	#plt.figure(figsize=(1.5,1.5))	
	#plt.plot(x_path,y_path)
	#plt.show()
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
