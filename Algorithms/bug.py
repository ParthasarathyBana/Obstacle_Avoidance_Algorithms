#!/usr/bin/env python

# Python dependancy in ROS 
import rospy

# Importing math library to perform trigonometric operations
from math import *

# The velocity command message
from geometry_msgs.msg import Twist

# The odometry command message
from nav_msgs.msg import Odometry

# The laser scan message
from sensor_msgs.msg import LaserScan

# TO convert the raw angles to euler angles
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Robot:

	def __init__(self, goal_x, goal_y):
		
		# Creating a unique node that can be subscribed exclusively for potential field planning purpose
		rospy.init_node('bug', anonymous = True)

		# Publishing the velocity message of the robot
		self.vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		
		# Subscribing to the odometry message to know the odom of the robot
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

		# Subscribing to the laser scan message
		self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

		# Initialize fields for storing goal coordinates
		self.goal_y = goal_y
		self.goal_x = goal_x

		# Maintain a flag and counter to keep track of robot action (whether to circum-navigate or go to goal)
		self.flag = True
		self.counter = 0

		# Initialize a field for robot's target heading
		self.target_heading = []


	def odom_callback(self, msg):
		# Get the current position (x,y) coordinates of robot
		self.pose_x = round(msg.pose.pose.position.x, 3)
		self.pose_y = round(msg.pose.pose.position.y, 3)
		
		# Get the current heading of the robot
		self.target_heading = self.get_heading(msg)

	def scan_callback(self, msg):
		
		# Since the laserscan runs first, check if there is any attributes related to robot instance
		if not (self.target_heading):
			return

		# Check if the robot is already on goal position
		if abs(self.goal_y- self.pose_y) < 0.4 and abs(self.goal_x - self.pose_x) < 0.4:
			return

		rot_z_speed = self.status_check(msg)

		self.motion(rot_z_speed)

	def motion(self, rot_z_speed):
		
		# Subscribing to Twist message to provide movement commands
		move = Twist()

		# Do not move the robot in forward unless it is facing the goal direction 
		if abs(rot_z_speed) > 0.1 and self.flag == True :
			move.linear.x = 0.0
		
		# Do this while robot is away from goal position
		else:
			# move robot forward at 2 m/s
			move.linear.x = 0.3
			self.counter += 1

		# turn robot around at the same position at desired speed
		move.angular.z = rot_z_speed

		# Publish the details of velocity parameters on the vel_pub node	
		self.vel_pub.publish(move)

	def get_heading(self,msg):
		
		# Making these parameters to be visible for everyone
		global roll,pitch,yaw

		# Get the current raw x,y,z values of robot from odometry
		self.orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		
		# Convert the raw angles to human understandable euler angles
		(roll, pitch, yaw) = euler_from_quaternion(self.orientation_list)

		# Get the target heading towards which the robot has to turn
		self.target_heading = atan2((self.goal_y - self.pose_y), (self.goal_x - self.pose_x)) - yaw 
		
		# Convert degrees to radians	
		if yaw < 0:
			yaw = yaw + 2*pi

		# Distance of robot from the line connecting start and goal	
		self.deviation = abs((self.pose_x*self.goal_y)- self.pose_y*self.goal_x)/sqrt(self.goal_x**2 + self.goal_y**2)

		return self.target_heading

	def status_check(self,msg):
	
		rot_z_speed = 0
		kp = 0
		ki = 1

		# Angle at which the goal is located for the robot to turn
		theta = self.target_heading - yaw
		
		# Condition to check if the robot is near any object to go around it 
		if self.flag == True and self.counter >= 15 and min(msg.ranges) <= 1.2:
			self.flag = False
			self.counter = 0

		# Condition to check if the robot is free to go towards goal
		elif self.flag == False and self.deviation <= 0.1 and self.counter >= 35:
			self.flag = True
			self.counter = 0	
	
		# Condition when robot has circum-navigated the object and needs to move to goal
		elif self.flag == False:

			# Go through the scope of the scan to calculate repulsive vectors 
			for i in range(len(msg.ranges)):
				# List of all beams in the scope of the scan
				distance = msg.ranges[i]
				
				if msg.intensities[i] > 0:
					if distance < 1.2:
						kp = 1
						rot_z_speed = msg.angle_min + i*msg.angle_increment
						break;
			if kp == 1:
				rot_z_speed = rot_z_speed - 0.8
			else:
				rot_z_speed = rot_z_speed + 0.20
		
		# Condition to check if there is not obstacle in front and the robot has to go to goal
		elif self.flag == True:
			# If the scan range is between 310 to 620
			if theta < yaw and self.target_heading + 2*pi < pi:
				rot_z_speed = self.target_heading + 2*pi * ki

			# if the scan rangei between 0 to 310
			elif ((theta > yaw and self.target_heading < pi) or (theta < yaw and self.target_heading < pi)):
				rot_z_speed = self.target_heading * ki
			
			else:
				rot_z_speed = rot_z_speed - self.target_heading + 2*pi * ki

		return rot_z_speed


if __name__ == '__main__':

	# Creating an instance of the robot class
	x = Robot(goal_x = 8, goal_y = 5)

	# To stop the node when ctrl + C is pressed
	rospy.spin()