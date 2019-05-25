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

# TO convert the raw (quaternion) angles to euler angles
from tf.transformations import euler_from_quaternion

# Creating a template for robot
class Robot:

	# Function to initialize all the related attributes that the robot should have
	def __init__(self, goal_x, goal_y):
		
		# Creating a unique node that can be subscribed exclusively for potential field planning purpose
		rospy.init_node('avoider1', anonymous = True)

		# Publishing the velocity message of the robot
		self.vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		
		# Subscribing to the odometry message to know the odom of the robot
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

		# Subscribing to the laser scan message
		self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

		# Initialize fields for storing goal coordinates
		self.goal_y = goal_y
		self.goal_x = goal_x

		# Initialize a field for robot's target heading
		self.target_heading = []

	# Setting initial roll, pitch and yaw angles to 0 thereby avoiding offsets	
	roll = pitch = yaw = 0.0

	# Function to call to get the current position from odometry topic of the robot
	def odom_callback(self,msg):

		# Get the current position (x,y) coordinates of robot
		self.pose_x = round(msg.pose.pose.position.x, 3)
		self.pose_y = round(msg.pose.pose.position.y, 3)

		# Get the angle at which target is present for the robot to turn
		target_heading = self.get_heading(msg)

	# Function to calculate the roll, pitch and yaw angles of the robot
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
		if self.target_heading < 0:
			self.target_heading = self.target_heading + 2*pi		

		# Convert degrees to radians	
		if yaw < 0:
			yaw = yaw + 2*pi

		return self.target_heading

	def scan_callback(self,msg):
		
		# proportinality constant for angular velocity along z
		ki = 2.5

		# Proportionality constant for potential field strength
		kp = 0.01

		# Since the laserscan runs first, check if there is any attributes related to it
		if not (self.target_heading):
			return
		
		# Difference in angle between current heading and target heading
		theta = self.target_heading - yaw
		
		# If the scan range is between 310 to 620
		if self.target_heading < yaw and theta + 2*pi < pi:
			rot_z_speed = theta + 2*pi * ki

		# if the scan rangei between 0 to 310
		elif ((self.target_heading > yaw and theta < pi) or (self.target_heading < yaw and theta < pi)):
			rot_z_speed =  theta * ki
		
		else:
			rot_z_speed = -theta + 2*pi * ki
		
		# Go through the scope of the scan and calculate all the repulsive vectors
		for i in range(len(msg.ranges)):
			
			# List of all beams within the scope of the scan
			distance = msg.ranges[i]

			# Obstacle is detected when laserscan intensity is received
			if msg.intensities[i] > 0:

				# Adjust the angle towards which the robot has to turn so that it avoids the obstacle
				rot_z_speed = rot_z_speed - kp * (msg.angle_min + i * msg.angle_increment) / (distance**2) 
		
		# Move the robot according to the force vectors as generated above
		self.motion(rot_z_speed,theta)

		# To check what is the current position
		print(" current x = {0}, current y = {1}".format(self.pose_x,self.pose_y))
	
	# Funtion to define the motion of the robot
	def motion(self, rot_z_speed, theta):
		
		# Subscribing to Twist message to provide movement commands
		move = Twist()

		# Check if the robot is already on goal position
		if abs(self.goal_y - self.pose_y) < 0.4 and abs(self.goal_x - self.pose_x) < 0.4:
			return
		
		# Do this while robot is away from goal position
		else:
			# move robot forward at 0.2 m/s
			move.linear.x = 0.2
			# turn robot around at the same position at desired speed
			move.angular.z = rot_z_speed

		# Publish the details of velocity parameters on the vel_pub node	
		self.vel_pub.publish(move)


if __name__ == '__main__':

	# Creating an instance of the robot class
	x = Robot(goal_x = 8, goal_y = 5)
	
	# To stop the node when ctrl + C is pressed
	rospy.spin()
		
