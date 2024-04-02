#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(data):
	# Create a Twist message
	twist_msg = Twist()

	vel = 0.1

	# Map joystick axes to linear and angular velocities
	# twist_msg.linear.x = data.axes[1]
	# twist_msg.linear.y = -data.axes[0]
	# twist_msg.angular.z = data.axes[3]



	# Check buttons for additional control
	if data.buttons[4]:  # Button 4 for on-the-spot rotation (counter-clockwise)
		twist_msg.angular.z = vel
	elif data.buttons[5]:  # Button 5 for on-the-spot rotation (clockwise)
		twist_msg.angular.z = -1*vel
	elif data.buttons[0]:
		twist_msg.linear.y = -1*vel
	elif data.buttons[1]:
		twist_msg.linear.x = vel
	elif data.buttons[2]:
		twist_msg.linear.x = -1*vel
	elif data.buttons[3]:
		twist_msg.linear.y = vel

	# Publish the Twist message
	twist_pub.publish(twist_msg)

if __name__ == '__main__':
	# Initialize the ROS node
	rospy.init_node('joy_to_twist')

	# Create a subscriber for the Joy message
	joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

	# Create a publisher for the Twist message
	twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Spin the node
	rospy.spin()
