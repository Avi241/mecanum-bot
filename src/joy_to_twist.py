#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(data):
	# Create a Twist message
	twist_msg = Twist()

	# Map joystick axes to linear and angular velocities
	twist_msg.linear.x = data.axes[1]
	twist_msg.linear.y = -data.axes[0]
	twist_msg.angular.z = data.axes[3]

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
