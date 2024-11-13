#!/usr/bin/env python3


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to subscribe to 64-bit integers, so we need to import the defintion
# for them.
from sensor_msgs.msg import JointState


# This is a function that is called whenever a new message is received.  The
# message is passed to the function as a parameter.
def callback(msg):
	"""
	Callback function to deal with incoming messages.
	:param msg: The message.
	"""
	names = list(msg.name)
	positions = list(msg.position)
	# The value of the integer is stored in the data attribute of the message.
	joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
			  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

	joint_positions = []

	for i in range(len(joints)):
		index = names.index(joints[i])
		joint_positions.append(round(positions[index],2))

	print(joint_positions)
	rospy.signal_shutdown("done")
	sys.exit(0)



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('joint_sate_printer', anonymous=True)

	# Set up a subscriber.  We're going to subscribe to the topic "counter",
	# looking for Int64 messages.  When a message comes in, ROS is going to pass
	# it to the function "callback" automatically.
	# for i in range()
	subscriber = rospy.Subscriber('joint_states', JointState, callback)

	# Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages.
	rospy.spin()
