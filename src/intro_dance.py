#!/usr/bin/env python

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moveit_python import PlanningSceneInterface

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Twist, Point
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        self.scene = PlanningSceneInterface("base_link")
        self.scene.removeCollisionObject("keepout")
        self.scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

    def move_to(self, positions, velocity=1):
        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     positions,
                                                     0.0,
                                                     max_velocity_scaling_factor=velocity)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.scene.removeCollisionObject("keepout")
                return

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

class FootWork(object):

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.twist = Twist()
		self.twist.linear.x = 0.0
		self.twist.linear.y = 0.0
		self.twist.linear.z = 0.0
		self.twist.angular.x = 0.0
		self.twist.angular.y = 0.0
		self.twist.angular.z = 0.0

    def left_turn(self, iter):
        for i in iter:
            self.twist.angular.y = 1.0
            self.pub.publish(self.twist)

    def right_turn(self, twist_msg, iter):
        for i in iter:
            self.twist.angular.y = -1.0
            self.pub.publish(self.twist)

if __name__ == "__main__":
    # Create a node
    rospy.init_node("intro_dance")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

# ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
# "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    # Setup clients
    body_action = FollowTrajectoryClient()
    head_action = PointHeadClient()
    base_action = FootWork()

    # body_action.move_to([0.29, 1.17, 1.52, 1.47, -0.80,  0.00,  0.00, 0.00], velocity = 0.5)
    #
    # body_action.move_to([0.29, 1.17, 1.52, 1.47, -0.80,  0.00,  0.00, 0.00], velocity = 1.0)
    # body_action.move_to([0.29, 1.17, 1.52, 1.47, -1.57,  0.00,  0.00, 0.00], velocity = 1.0)
    # body_action.move_to([0.29, 1.46, 1.52, 1.47, -1.57,  0.00,  0.00, 0.00], velocity = 1.0)

    base_action.left_turn(10)


    # body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00, -1.57, -1.57, 0.00], velocity = 1.0)
    # body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  0.00, -1.57, 0.00], velocity = 1.0)
    # body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  1.57, -1.57, 0.00], velocity = 1.0)
    # body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  3.12, -1.57, 0.00], velocity = 1.0)
    # body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00, -1.57, -1.57, 0.00], velocity = 1.0)
    # body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  1.57, -1.57, 0.00], velocity = 1.0)
    # # body_action.move_to([0.29, 1.55, 0.00, 1.57, -1.56,  1.57, -1.57, 0.00], duration = .8)
    #
    # body_action.move_to([0.29, 0.60, 0.00, 1.57, -2.16,  1.57, -1.57, 0.00], velocity =  1.0)
    # body_action.move_to([0.29, 0.60, 0.00, 1.57, -2.16,  1.57,  0.00, 0.00], velocity =  1.0)
    # body_action.move_to([0.29, 1.32, 1.40, -0.20, 1.72,  0.00,  1.66, 0.00], velocity =  .5)
