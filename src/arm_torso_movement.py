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
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("/arm_with_torso_controller/follow_joint_trajectory" ,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_with_torso...")
        self.client.wait_for_server()
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]



    def move_to(self, positions, duration=1):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        scene = PlanningSceneInterface("base_link")
        scene.removeCollisionObject("keepout")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)
        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

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



if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

# ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
# "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    # Setup clients
    body_action = FollowTrajectoryClient()
    head_action = PointHeadClient()
    body_action.move_to([0.29, 1.17, 1.52, 1.47, -0.80,  0.00,  0.00, 0.00], duration =  5)

    body_action.move_to([0.29, 1.17, 1.52, 1.47, -0.80,  0.00,  0.00, 0.00], duration = .8)
    body_action.move_to([0.29, 1.17, 1.52, 1.47, -1.57,  0.00,  0.00, 0.00], duration = .8)
    body_action.move_to([0.29, 1.46, 1.52, 1.47, -1.57,  0.00,  0.00, 0.00], duration = .8)

    body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00, -1.57, -1.57, 0.00], duration =  3)
    body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  0.00, -1.57, 0.00], duration = .8)
    body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  1.57, -1.57, 0.00], duration = .8)
    body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  3.12, -1.57, 0.00], duration = .8)
    body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00, -1.57, -1.57, 0.00], duration = .8)
    body_action.move_to([0.29, 1.55, 0.00, 1.57,  0.00,  1.57, -1.57, 0.00], duration = 1.2)
    # body_action.move_to([0.29, 1.55, 0.00, 1.57, -1.56,  1.57, -1.57, 0.00], duration = .8)

    body_action.move_to([0.29, 0.60, 0.00, 1.57, -2.16,  1.57, -1.57, 0.00], duration =  3)
    body_action.move_to([0.29, 0.60, 0.00, 1.57, -2.16,  1.57,  0.00, 0.00], duration =  .8)
    body_action.move_to([0.29, 1.32, 1.40, -0.20, 1.72,  0.00,  1.66, 0.00], duration =  3)
