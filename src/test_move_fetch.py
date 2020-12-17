#!/usr/bin/env python

# simple_disco.py: Move the fetch arm through a simple disco motion
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    rospy.init_node("simple_disco")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    # TF joint names
    joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    # Lists of joint angles in the same order as in joint_names
    disco_poses = [[0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
                   [0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
                   [0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

    for pose in disco_poses:
        if rospy.is_shutdown():
            break

        # Plans the joints in joint_names to angles in pose
        move_group.moveToJointPosition(joint_names, pose, wait=False)

        # Since we passed in wait=False above we need to wait here
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Disco!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()
# from __future__ import print_function
# import time
# import roslib#; roslib.load_manifest('ur_driver')
# import rospy
# import actionlib
# from control_msgs.msg import *
# from trajectory_msgs.msg import *
#
#
#
#
# JOINT_NAMES = ['torso_life_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
#                'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'] #'torso_lift_joint'
#
#
# Q1 =  [0.29, 1.49, 1.52, 1.47, -0.80, 0.00, 0.00, 0.00]
#
#
#
# client = None
#
# def move_init():
#     g = FollowJointTrajectoryGoal()
#     g.trajectory = JointTrajectory()
#     g.trajectory.joint_names = JOINT_NAMES
#     g.trajectory.points = [
#         JointTrajectoryPoint(positions=Q1, velocities=[0], time_from_start=rospy.Duration(5.0))]
#     client.send_goal(g)
#     try:
#         client.wait_for_result()
#     except KeyboardInterrupt:
#         client.cancel_goal()
#         raise
#
# def move_repeated():
#     g = FollowJointTrajectoryGoal()
#     g.trajectory = JointTrajectory()
#     g.trajectory.joint_names = JOINT_NAMES
#
#     d = 5.0
#     g.trajectory.points = []
#     for i in range(3):
#         g.trajectory.points.append(
#             JointTrajectoryPoint(positions=Q1, velocities=[0], time_from_start=rospy.Duration(d)))
#         d += 8
#         g.trajectory.points.append(
#             JointTrajectoryPoint(positions=Q2, velocities=[0], time_from_start=rospy.Duration(d)))
#         d += 8
#         g.trajectory.points.append(
#             JointTrajectoryPoint(positions=Q3, velocities=[0], time_from_start=rospy.Duration(d)))
#         d += 8
# 	g.trajectory.points.append(
#             JointTrajectoryPoint(positions=Q4, velocities=[0], time_from_start=rospy.Duration(d)))
#         d += 8
# 	g.trajectory.points.append(
#             JointTrajectoryPoint(positions=Q5, velocities=[0], time_from_start=rospy.Duration(d)))
#         d += 8
# 	g.trajectory.points.append(
#             JointTrajectoryPoint(positions=Q6, velocities=[0], time_from_start=rospy.Duration(d)))
# 	d += 8
#         g.trajectory.points.append(
#             JointTrajectoryPoint(positions=Q7, velocities=[0], time_from_start=rospy.Duration(d)))
#         d += 8
#
#
#     client.send_goal(g)
#     try:
#         client.wait_for_result()
#     except KeyboardInterrupt:
#         client.cancel_goal()
#         raise
#
# def main():
#     global client
#     try:
#         rospy.init_node("test_move", anonymous=False, disable_signals=False)
#         #client = actionlib.SimpleActionClient('/arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
#         client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
#         print("Waiting for server...")
#         client.wait_for_server()
#         print("Connected to server")
#         raw_input('Move to initial position')
# 	move_init()
# 	#raw_input('Run sequence')
#         #qqmove_repeated()
#
#     except KeyboardInterrupt:
#         rospy.signal_shutdown("KeyboardInterrupt")
#         raise
#
# if __name__ == '__main__': main()
