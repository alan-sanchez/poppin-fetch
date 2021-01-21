#!/usr/bin/env python

# Import what we need
import copy
import actionlib
import rospy
import time

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moveit_python import PlanningSceneInterface

# Import from command messages
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryFeedback,FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction,PointHeadFeedback, PointHeadGoal
from control_msgs.msg import GripperCommandGoal, GripperCommandAction, GripperCommandFeedback

from geometry_msgs.msg import PoseStamped, Twist, Point
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FollowTrajectoryClient(object):
    def __init__(self):
        # Setup action client that plans around objects
        rospy.loginfo("Waiting for MoveIt...")
        self.safe_client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Setup action client that moves the fetch "quicker" than the previous client
        rospy.loginfo("Waiting for Joint trajectory...")
        self.fast_client = actionlib.SimpleActionClient("/arm_with_torso_controller/follow_joint_trajectory" ,FollowJointTrajectoryAction)
        self.fast_client.wait_for_server()
        rospy.loginfo("...connected")


        # Set up action client for gripper
        rospy.loginfo("Waiting for Gripper client")
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected")

        # gripper params
        self.gripper_closed_pos = 0  # The position for a fully-closed gripper (meters).
        self.gripper_open_pos = 0.10  # The position for a fully-open gripper (meters).


        # Set the names of the joints
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        self.scene = PlanningSceneInterface("base_link")
        self.scene.removeCollisionObject("keepout")
        self.scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        # Include base and head clients to this class
        self.base_action = FootWork()
        self.head_action = PointHeadClient()


    # Function that plans and moves fetch around the "keepout" object.
    # Function takes both the arm_and_torso joint positions and velocity arguments.
    def safe_move_to(self, positions, velocity=1):
        # Execute motion with the moveToJointPosition function.
        # while not rospy.is_shutdown():
        result = self.safe_client.moveToJointPosition(self.joint_names,
                                                 positions,
                                                 0.0,
                                                 max_velocity_scaling_factor=velocity)


    # This function allows the fecth to move quicker.
    # WARNING: This does not plan around objects.
    def fast_move_to(self,positions, duration = 1, base_motion = None, head_frame = None, head_pose = None):
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

        # Send goal to action server
        self.fast_client.send_goal(follow_goal)


        # condtions to move fetch base or head while arm is moving.
        start_time = time.time()
        total_time = 0
        while total_time < duration:
            if base_motion == "Forward":
                self.base_action.move_forward(1)

            elif base_motion == "Backward":
                self.base_action.move_backward(1)

            elif base_motion == "Left_turn":
                self.base_action.left_turn(1)

            elif base_motion == "Right_turn":
                self.base_action.right_turn(1)

            if head_frame != None:
                x, y, z = head_pose
                self.head_action.look_at(x, y, z, frame = head_frame, duration = .4)

            # If total time is larger than duration time argument then loop breaks
            total_time = time.time() - start_time

        self.fast_client.wait_for_result()


    def open_gripper(self):
        # Functions opens Grippers
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_open_pos
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()


    def close_gripper(self):
        # Functions closses Grippers
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_closed_pos
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        # Setup action client for the head movement
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

        # Include base clients to this class
        self.base_action = FootWork()

    def look_at(self, x, y, z, frame = "base_link", duration=1.0, base_motion = None):
        self.base_motion = base_motion

        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)

        # condtions to move fetch while head is moving.
        start_time = time.time()
        total_time = 0
        while total_time < duration:
            if self.base_motion == "Forward":
                self.base_action.move_forward(1)

            elif self.base_motion == "Backward":
                self.base_action.move_backward(1)

            elif self.base_motion == "Left_turn":
                self.base_action.left_turn(1)

            elif self.base_motion == "Right_turn":
                self.base_action.right_turn(1)

            else:
                break

            total_time = time.time() - start_time

        self.client.wait_for_result()


class FootWork(object):

    def __init__(self):
        # Initialize publisher.The Fetch will listen for Twist messages on the cmd_vel topic.
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Set up a twist message
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        # Limit the publication rate.
        self.rate = rospy.Rate(10)


    def left_turn(self, iter):
        # Set angular rotation around z access to 1 (turn left/CCW)
        self.twist.angular.z = 1.5

        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        # Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0


    def right_turn(self, iter):
        # Set angular rotation around z access to -1 (turn right/CW)
        self.twist.angular.z = -1.5

        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        # Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0


    def move_forward(self,iter):
        # Set angular rotation around z access to -1 (turn right/CW)
        self.twist.linear.x = 1.0
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        # Reset the angular rotation value to be zero
        self.twist.linear.x = 0.0


    def move_backward(self,iter):
        # Set angular rotation around z access to -1 (turn right/CW)
        self.twist.linear.x = -1.0
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        # Reset the angular rotation value to be zero
        self.twist.linear.x = 0.0



if __name__ == "__main__":
    # Create a node
    rospy.init_node("true_dance", anonymous = False)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    arm_action = FollowTrajectoryClient()
    head_action = PointHeadClient()
    base_action = FootWork()

    # init configuration
    arm_action.safe_move_to([0.38, -0.84, 0.87, -2.3, -2.05, 0.52, -1.6, 0.0], velocity = 0.5)
    head_action.look_at(1.0, -0.0, 1.2, duration = 1)
    rospy.sleep(4)

    # Begin Tutting
    arm_action.fast_move_to([0.38, 0.1, 0.77, -1.9, -1.75, 0.77, -1.68, -0.45], duration = 0.8)
    arm_action.fast_move_to([0.38, 0.1, -0.57, -1.61, -1.53, 2.15, 1.54, 0.13], duration = 1.2)
    arm_action.fast_move_to([0.38, -0.69, 0.01, 0.0, -1.62, 2.28, 1.57, 0.04], duration = 2.0,
                             head_frame = "gripper_link",
                             head_pose = [0.0, 0.0, 0.0])

    # Begin Team Tut
    arm_action.fast_move_to([0.38, -0.69, 0.01, 0.0, -1.62, 0.65, 1.57, 0.04], duration = 0.8)
    arm_action.fast_move_to([0.38, -0.69, 0.01, 0.0, -1.62, 0.65, 0.06, 0.04], duration = 0.8)
    arm_action.fast_move_to([0.38, -0.69, 0.01, -1.53, -2.16, 0.65, 0.06, -0.56], duration = 1.6,
                             head_frame = "gripper_link",
                             head_pose = [0.0, 0.0, 0.0])

    rospy.sleep(2.4)

    # Twist Fetch's Elbow
    arm_action.fast_move_to([0.38, -0.69, 0.01, -1.53, -2.16, 0.65, 0.06, 1.00], duration = 1.2,
                             head_frame = "gripper_link",
                             head_pose = [0.0, 0.0, 0.0])
    arm_action.fast_move_to([0.38, -0.69, 0.01, -1.53, -2.16, 0.65, 0.06, 2.41], duration = 1.2)

    # Fetch Raises hand
    arm_action.fast_move_to([0.38, -0.69, 0.01, 0.04, -1.65, 0.69, 0.06, 3.14], duration = 1.6,
                             head_frame = "gripper_link",
                             head_pose = [0.0, 0.0, 0.0])
    arm_action.fast_move_to([0.38, -0.69, 0.01, 0.04, -1.65, 0.69, 1.66, 3.14], duration = 1.2)
    arm_action.fast_move_to([0.38, -0.69, 0.01, 0.04, -1.65, 2.19, 1.57, 3.14], duration = 1.6,
                             head_frame = "gripper_link",
                             head_pose = [0.0, 0.0, 0.0])

    # Fetch arm in the Center
    arm_action.fast_move_to([0.38, -0.03, 0.01, 0.0, -1.59, -0.04, 1.57, 3.14], duration = 1.2)
    arm_action.fast_move_to([0.38, -0.03, 0.01, 0.0, -1.59, 1.59, -1.54, 3.14], duration = 1.2)
    rospy.sleep(.6)

    # Fetch's tut moves me
    arm_action.fast_move_to([0.38, -0.2, 0.61, 0.0, -2.05, 1.59, -0.03, 3.14], duration = 1.2)
    arm_action.fast_move_to([0.38, -0.65, 0.03, 0.0, -1.65, 2.07, 1.54, -3.1], duration = 1.2)
    arm_action.fast_move_to([0.38, -0.65, 0.03, -1.57, -2.22, 1.55, 0, -3.1], duration = 2.0)

    # Tee shape with arms. Moving wrists
    arm_action.fast_move_to([0.38, -0.65, 0.03, -1.57, -2.22, 1.55, 1.6, -3.1], duration = .8)
    arm_action.fast_move_to([0.38, -0.65, 0.03, -1.57, -2.22, 1.55, -1.57, -3.1], duration = 1.2)
    rospy.sleep(2.4)

    # Bring Fetch's arm out
    arm_action.fast_move_to([0.38, -0.65, 0.03, -1.57, -0.63, 1.55, -1.57, -3.1], duration = 1.6)
    arm_action.fast_move_to([0.38, -0.65, 0.03, -1.57, -0.63, -1.64, -1.57, -3.1], duration = 1.6)

    # Bring Fetch's arm out. Rotate wrist
    arm_action.fast_move_to([0.38, -0.65, 0.03, -1.57, -0.63, 0.0, -1.57, 1.85], duration = 1.6)
    arm_action.fast_move_to([0.38, -0.71, 0.03, -1.57, -2.22, 1.51, -1.57, 0.13], duration = 1.6)
    arm_action.fast_move_to([0.38, -0.71, 0.03, -1.57, -2.22, 1.51, -1.57, 0.13], duration = 1.4)

    # End pose
    arm_action.fast_move_to([0.38, -0.84, 0.87, -2.3, -2.05, 0.52, -1.6, 0.0], duration = 1.0,
                             head_frame = "base_link",
                             head_pose = [0.2, 0.0, 0.0])
