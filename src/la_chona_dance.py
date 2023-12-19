#!/usr/bin/env python

## Import what we need
import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

## Import from message types
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction,PointHeadFeedback, PointHeadGoal
from control_msgs.msg import GripperCommandGoal, GripperCommandAction, GripperCommandFeedback

from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Twist, Point
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FollowTrajectoryClient(object):
    def __init__(self):
        ## Setup action client that plans around objects. This is ideal for getting robot to move to it's initial position
        rospy.loginfo("Waiting for MoveIt...")
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        ## Setup action client that allows us to use the feedback callback
        self.fast_client = actionlib.SimpleActionClient("/arm_with_torso_controller/follow_joint_trajectory" ,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for Joint trajectory...")
        self.fast_client.wait_for_server()
        rospy.loginfo("...connected")

        ## Set the names of the joints
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        ## Set up action client for gripper
        rospy.loginfo("Waiting for Gripper client")
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected")

        ## gripper params
        self.gripper_closed_pos = 0  # The position for a fully-closed gripper (meters).
        self.gripper_open_pos = 0.10  # The position for a fully-open gripper (meters).

        ## Padding does not work (especially for self collisions)
        ## So we are adding a box above the base of the robot
        self.scene = PlanningSceneInterface("base_link")
        self.scene.removeCollisionObject("keepout")
        self.scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        self.base_action = FootWork()
        self.head_action = PointHeadClient()

    ## Function that plans and moves fetch around the "keepout" object.
    ## Function takes both the arm_and_torso joint positions and velocity arguments.
    def safe_move_to(self, positions, velocity=1):
        ## Execute motion with the moveToJointPosition function.
        ## while not rospy.is_shutdown():
        result = self.move_group.moveToJointPosition(self.joint_names,
                                                 positions,
                                                 0.0,
                                                 max_velocity_scaling_factor=velocity)

    ## This function allows the fecth to move quicker.
    ## WARNING: This does not plan around objects.
    def fast_move_to(self,positions, duration = 1, base_motion = None, head_motion = None):
        self.base_motion = base_motion
        self.head_motion = head_motion

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

        self.fast_client.send_goal(follow_goal,feedback_cb=self.feedback_callback)
        self.fast_client.wait_for_result()


    def feedback_callback(self,feedback):
        if self.base_motion == "Forward":
            self.base_action.move_forward(1)

        elif self.base_motion == "Backward":
            self.base_action.move_backward(1)

        elif self.base_motion == "Left_turn":
            self.base_action.left_turn(1)

        elif self.base_motion == "Right_turn":
            self.base_action.right_turn(1)

        elif self.base_motion == "Wide_turn":
            self.base_action.wide_turn(1)

        if self.head_motion == "move":
            self.head_action.look_at(0, 0 ,0, frame = "gripper_link", duration = 1)

        else:
            pass

    def open_gripper(self):
        ## Functions opens Grippers
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_open_pos
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()


    def close_gripper(self):
        ## Functions closses Grippers
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_closed_pos
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

## Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        ## Setup action client for the head movement
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame = "base_link", duration=1.0):
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
        ## Initialize publisher.The Fetch will listen for Twist messages on the cmd_vel topic.
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        ## Set up a twist message
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        ## Limit the publication rate.
        self.rate = rospy.Rate(10)

    def wide_turn(self, iter):
        ## Set angular rotation around z access to 1 (turn left/CCW)
        self.twist.angular.z = -1
        self.twist.linear.x = 0.6

        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        ## Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0

    def left_turn(self, iter):
        ## Set angular rotation around z access to 1 (turn left/CCW)
        self.twist.angular.z = 1.0

        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        ## Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0


    def right_turn(self, iter):
        ## Set angular rotation around z access to -1 (turn right/CW)
        self.twist.angular.z = -.90

        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        ## Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0

    def move_forward(self,iter):
        ## Set angular rotation around z access to -1 (turn right/CW)
        self.twist.linear.x = 0.2
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()

        ## Reset the angular rotation value to be zero
        self.twist.linear.x = 0.0

    def move_backward(self,iter):
        ## Set angular rotation around z access to -1 (turn right/CW)
        self.twist.linear.x = -0.2
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()

        ## Reset the angular rotation value to be zero
        self.twist.linear.x = 0.0



if __name__ == "__main__":
    ## Create a node
    rospy.init_node("la_chona_dance", anonymous = False)

    ## Make sure sim time is working
    while not rospy.Time.now():
        pass

    ## Setup clients
    arm_action = FollowTrajectoryClient()
    head_action = PointHeadClient()
    base_action = FootWork()


    ## init configuration
    rospy.sleep(.5)
    head_action.look_at(1.0, 0.0, 1.2, duration = 1)
    arm_action.safe_move_to([0.3, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], velocity = .5)
    rospy.sleep(1)

    ## 4 basics moving forward
    for i in range(4):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Forward")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Forward")

    ## 4 basics moving forward
    for i in range(4):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Backward")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Backward")

    ## 2 loops of right and left turns
    for i in range(2):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Right_turn")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Right_turn")
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Left_turn")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Left_turn")

    ## 2 basics turning right
    for i in range(2):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Right_turn")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Right_turn")

    ## 2 basics moving forward
    for i in range(2):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Forward")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Forward")

    ## 2 basics turning left
    for i in range(3):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Left_turn")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Left_turn")

    ## 2 basics moving forward
    for i in range(3):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Forward")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Forward")

    ## 2 basics turning right
    for i in range(2):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Right_turn")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Right_turn")

    ## 3 basics moving backward
    for i in range(2):
        arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Backward")
        arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4, base_motion = "Backward")


    rospy.sleep(.4)

    ## Only 3 movements for torso. Goes better with song
    arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4)
    arm_action.fast_move_to([0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4)
    arm_action.fast_move_to([0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0], duration = 0.4)

    ## Begin wide left turn while changing new arm configuration
    arm_action.fast_move_to([0.3, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0], duration = 4.0, base_motion = "Wide_turn")

    rospy.sleep(.4)

    ## 7 basics turning left
    for i in range(7):
        arm_action.fast_move_to([0.28, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0], duration = 0.4, base_motion = "Left_turn")
        arm_action.fast_move_to([0.3, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0], duration = 0.4, base_motion = "Left_turn")

    ## 4 basics moving forward
    for i in range(4):
        arm_action.fast_move_to([0.28, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0], duration = 0.4, base_motion = "Forward")
        arm_action.fast_move_to([0.3, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0], duration = 0.4, base_motion = "Forward")

    ## Grab sobrero and remove
    arm_action.fast_move_to([0.3, -1.33, 0.04, 2.64, 1.94, 0.0, 1.03, 0.69], duration = 1)
    arm_action.close_gripper()
    arm_action.fast_move_to([0.3, -1.31, -0.29, 2.64, 1.33, 0.0, 0.59, 0.69], duration = 1)

    rospy.sleep(5)
    arm_action.open_gripper()
