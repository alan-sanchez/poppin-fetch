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

# Import from messages
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryFeedback,FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction,PointHeadFeedback, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
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
        self.fast_client = actionlib.SimpleActionClient("/arm_with_torso_controller/follow_joint_trajectory" ,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for Joint trajectory...")
        self.fast_client.wait_for_server()
        rospy.loginfo("...connected")

        # Set the names of the joints
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        self.scene = PlanningSceneInterface("base_link")
        self.scene.removeCollisionObject("keepout")
        self.scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

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
    def fast_move_to(self,positions, duration = 1, base_motion = None, head_motion = False):
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

        if self.head_motion == True:
            self.head_action.look_at(1.0, 0 ,1.1, duration = .4)

        else:
            pass


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        # Setup action client for the head movement
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

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

        # Condtions to move fetch while head is moving.
        start_time = time.time()
        total_time = 0
        while total_time < duration:
            if self.base_motion == "Forward":
                self.base_action.move_forward(1)
            elif self.base_motion == "Backward":
                self.base_action.move_backward(1)
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
        self.twist.angular.z = 3

        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        # Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0


    def right_turn(self, iter):
        # Set angular rotation around z access to -1 (turn right/CW)
        self.twist.angular.z = -3

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
    arm_action.safe_move_to([.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], velocity = .5)
    head_action.look_at(1.0, -0.5, 1.2, duration = 1)
    rospy.sleep(1)

    # Start motions
    base_action.move_backward(2)
    head_action.look_at(0.2, -1.0, 1.2, duration = .3)
    head_action.look_at(1.0, 0.0, 1.2, duration = .5, base_motion = "Backward")
    base_action.right_turn(13)
    rospy.sleep(.2)
    arm_action.fast_move_to([.3, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .5)
    base_action.move_backward(6)
    rospy.sleep(.7)
    
    # Forward Motion with height change
    arm_action.fast_move_to([.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .5)
    base_action.move_backward(2)
    arm_action.fast_move_to([.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .5)
    rospy.sleep(.4)
    base_action.move_forward(5)
    rospy.sleep(.4)
    arm_action.fast_move_to([.34, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .2)
    base_action.move_forward(5)
    rospy.sleep(.4)
    arm_action.fast_move_to([.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .4)
    arm_action.fast_move_to([.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .4)
    arm_action.fast_move_to([.34, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .4)
    arm_action.fast_move_to([.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .5)
    rospy.sleep(.4)

   # Moonwalk
    base_action.move_backward(5)
    arm_action.fast_move_to([.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .3)
    base_action.move_backward(5)
    arm_action.fast_move_to([.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0], duration = .3)
    base_action.move_backward(10)
