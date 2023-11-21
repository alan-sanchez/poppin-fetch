#!/usr/bin/env python

# Import what we need
import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

# Import from messages
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal
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

    # Function that plans and moves fetch around the "keepout" object.
    # Function takes both the arm_and_torso joint positions and velocity arguments.
    def safe_move_to(self, positions, velocity=1):
        # Execute motion with the moveToJointPosition function.
        while not rospy.is_shutdown():
            result = self.safe_client.moveToJointPosition(self.joint_names,
                                                     positions,
                                                     0.0,
                                                     max_velocity_scaling_factor=velocity)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.scene.removeCollisionObject("keepout")
                return

    # This function allows the fecth to move quicker.
    # WARNING: This does not plan around objects.
    def fast_move_to(self,positions, duration = 1):
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

        self.fast_client.send_goal(follow_goal)
        self.fast_client.wait_for_result()


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        # Setup action client for the head movement
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
        self.twist.angular.z = 1.0
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
            
        # Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0

    def right_turn(self, iter):
        # Set angular rotation around z access to -1 (turn right/CW)
        self.twist.angular.z = -1.0
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()

        # Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0

if __name__ == "__main__":
    # Create a node
    rospy.init_node("intro_dance", anonymous = False)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    arm_action = FollowTrajectoryClient()
    head_action = PointHeadClient()
    base_action = FootWork()

    # init configuration
    head_action.look_at(0.0,0.0,0.0, duration = 1)
    body_action.safe_move_to([0.39, 1.17, 1.52, 1.47, -0.80,  0.00,  0.00, 0.00], velocity = .2)

    # Begin Movements
    head_action.look_at(1.0,0.0,1.3, duration = .5)
    arm_action.fast_move_to([0.39, 1.17, 1.52, 1.47, -0.80,  0.00,  0.00, 0.00], duration = 1.0)
    arm_action.fast_move_to([0.39, 1.17, 1.52, 1.47, -1.57,  0.00,  0.00, 0.00], duration = 1.0)
    arm_action.fast_move_to([0.39, 1.46, 1.52, 1.47, -1.57,  0.00,  0.00, 0.00], duration = 1.0)
    base_action.left_turn(8)
    base_action.right_turn(10)
    base_action.left_turn(10)
    base_action.right_turn(20)
    rospy.sleep(1)
    base_action.left_turn(12)
    arm_action.fast_move_to([0.39, 1.46, 1.52, 1.47, -1.57,  0.00,  0.00, 0.00], duration = 1.0)
    head_action.look_at(0.1,1.0,1.3, duration = .5)
    arm_action.safe_move_to([0.39, 1.55, 0.00, 1.57,  0.00, -1.57, -1.57, 0.00], velocity = 1.0)
    arm_action.fast_move_to([0.39, 1.55, 0.00, 1.57,  0.00,  0.00, -1.57, 0.00], duration = 1.0)
    arm_action.fast_move_to([0.39, 1.55, 0.00, 1.57,  0.00,  1.57, -1.57, 0.00], duration = 1.0)
    arm_action.fast_move_to([0.39, 1.55, 0.00, 1.57,  0.00,  3.12, -1.57, 0.00], duration = 1.0)
    arm_action.fast_move_to([0.39, 1.55, 0.00, 1.57,  0.00, -1.63, -1.57, 0.00], duration = 1.0)
    arm_action.fast_move_to([0.39, 1.55, 0.00, 1.57,  0.00,  1.63, -1.57, 0.00], duration = 2.0)
    rospy.sleep(.1)
    head_action.look_at(1.0,0.0,1.3, duration = .7)
    arm_action.fast_move_to([0.39, 0.60, 0.00, 1.57, -2.16,  1.57, -1.57, 0.00], duration =  1.5)
    arm_action.fast_move_to([0.39, 0.60, 0.00, 1.57, -2.16,  1.57,  0.00, 0.00], duration =  1.0)
    arm_action.fast_move_to([0.39, 1.55, 0.00, 1.57, -1.57,  1.57,  0.00, 0.00], duration =  1.0)
    arm_action.fast_move_to([0.27, 1.55, 0.00, 1.57, -1.57,  1.57,  0.00, 0.00], duration =  1.0)
    rospy.sleep(.1)
    head_action.look_at(0.0,0.0,0.0, duration = .5)
