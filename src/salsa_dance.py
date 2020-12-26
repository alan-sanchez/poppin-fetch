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
from moveit_python import PlanningSceneInterface

# Import from messages
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
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

        if self.head_motion == "move":
            self.head_action.look_at(0, 0 ,0, frame = "gripper_link", duration = 1)

        else:
            pass
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

    def cumbia(self, iter):
        ang_cmds = [1.0, -1.0, -1.0, 1.0]
        lin_cmds = [-0.05, 0.05, -0.05, 0.05]

        # Set angular rotation around z access to 1 (turn left/CCW)

        for i in range(len(ang_cmds)):
            self.twist.angular.z = ang_cmds[i]
            self.twist.linear.x  = lin_cmds[i]
            for it in range(iter):
                self.pub.publish(self.twist)
                self.rate.sleep()

        # Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0
        self.twist.linear.x  = 0.0

    def move_forward(self,iter):
        # Set angular rotation around z access to -1 (turn right/CW)
        self.twist.linear.x = 0.5
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()

        # Reset the angular rotation value to be zero
        self.twist.linear.x = 0.0

    def move_backward(self,iter):
        # Set angular rotation around z access to -1 (turn right/CW)
        self.twist.linear.x = -0.5
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()

        # Reset the angular rotation value to be zero
        self.twist.linear.x = 0.0



if __name__ == "__main__":
    # Create a node
    rospy.init_node("salsa_dance", anonymous = False)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    body_action = FollowTrajectoryClient()
    head_action = PointHeadClient()
    base_action = FootWork()

    # init configuration
    rospy.sleep(.5)
    head_action.look_at(1.0, 0.0, 1.2, duration = 1)
    body_action.safe_move_to([0.35, 1.49, 1.00, -1.36, 1.76,  0.50,  0.42, 0.00], velocity = .5)
    rospy.sleep(1)

    # 2 basics
    for i in range(2):
        body_action.fast_move_to([0.35, 1.59, 1.00, -1.36, 1.66,  0.50,  0.42, 0.00], duration = .5, base_motion = "Forward")
        body_action.fast_move_to([0.35, 1.49, 1.00, -1.36, 1.76,  0.50,  0.42, 0.00], duration = .5, base_motion = "Backward")
        rospy.sleep(.5)
        body_action.fast_move_to([0.35, 1.40, 1.00, -1.36, 1.86,  0.50,  0.42, 0.00], duration = .5, base_motion = "Backward")
        body_action.fast_move_to([0.35, 1.49, 1.00, -1.36, 1.76,  0.50,  0.42, 0.00], duration = .5, base_motion = "Forward")
        rospy.sleep(.5)

    # rospy.sleep(2)
    head_action.look_at(0.2, -1.0, 1.2, duration = 1)


    # 3 cumbias
    rospy.sleep(2)
    head_action.look_at(1.0, 0.0, 1.2, duration = 1)
    body_action.fast_move_to([0.35, 1.59, 1.00, -1.36, 2.00,  0.50,  0.62, 0.00], duration = 1)
    for i in range(3):
        base_action.cumbia(8)
    rospy.sleep(2)

    # Fetch Challenges human
    head_action.look_at(0.2, -1.0, 1.2, duration = 1)
    head_action.look_at(0.2, -1.0, 1.6, duration = .7)
    head_action.look_at(0.2, -1.0, 1.2, duration = .7)
    rospy.sleep(3)

    # shame pose
    body_action.fast_move_to([0.35, 0.74, 0.22, -1.72, 2.20, -1.11, 1.41, 1.34], duration = 1, head_motion = "move")
    head_action.look_at(1.0, -0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0,  0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0, -0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0,  0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0, -0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0,  0.1, 1.0, duration = 0.5)
    body_action.fast_move_to([0.35, 0.90, 0.55, -1.72, 2.20, -1.11, 1.20, 1.34], duration = 1)
    head_action.look_at(0.2,-1.0,1.2, duration = 1)
    body_action.fast_move_to([0.35, 0.74, 0.22, -1.72, 2.20, -1.11, 1.41, 1.34], duration = 1, head_motion = "move")
    head_action.look_at(1.0, -0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0,  0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0, -0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0,  0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0, -0.1, 1.0, duration = 0.5)
    head_action.look_at(1.0,  0.1, 1.0, duration = 0.5)
