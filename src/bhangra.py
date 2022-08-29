#!/usr/bin/env python

# Import what we need
import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)

# Import from messages
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction,PointHeadFeedback, PointHeadGoal
from control_msgs.msg import GripperCommandGoal, GripperCommandAction, GripperCommandFeedback

from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Twist, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FollowTrajectoryClient(object):
    """
    A class that sends joint goals to the FollowJointTrajectoryAction client.
    """
    def __init__(self):
        """ 
        Initialize action client and other class objects. 
        :param self: The self reference. 
        """
        # Setup action client that plans around objects
        rospy.loginfo("Waiting for MoveIt...")
        self.safe_client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Setup action client for torso and arm movement
        self.arm_client = actionlib.SimpleActionClient("/arm_with_torso_controller/follow_joint_trajectory" ,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for Joint trajectory...")
        self.arm_client.wait_for_server()
        rospy.loginfo("...connected")

        # Set the names of the joints
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        # Set up action client for gripper
        rospy.loginfo("Waiting for Gripper client")
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected")

        # gripper params
        self.gripper_closed_pos = 0  # The position for a fully-closed gripper (meters)
        self.gripper_open_pos = 0.10  # The position for a fully-open gripper (meters)

        # Instantiate a `FootWork()` object
        self.base_action = FootWork()

        # Insantiate a `PointHeadClient()` object
        self.head_action = PointHeadClient()

    def safe_move_to(self, positions, velocity=1):
        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        self.scene = PlanningSceneInterface("base_link")
        self.scene.removeCollisionObject("keepout")
        self.scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        # Execute motion with the moveToJointPosition function.
        result = self.safe_client.moveToJointPosition(self.joint_names,
                                                      positions,
                                                      0.0,
                                                      max_velocity_scaling_factor=velocity)

    def move_to(self,positions, duration = 1, base_motion = None, head_motion = None):
        """
        A function that sends joint goals the client. 
        :param self: The self reference.
        :param positions: A list float values.
        :param duration: A float value.
        :param base_motion: A string message type.
        :param head_motion: A string message type.
        """
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

        # Send goal to the client. 
        self.arm_client.send_goal(follow_goal,feedback_cb=self.feedback_callback)
        self.arm_client.wait_for_result()

    def feedback_callback(self,feedback):
        """
        The feedback_callback function deals with the incoming feedback messages
        from the trajectory_client. Although, in this function, we do not use the
        feedback information.
        :param self: The self reference.
        :param feedback: FollowJointTrajectoryActionFeedback message.
        """
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
        """
        A function that sends an open command to the gripper client. 
        :param self: The self reference.
        """
        # Create a GripperCommandGoal messate type
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_open_pos

        # Send goal to gripper client
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def close_gripper(self):
        """
        A function that sends a close command to the gripper client. 
        :param self: The self reference.
        """
        # Create a GripperCommandGoal message type
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_closed_pos

        # Send goal to gripper client
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()


class PointHeadClient(object):
    """
    A class that sends joint goals to the PointHeadAction client.
    """
    def __init__(self):
        """
        Initialize head action client.
        :param self: The self reference.
        """
        # Setup action client for the head movement
        self.head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.head_client.wait_for_server()

    def look_at(self, x, y, z, frame = "base_link", duration=1.0):
        """
        A function that sends joint goals the head client. 
        :param self: The self reference.
        :param x: A float value.
        :param y: A float value.
        :param z: A float value.
        :param frame: A string message type.
        :param duration: A float value. 
        """
        # Create a PointHeadGoal message type
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)

        # Send goal to head client
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()


class FootWork(object):
    """
    A class that sends twist messages to move the base.
    """
    def __init__(self):
        """
        A function that initializes the publisher and other variables. 
        :param self: The self reference.
        """
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
        self.twist.angular.z = -.90

        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()
        # Reset the angular rotation value to be zero
        self.twist.angular.z = 0.0

    def move_forward(self,iter):
        # Set forward linear motion
        self.twist.linear.x = 0.2
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()

        # Reset twist message in the x direction
        self.twist.linear.x = 0.0

    def move_backward(self,iter):
        # Set backward linear motion
        self.twist.linear.x = -0.2
        for i in range(iter):
            self.pub.publish(self.twist)
            self.rate.sleep()

        # Reset twist message in x direction
        self.twist.linear.x = 0.0


if __name__ == "__main__":
    # Create a node
    rospy.init_node("bhangra", anonymous = False)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Instantiate objects
    arm_action = FollowTrajectoryClient()
    head_action = PointHeadClient()
    base_action = FootWork()

    # Initial configuration
    rospy.sleep(.5)
    head_action.look_at(1.0, 0.0, 1.2, duration = 2)
    arm_action.safe_move_to([0.29, -1.19, 0.52, 0.0, -1.29, 0.0, -0.47, 0.0], velocity = 0.5)
    rospy.sleep(1)

    ## Begin dance

    # 4 basics moving 
    for i in range(4):
        arm_action.move_to([0.33, -1.19, 0.45, 0.0, -1.09, 0.0, -0.47, 0.0], duration = 1.4, base_motion = "Left_turn")
        arm_action.move_to([0.29, -1.19, 0.52, 0.0, -1.29, 0.0, -0.47, 0.0], duration = 1.4, base_motion = "Left_turn")

    # 4 basics moving 
    for i in range(4):
        arm_action.move_to([0.33, -1.19, 0.45, 0.0, -1.09, 0.0, -0.47, 0.0], duration = 1.4, base_motion = "Right_turn")
        arm_action.move_to([0.29, -1.19, 0.52, 0.0, -1.29, 0.0, -0.47, 0.0], duration = 1.4, base_motion = "Right_turn")

    # arm_action.close_gripper()

    arm_action.move_to([0.33, -1.27, 1.19, 0.39, -1.37, 0.0, 0.07, 0.0], duration =1.4, head_motion="move")
    arm_action.move_to([0.33, -1.27, 0.89, 0.39, -1.85, 0.0, 0.62, 0.0], duration =1.4, head_motion="move")
    

    # rospy.sleep(5)
    # arm_action.open_gripper()
