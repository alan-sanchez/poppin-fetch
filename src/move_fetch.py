#!/usr/bin/env python
import sys
import actionlib
import copy
import rospy
from math import pi, sin, cos
from std_msgs.msg import String, Int16
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from moveit_python import MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveGroupInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupInteface, self).__init__()
    self.gui_input_sub = rospy.Subscriber('gui_input', String, self.callback)
    self.waypoints_sub = rospy.Subscriber('waypoints', PoseArray, self.plan_path)
    ## First initialize `moveit_commander`
    # Create move group interface for a fetch robot
    self.move_group = MoveGroupInterface("arm_with_torso", "base_link")

    self.path_to_goal=FollowTrajectoryClient()

    self.poses = None
    self.ee_frame = 'gripper_link'
    self.ee_pose_stamped = PoseStamped()
    self.ee_pose_stamped.header.frame_id = 'base_link'

  def callback(self,gui_input):

      if gui_input.data == "1":
          self.execute_plan()

      elif gui_input.data == "2":
          self.path_to_goal.init_pose()

      elif gui_input.data == "3":
          self.path_to_goal.tuck_pose()

  def plan_path(self,msg):

    ## Plan Paths
    waypoints = []
    for i in range(len(msg.poses)):
        waypoints.append(msg.poses[i])

    self.poses = waypoints
    print("Waypoints have been updated and stored")

  def execute_plan(self):
      scene = PlanningSceneInterface("base_link")
      scene.removeCollisionObject("table")
      scene.addBox("table", 0.73, 1.52, 0.76, .6, 0.0, 0.76/2)

      for pose in self.poses:
          # Finish building the Pose_stamped message
          # If the message stamp is not current it could be ignored
          self.ee_pose_stamped.header.stamp = rospy.Time.now()
          #set the message pose
          self.ee_pose_stamped.pose = pose

          # Move gripper frame to the pose specified
          self.move_group.moveToPose(self.ee_pose_stamped,
                                self.ee_frame,
                                max_velocity_scaling_factor=0.2)

          result = self.move_group.get_move_action().get_result()

          if result:
              # Checking the MoveItErrorCode
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                  rospy.loginfo("Hello there!")
              else:
                  # If you get to this point please search for:
                  # moveit_msgs/MoveItErrorCodes.msg
                  rospy.logerr("Arm goal in state: %s",
                               self.move_group.get_move_action().get_state())
          else:
              rospy.logerr("MoveIt! failure no result returned.")

      # This stops all arm movement goals
      # It should be called when a program is exiting so movement stops
      self.move_group.get_move_action().cancel_all_goals()

class FollowTrajectoryClient(object):

    def __init__(self):
        self.client = None

    def tuck_pose(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.removeCollisionObject("keepout")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)
        scene.removeCollisionObject("table")
        scene.addBox("table", 0.73, 1.52, 0.80, .6, 0.0, 0.80/2)


        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.1, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=0.5)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                scene.removeCollisionObject("table")
                return

    def init_pose(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")
        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)
        scene.removeCollisionObject("table")
        scene.addBox("table", 0.73, 1.52, 0.80, .6, 0.0, 0.80/2)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [.05, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]
        # pose = [.15, 1.61, 1.52, 0.05, -2.25, -3.14, 1.2, -0.37,]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=0.2)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                scene.removeCollisionObject("table")
                return



if __name__=="__main__":
    rospy.init_node('move_fetch',anonymous=True)
    MoveGroupInteface()
    path_to_goal=FollowTrajectoryClient()
    path_to_goal.init_pose()
    rospy.spin()
