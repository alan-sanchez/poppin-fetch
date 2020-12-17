#!/usr/bin/env python3
import sys
import actionlib
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sin, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_python import MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('lawn_mower_path_cartesian_movement',
    #                 anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    # scene = moveit_commander.PlanningSceneInterface()
    scene = PlanningSceneInterface("base_link")
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    #group_name = "arm_with_torso" #or arm_with_torso
    group = moveit_commander.MoveGroupCommander("arm_with_torso")

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()

    # print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    # print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    # print "============ Robot Groups:", robot.get_group_names()
    #
    # # Sometimes for debugging it is useful to print the entire state of the
    # # robot:
    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print ""
    #
    # print "============ Printing robot arm joint state"
    # print group.get_joints()
    # print group.get_current_joint_values()
    # print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.group.set_max_velocity_scaling_factor(.7)

  def plan_cartesian_path_table(self):
    group = self.group
    robot = self.robot
    scene = self.scene
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:

    waypoints = [Pose(Point(0.50,  0.32, 1.2),Quaternion(0, 0, 0, 1)),
                 Pose(Point(0.50, -0.32, 1.2),Quaternion(0, 0, 0, 1)),
                 Pose(Point(0.70, -0.32, 1.2),Quaternion(0, 0, 0, 1)),
                 Pose(Point(0.70,  0.32, 1.2),Quaternion(0, 0, 0, 1)),
                 # Pose(Point(0.85,  0.32, 1.2),Quaternion(0, 0, 0, 1)),
                 # Pose(Point(0.85, -0.32, 1.2),Quaternion(0, 0, 0, 1)),
                 # Pose(Point(0.85, -0.3, 1),Quaternion(0.000, -0.216, 0, 0.976)),
                 # Pose(Point(0.85,  0.3, 1),Quaternion(0.000, -0.216, 0, 0.976)),
                 # Pose(Point(0.50,  0.4, 1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.17, 0.335, 1),Quaternion(0.000, 0.0, 0, 1)),]

    # scene = PlanningSceneInterface("base_link")
    scene.removeCollisionObject("table")
    scene.addBox("table", 0.73, 1.52, 0.74, .9, 0.0, 0.73/2)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(robot.get_current_state(),plan,.4)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been completed")
    return plan, fraction

  def plan_cartesian_path_couch_a(self):
    group = self.group
    robot = self.robot
    scene = self.scene

    waypoints = [Pose(Point(0.17, 0.335, 1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.65, 0.3, 1),Quaternion(0.0, 0.0, 0, 1)),
                 Pose(Point(0.65,  0.3, 1),Quaternion(0.000, 0, 0, 1)),
                 Pose(Point(0.65, -0.3, 1),Quaternion(0.000, 0, 0, 1)),
                 Pose(Point(0.65, -0.3, 1),Quaternion(-0.383, 0, 0, 0.924)),
                 Pose(Point(0.80, -0.3, 1),Quaternion(-0.383, 0, 0, 0.924)),
                 Pose(Point(0.80, -0.3, 1.1),Quaternion(0.000, 0, 0, 1)),
                 Pose(Point(0.80, -0.3, 1.1),Quaternion(0.000,-0.383, 0,  0.924)),
                 Pose(Point(0.80, 0.3,  1.1),Quaternion(0.000, -0.383, 0,  0.924)),]


    scene.addBox("couch_a", 0.89, 1.4, 0.50, 1.2, 0.2, 0.50/2)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(robot.get_current_state(),plan,.2)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been completed")
    return plan, fraction

  def plan_cartesian_path_couch_b(self):
    group = self.group
    robot = self.robot
    scene = self.scene

    waypoints = [Pose(Point(0.17, 0.335, 1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.65, 0.2, 1.1),Quaternion(0.0, 0.0, 0, 1)),
                 Pose(Point(0.65,  0.2, 1.1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.65, -0.2, 1.1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.80, -0.2, 1.1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.80,  0.2, 1.1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.80,  0.2, 1.1),Quaternion( 0,-0.383, 0, 0.924)),
                 Pose(Point(0.80, -0.2, 1.1),Quaternion( 0, -0.383,0, 0.924)),]



    scene.addBox("couch_b", 1.4, .6, 0.50, 1.4, 0, 0.50/2)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(robot.get_current_state(),plan,.2)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been completed")
    return plan, fraction

  def plan_cartesian_path_metal_cabinet(self):
    group = self.group
    robot = self.robot
    scene = self.scene

    waypoints = [Pose(Point(0.70,  0, .9),Quaternion(0.000, -0.462, 0, 0.887)),
                 Pose(Point(0.70,  -0.1, 1),Quaternion(0.192,-0.451, -0.1,0.866)),
                 Pose(Point(0.70,  0.1, 1),Quaternion(-0.192,-0.451, -0.1,0.866)),]


    # scene.addBox("metal_cabinet", 0.4, 1, 2.2 , 1.8, 0, 2.2/2)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(robot.get_current_state(),plan,.4)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been completed")
    return plan, fraction

  def plan_cartesian_path_door(self):
    group = self.group
    robot = self.robot
    scene = self.scene

    waypoints = [Pose(Point(0.17, 0.335, 1.1),Quaternion(0.000, 0.0, 0, 1)),
                 # Pose(Point(0.5, 0.35, 1.1),Quaternion(0.0, 0.0, 0, 1)),
                 # Pose(Point(0.5, 0.35, 1.1),Quaternion(0.0, -.609, 0, .793)),
                 # Pose(Point(0.5, -0.35, 1.1),Quaternion(0.0, -.609, 0, .793)),
                 Pose(Point(0.5, 0.35, .87),Quaternion(0.0, -.707, 0, .707)),
                 Pose(Point(0.5, -0.35, .87),Quaternion(0.0, -.707, 0, .707)),
                 Pose(Point(0.5, -0.35, .75),Quaternion(0.0, -.793, 0, .609)),
                 Pose(Point(0.5, 0.35, .75),Quaternion(0.0, -.793, 0, .609)),]
                 # Pose(Point(0.5, -0.35, .87),Quaternion(0.0, -.707, 0, .707)),
                 # Pose(Point(0.5, 0.35, .87),Quaternion(0.0, -.707, 0, .707)),]
                 # Pose(Point(0.5, 0.3, 1),Quaternion(0.0, -.707, 0, .707)),
                 # Pose(Point(0.5, 0.3, .8),Quaternion(0.0, 0, 0, 1)),]
                 # Pose(Point(0.17, 0.335, 1),Quaternion(0.000, 0.0, 0, 1)),]


    scene.addBox("door",  0.4, 1, 2.2 , 1.2, 0, 2.2/2)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(robot.get_current_state(),plan,.2)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been completed")
    return plan, fraction

  def plan_cartesian_path_bed(self):
    group = self.group
    robot = self.robot
    scene = self.scene

    waypoints = [Pose(Point(0.17, 0.335, 1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.5, 0.4, 1),Quaternion(0.383, 0, 0, 0.924)),
                 Pose(Point(0.5,  -0.4, 1),Quaternion(-0.383, 0, 0, 0.924)),
                 Pose(Point(0.85, -0.4, 1.1),Quaternion(-0.383, 0, 0, 0.924)),
                 Pose(Point(0.85, 0.4, 1.1),Quaternion(0.383, 0, 0, 0.924)),]



    scene.addBox("couch_b", 1.4, .6, 0.50, 1.4, 0, 0.50/2)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(robot.get_current_state(),plan,.4)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been completed")
    return plan, fraction

  def plan_cartesian_path_chair(self):
    group = self.group
    robot = self.robot
    scene = self.scene

    waypoints = [Pose(Point(0.17, 0.335, 1),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.6, 0.55, .85),Quaternion(-0.383, 0, 0, 0.924)),
                  Pose(Point(0.6,  0.1, .95),Quaternion(0, -0.383, 0, 0.924)),
                  Pose(Point(0.6, -0.15, .85),Quaternion(0.131, 0, 0, 0.991)),
                  Pose(Point(0.6, -0.15, .85),Quaternion(-.609, 0, 0, 0.793)),]
                 # Pose(Point(0.85, 0, 1.1),Quaternion( 0,-0.383, 0, 0.924)),]
                 # Pose(Point(0.5,  -0.4, 1),Quaternion(-0.383, 0, 0, 0.924)),
                 # Pose(Point(0.8, -0.4, 1),Quaternion(-0.383, 0, 0, 0.924)),
                 # Pose(Point(0.8, 0.4, 1),Quaternion(0.383, 0, 0, 0.924)),]



    scene.addBox("couch_b", 1.4, .6, 0.50, 1.4, 0, 0.50/2)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(robot.get_current_state(),plan,.2)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been completed")
    return plan, fraction

  def execute_plan(self, plan):
    group = self.group
    scene = self.scene
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    print "======== Press 'Enter' to start the lawn mower path. "
    raw_input()
    group.execute(plan, wait=True)
    scene.removeCollisionObject("table")
    scene.removeCollisionObject("couch_a")
    scene.removeCollisionObject("couch_b")
    scene.removeCollisionObject("metal_cabinet")
    scene.removeCollisionObject("door")

  def move_to_goal(self,ee_pose):
    scene=self.scene
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = ee_pose[0]
    pose_goal.position.y = ee_pose[1]
    pose_goal.position.z = ee_pose[2]
    if len(ee_pose) == 6:
        quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]), math.radians(ee_pose[5]))
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
    else:
        pose_goal.orientation.x = ee_pose[3]
        pose_goal.orientation.y = ee_pose[4]
        pose_goal.orientation.z = ee_pose[5]
        pose_goal.orientation.w = ee_pose[6]


    self.group.set_pose_target(pose_goal)
    scene.addBox("metal_cabinet", 0.4, 1, 2.2, 1.2, 0, 1.1)
    self.group.set_max_velocity_scaling_factor(.3)
    self.group.set_planning_time(5)
    self.plan = self.group.plan()
    self.group.go(wait=True)
    # self.group.execute(self.plan, wait=True)
    self.group.stop()
    self.group.clear_pose_targets()
    scene.removeCollisionObject("metal_cabinet")

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
                return

    def init_pose(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")
        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

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
                return


# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


def main():
  rospy.init_node('lawn_mower_path_cartesian_movement',
                    anonymous=True)
  global client
  try:

    path = MoveGroupPythonInteface()
    arm_with_torso_action = FollowTrajectoryClient()
    move_base = MoveBaseClient()

    # raw_input("======== Press 'Enter' to Move to initial pose")
    # arm_with_torso_action.init_pose()
    # raw_input("======== Press 'Enter' to Move to the Table. ")
    # rospy.loginfo("Moving to table...")
    # move_base.goto(-5.0,2.6, 0.07)
    # cartesian_plan_table, fraction = path.plan_cartesian_path_table()
    # path.execute_plan(cartesian_plan_table)
    # raw_input('Press Enter to move to initial arm position')
    # arm_with_torso_action.init_pose()

    # raw_input("======== Press 'Enter' to Move to couch. ")
    # rospy.loginfo("Moving to big book shelf...")
    # move_base.goto(-5.6, 6.2, 0.07)
    # cartesian_plan_couch_a, fraction =path.plan_cartesian_path_couch_a()
    # path.execute_plan(cartesian_plan_couch_a)
    # raw_input("Move to initial position")
    # arm_with_torso_action.init_pose()
    #
    # raw_input("======== Press 'Enter' to Move to other side of the couch. ")
    # rospy.loginfo("Moving to oher side of the couch...")
    # move_base.goto(-6.4, 7.3, 0.09)
    # cartesian_plan_couch_b, fraction =path.plan_cartesian_path_couch_b()
    # path.execute_plan(cartesian_plan_couch_b)
    # raw_input("Move to initial position")
    # arm_with_torso_action.init_pose()
    #
    # raw_input("======== Press 'Enter' to the Metal cabinet. ")
    # rospy.loginfo("Moving to small book shelf...")
    # move_base.goto(-13.25,5.9, -3.051)
    # raw_input("======== Press 'Enter' to execute motion. ")
    # rospy.loginfo("Moving to waypoints...")
    # path.move_to_goal([0.70,   0, .9,   0.000, -0.707,  0.000, 0.707])
    # path.move_to_goal([0.68, .2, .95,  -0.084, -0.637, -0.1, 0.759])
    # path.move_to_goal([0.68, -.2, .95,   0.084, -0.637,  0.1, 0.759])
    # raw_input("Move to initial position")
    # arm_with_torso_action.init_pose()
    # #
    raw_input("======== Press 'Enter' to the door. ")
    rospy.loginfo("Moving to door...")
    move_base.goto(-6.63, 5.7, -1.47)
    move_base.goto(-12.6,-4.5,3.14)
    cartesian_plan_door, fraction = path.plan_cartesian_path_door()
    path.execute_plan(cartesian_plan_door)
    raw_input("Move to initial position")
    arm_with_torso_action.init_pose()

    # raw_input( "======== Press 'Enter' to Move to bed. ")
    # rospy.loginfo("Moving to coffee table...")
    # # move_base.goto(-6.0, -3.9, .319)
    # move_base.goto(-.25, 1.9, 1.7)
    # cartesian_plan_bed, fraction = path.plan_cartesian_path_bed()
    # path.execute_plan(cartesian_plan_bed)
    # raw_input("Move to initial position")
    # arm_with_torso_action.init_pose()
    # #
    #
    # raw_input( "======== Press 'Enter' to Move to other the side of the bed. ")
    # rospy.loginfo("Moving to coffee table...")
    # move_base.goto(-2.25, 1.45, 1.65)
    # move_base.goto(-2, 3.60, 0.09)
    # cartesian_plan_chair, fraction = path.plan_cartesian_path_chair()
    # path.execute_plan(cartesian_plan_chair)
    # raw_input("Move to initial position")
    # arm_with_torso_action.init_pose()


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
