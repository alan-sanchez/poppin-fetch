// //
#include "MoveItClient.h"


MoveGroupClient::MoveGroupClient()
    : _move_group(nullptr)
{
    // // Notify user for MoveIt connection
    ROS_INFO("Waiting for MoveIt!");

    // // Initialize move group interface
    _move_group = new moveit::planning_interface::MoveGroupInterface("arm_with_torso");
    _gripper_frame = "gripper_link";

    // // TODO: Add collision objects to the planning scene

    // // Define joint names
    std::vector<std::string> _joints = {
        "torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
        "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
    };
}


int MoveGroupClient::init_pose(std::vector<double> joint_values, double vel)
{
    
    // // List of joint values for initial position
    // std::vector<double> _joint_values = {0.27, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37};

    // while (ros::ok()) {
    //     _move_group->setJointValueTarget(joints, _joint_values);
    //     _move_group->setMaxVelocityScalingFactor(vel);

    //     moveit::planning_interface::MoveGroupInterface::Plan plan;
    //     bool success = (_move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     if (success) {
    //         moveit::planning_interface::MoveItErrorCode result = _move_group->execute(plan);
    //         if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    //             _planning_scene.removeCollisionObject("keepout");
    //             return 0;
    //         }
    //     }
    // }
    return 1;
}