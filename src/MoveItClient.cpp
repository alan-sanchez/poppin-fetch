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

    // // Add collision objects to the planning scene
    _collision_object.header.frame_id = "base_link";
    // _collision_object.id = "base_frame";

    // // Define the shape and dimensions of the primitive collision object
    // // In this case, we're creating a box with specified X, Y, Z dimensions (in meters)
    _primitive.type = _primitive.BOX;
    _primitive.dimensions = {0.2, 0.5, 0.05}; // X, Y, and Z dimension
    
    // // Specify the pose (position and orientation) of the box in the reference frame
    // // Here, we set it directly in front of the robot, on the ground or table
    _box_pose.orientation.w = 1.0;
    _box_pose.position.x = 0.15;
    _box_pose.position.y = 0.00;
    _box_pose.position.z = 0.375;

    // // Add the primitive shape and its pose to the collision object definition
    _collision_object.primitives.push_back(_primitive);
    _collision_object.primitive_poses.push_back(_box_pose);
    _collision_object.operation = _collision_object.ADD;
    
    // // Define joint names
    std::vector<std::string> _joint_names = {
        "torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
        "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
    };
}


void MoveGroupClient::init_pose(const std::vector<double>& joint_values, double vel)
{
    std::map<std::string, double> joint_targets;
    for (size_t i = 0; i < _joint_names.size(); ++i) {
        joint_targets[_joint_names[i]] = joint_values[i];
    }
    _move_group->setJointValueTarget(joint_targets);
    _move_group->setMaxVelocityScalingFactor(vel);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (_move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   
    //     if (success) {
    //         moveit::planning_interface::MoveItErrorCode result = _move_group->execute(plan);
    //         if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    //             _planning_scene.removeCollisionObject("keepout");
    //             return 0;
    //         }
    //     }
    // }
    // return 1;
}