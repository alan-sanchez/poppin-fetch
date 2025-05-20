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
    ROS_INFO("Moveit Connected");

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

    // // 
    _collision_objects.push_back(_collision_object);

    // //     Add the collision to the planning scene    
    _planning_scene.addCollisionObjects(_collision_objects);

    // // Define joint names
    _joint_names = {
        "torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
        "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
    };
}

/**
 * @brief Sends a joint goal to move the Fetch's arm and torso to the initial position.
 * @param joint_values Vector containing target joint positions.
 * @param vel Maximum velocity scaling factor (range: 0.0 to 1.0).
 */
void MoveGroupClient::init_pose(const std::vector<double>& joint_values, double vel)
{
    // // Map joint names to their corresponding target values.
    std::map<std::string, double> joint_targets;
    for (size_t i = 0; i < _joint_names.size(); ++i) {
        joint_targets[_joint_names[i]] = joint_values[i];
    }

    // // Set the target joint values for the motion planner.
    _move_group->setJointValueTarget(joint_targets);

    // // Set the maximum velocity scaling factor to control the speed of the motion.
    _move_group->setMaxVelocityScalingFactor(vel);

    // // Log a message indicating that the joint targets have been set.
    ROS_INFO("Joint targets set. Proceeding to plan the motion.");

    // // Start an asynchronous spinner to process ROS callbacks in the background.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // // Create a plan object to store the planned trajectory.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // // Plan the motion to the specified joint targets.
    bool success = (_move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // // Log the result of the planning attempt.
    if (success) {
        ROS_INFO("Motion plan found. Executing the plan asynchronously.");
        // Execute the planned trajectory without blocking.
        _move_group->asyncExecute(my_plan);
    } else {
        ROS_WARN("Motion planning failed. Unable to find a valid plan.");
    }
}