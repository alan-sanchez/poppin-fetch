#ifndef MOVE_GROUP_CLIENT_H
#define MOVE_GROUP_CLIENT_H

// // Provides the std::string class for string manipulation
#include <string>

// // Provides the std::vector container for dynamic arrays
#include <vector>

// // Inlcude ROS core library for general ROS functionality
#include <ros/ros.h>

// // Interface for motion planning and execution using MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

// // Interface for managing the planning scene, including adding or removing collision objects
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// // Pull the message definition for representing collision objects in the planning scene
#include <moveit_msgs/CollisionObject.h>

// // Include message type that represents the geometric shape of an object
#include <shape_msgs/SolidPrimitive.h>

// // Include Pose message type for box object
#include <geometry_msgs/Pose.h>

/**
 * @brief Class for controlling mid-level operations of the arm and torso.
 */
class MoveGroupClient {
public:
    /**
     * @brief Constructor that initializes the MoveGroupClient.
     */
    MoveGroupClient();

   /**
    * @brief Sends a joint goal to move the Fetch's arm and torso to the initial position.
    * @param joint_values Vector containing target joint positions.
    * @param vel Maximum velocity scaling factor (range: 0.0 to 1.0).
    */
    void init_pose(const std::vector<double>& joint_values, double vel = 0.2);

private:
    moveit::planning_interface::MoveGroupInterface* _move_group;
    moveit::planning_interface::PlanningSceneInterface _planning_scene;
    std::string _gripper_frame;
    std::vector<std::string> _joint_names;
    std::vector<moveit_msgs::CollisionObject> _collision_objects;
    moveit_msgs::CollisionObject _collision_object;
    shape_msgs::SolidPrimitive _primitive;  
    geometry_msgs::Pose _box_pose;
};

#endif // MOVE_GROUP_CLIENT_H
