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
     * @param vel Maximum velocity scaling factor.
     * @return 0 on success, 1 on failure.
     */
    int init_pose(double vel = 0.2);

private:
    moveit::planning_interface::MoveGroupInterface* _move_group;
    moveit::planning_interface::PlanningSceneInterface _planning_scene;
    std::string gripper_frame_;
};

#endif // MOVE_GROUP_CLIENT_H
