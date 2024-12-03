#ifndef GRIPPER_CLIENT_H
#define GRIPPER_CLIENT_H

// // Include ROS core library for general ROS functionality
#include <ros/ros.h>

// // Include the ActionLib library for creating action clients
#include <actionlib/client/simple_action_client.h>

// // 
#include <control_msgs/GripperCommandAction.h>

/**
 * @class GripperClient
 * @brief A class to interface with the robot's gripper.
 * 
 * This class provides functionality to send commands to a robot's gripper controller
 * It utilizes `GripperAction` to send goals and wait for their execution.
 */
class GripperClient {
public:
    // // Constructor for the GripperClient
    GripperClient();

    /**
     * @brief  
     *
     */
    void close_gripper(bool close_gripper=false);

private:
    /**
     * @brief Action client for sending goals to the head controller.
     * 
     * The `SimpleActionClient` connects to the `head_controller/point_head` action server
     * and manages sending goals, receiving feedback, and waiting for results.
     */
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> _client;
};

#endif // GRIPPER_CLIENT_H
