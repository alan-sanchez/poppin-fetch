#include "GripperClient.h"

/**
 * @brief 
 *
 */
GripperClient::GripperClient() : _client("gripper_controller/gripper_action") {
    // // 
    ROS_INFO("Waiting for gripper_controller...");
    
    // // Wait until the action server for the head controller is available.
    _client.waitForServer();

    // // Log a message notifying user that the action server has connected.
    ROS_INFO("gripper_controller action server has connected");    
}


/**
 * @brief 
 *
 */
 void GripperClient::gripper_action(bool close_gripper){
    // //
    control_msgs::GripperCommandGoal goal;

    // //
    if (close_gripper==false){
        goal.command.position = 0.1;
    } else{
        goal.command.position = 0.0;
    }

    _client.sendGoal(goal);

    _client.waitForResult();
 }