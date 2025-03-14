// // Imports the ROS client library for C++. This provides the necessary 
// // functions and classes for creating nodes, subscribing to topics, logging, 
// // and managing node execution
#include <ros/ros.h>

// // Import message types
// // Provides the SimpleActionClient class for sending goals and receiving 
// // results/feedback from ROS action servers.
#include <actionlib/client/simple_action_client.h> 

// // Includes message definitions for the QueryControllerStates action, such as goal, 
// // result, and feedback structures.
#include <robot_controllers_msgs/QueryControllerStatesAction.h> 

// // STL container for managing dynamic arrays.
#include <vector> 

// // Sstandard library for string manipulation and storage
#include <string> // Standard library for string manipulation and storage.

int main(int argc, char** argv)
{
  // // Initialize the ROS node
  ros::init(argc, argv, "relax_arm_control");

  // // Creating a Nodehandle object to interact with the ROS system
  // // This provides acces to ROS functionalites (subscribe, publish, interact w/ services and params)
  ros::NodeHandle nh;

  // // Use ROS log info to notify user they can move the arm
  ROS_INFO("Relaxed arm node activated. You can now move the manipulator.");
  ROS_INFO("Type Ctrl + C when you are done.");

  // // Give control to ROS to process incoming messages
  ros::spin();

  return 0;
}