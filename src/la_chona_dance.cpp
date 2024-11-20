#include <ros/ros.h>
#include "PointHeadClient.h"


/**
 * @brief Entry point of the `fetch_move_node` program.
 * 
 * This program demonstrates the use of the `PointHeadClient` class to control Fetch's
 * head and gripper respectively. It initializes the ROS node, instantiates the clients, 
 * and sends example commands to the robot.
 * 
 * @param argc Number of command-line arguments passed to the program.
 * @param argv Array of command-line arguments.
 * @return int Returns 0 upon successful execution.
 */
int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "fetch_move_node");

     // // Create instances of `PointHeadClient`. This class provides functionality to interact 
     // with the robot's head controllers through ROS action servers.
    PointHeadClient head_client;       // Class to control the robot's head

    // Point the robot's head to a target location (1.0, 0.0, 1.5) in the "base_link" frame
    // over a duration of 2 seconds.
    head_client.lookAt(1.0, 0.0, 1.5, "base_link", 2.0);

    // Return 0 to indicate successful execution of the program.
    return 0;
}
