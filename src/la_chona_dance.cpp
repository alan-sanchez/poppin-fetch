#include <ros/ros.h>
#include "PointHeadClient.h"
#include "Footwork.h"
#include "FollowTrajectoryClient.h"

/**
 * @brief Entry point of the `fetch_move_node` program.
 * 
 * This program demonstrates the use of the `PointHeadClient` class to control Fetch's
 * head. It initializes the ROS node, instantiates the clients, 
 * and sends example commands to the robot.
 * 
 * @param argc Number of command-line arguments passed to the program.
 * @param argv Array of command-line arguments.
 *
 * @return int Returns 0 upon successful execution.
 */
int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "fetch_move_node");

     // // Create instances of `PointHeadClient`. This class provides functionality to interact 
     // // with the robot's head controllers through ROS action servers.
    PointHeadClient head_client;

    // // 
    Footwork base;

    FollowTrajectoryClient trajectory_client;

    // Example: Move the robot arm to a predefined position
    std::vector<double> positions = {0.4, -0.5, 0.2, 0.0, 0.5, 0.0, -0.4, 0.0};
    if (trajectory_client.move_joints_to(positions, 2.0, "Forward", "Move")) {
        ROS_INFO("Trajectory executed successfully!");
    } else {
        ROS_ERROR("Trajectory execution failed!");
    }

    base.linear_motion(true, 10); // forward = true, num_publishes=10
    base.turn(true, 10, true); // 

    // // 

    // Point the robot's head to a target location (1.0, 0.0, 1.5) in the "base_link" frame
    // over a duration of 2 seconds.
    head_client.lookAt(1.0, 0.0, 1.5, "base_link", 2.0);

    // //
    ros::spin();

    // Return 0 to indicate successful execution of the program.
    return 0;
}
