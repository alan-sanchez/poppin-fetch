#include <ros/ros.h>
#include "PointHeadClient.h"
#include "Footwork.h"
#include "FollowTrajectoryClient.h"
#include "GripperClient.h"

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

    // // Create an instance of `GripperClient`. This class allows control over the robot's gripper,
    // // enabling commands to open and close the gripper for object manipulation.
    GripperClient gripper_client;

    // // Create an instance of `Footwork`. This class provides functionality for base movements, 
    // // such as linear motion and turning. It sends velocity commands to the robot's base.
    Footwork base_cmd;

    // // Create an instance of `FollowTrajectoryClient`. This class interacts with the 
    // // robot's arm, allowing it to move to specified joint positions through a trajectory action.
    FollowTrajectoryClient trajectory_client;

    // Example: Move the robot arm to a predefined position
    // std::vector<double> positions = {0.4, -0.5, 0.2, 0.0, 0.5, 0.0, -0.4, 0.0};
    if (trajectory_client.move_joints_to({0.4, -0.5, 0.2, 0.0, 0.5, 0.0, -0.4, 0.0}, 2.0, "Forward", "Move")) {
        ROS_INFO("Trajectory executed successfully!");
    } else {
        ROS_ERROR("Trajectory execution failed!");
    }

    base_cmd.linear_motion(true, 10); // forward = true, num_publishes=10
    base_cmd.turn(true, 10, true); // 

    // // Command the robot's gripper to open and close. 
    gripper_client.gripper_action(true);
    gripper_client.gripper_action(false);

    // Point the robot's head to a target location (1.0, 0.0, 1.5) in the "base_link" frame
    // over a duration of 2 seconds.
    head_client.lookAt(1.0, 0.0, 0.5, "base_link", 2.0);
    head_client.lookAt(1.0, 0.0, 0.5, "gripper_link", 2.0);

    // // Keep the node running
    ros::spin();

    // Return 0 to indicate successful execution of the program.
    return 0;
}
