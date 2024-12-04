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
    ros::init(argc, argv, "bhangra_dance");

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

    // // Begin initial dance pose
    head_client.lookAt(1.0, 0.0, 1.2, "base_link", 1); 
    trajectory_client.move_joints_to({0.3, -1.03, 1.07, -2.79, -1.65, 0.04, -1.26, -0.62}, 4); // duration = 4 sec
    ros::Duration(3.0).sleep();

    // // Get Fetch to look at camera and move arm to first dance pose
    head_client.lookAt(0.0, -1.0, 1.2, "base_link",.5);
    trajectory_client.move_joints_to({0.3, -0.78, 0.37, -1.38, -1.65, 1.19, -1.27, -0.62}, 2); //, "Right Turn");
    head_client.lookAt(1.0, 0.0, 1.2, "base_link",.3);
    ros::Duration(.1).sleep();

    // // 
    for (int i=0; i<3; i++){
        base_cmd.turn(true,30,false);
        trajectory_client.move_joints_to({0.3, -1.46, 0.37, -0.65, -1.65, 1.19, -0.76, -0.19},2.4, "Right Turn");
        base_cmd.turn(false,30,false);
        trajectory_client.move_joints_to({0.3, -0.68, 0.37, -1.38, -1.65, 1.19, -1.27, -0.62},1.8, "Left Turn");
    }

    
    // Return 0 to indicate successful execution of the program.
    //[0.21, -1.37, 1.17, -0.23, -1.94, 2.96, -1.57, 0.14]
    // [0.3, -1.35, -0.25, -0.23, -1.74, 2.96, -1.5, 0.14]
    return 0;
}
