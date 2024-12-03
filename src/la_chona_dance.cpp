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
    ros::init(argc, argv, "la_chona_dance");

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

    // // Example: Move the robot arm to a predefined position
    // if (trajectory_client.move_joints_to({0.4, -0.5, 0.2, 0.0, 0.5, 0.0, -0.4, 0.0}, 2.0, "Forward", "Move")) {
    //     ROS_INFO("Trajectory executed successfully!");
    // } else {
    //     ROS_ERROR("Trajectory execution failed!");
    // }

    // base_cmd.linear_motion(true, 10); // forward = true, num_publishes=10
    // base_cmd.turn(true, 10, true); // 

    // // // Command the robot's gripper to open and close. 
    // gripper_client.gripper_action(true);
    // gripper_client.gripper_action(false);

    // // Point the robot's head to a target location (1.0, 0.0, 1.5) in the "base_link" frame
    // // over a duration of 2 seconds.
    // head_client.lookAt(1.0, 0.0, 0.5, "base_link", 2.0);
    // head_client.lookAt(1.0, 0.0, 0.5, "gripper_link", 2.0);

    // // Begin initial dance pose
    head_client.lookAt(1.0, 0.0, 1.2, "base_link", 1); 
    trajectory_client.move_joints_to({0.3, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 4); // duration = 4 sec
    ros::Duration(3.0).sleep();


    // // 8 basics moving forward and backward, while also moving the torso up and down
    for (int i=0; i<4; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward"); // duration=0.3, base_motion="Forward"
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward"); 
    }

    for (int i=0; i<4; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Backward"); // duration=0.3, base_motion="Backward"
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Backward"); 
    }
    
    // // 
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn"); // duration=0.3, base_motion="Right Turn"
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Left Turn"); // duration=0.3, base_motion="Right Turn"
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Left Turn");
    }

    // // 
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn"); 
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");
    }

    // // 
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward"); 
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward");
    }

    // // 
    for (int i=0; i<3; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Left Turn"); 
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Left Turn");
    }

    // // 
    for (int i=0; i<3; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward"); 
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward");
    }

    // // 
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn"); 
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");
    }

    // // 
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Backward"); 
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Backward");
    }

    // // 
    ros::Duration(0.4).sleep();


    // // 
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3); 
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3);
    }

    // // 
    trajectory_client.move_joints_to({0.3, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0}, 4, "Wide Left Turn"); //


    // Return 0 to indicate successful execution of the program.
    return 0;
}
