#include <ros/ros.h>
#include "PointHeadClient.h"
#include "Footwork.h"
#include "FollowTrajectoryClient.h"

/**
 * @brief Entry point of the `is_it_true_dance` program.
 * 
 * @param argc Number of command-line arguments passed to the program.
 * @param argv Array of command-line arguments.
 *
 * @return int Returns 0 upon successful execution.
 */
int main(int argc, char** argv) {
    // // Initialize the ROS node
    ros::init(argc, argv, "is_it_true_dance");

    // // Create instances of `PointHeadClient`. This class provides functionality to interact 
    // // with the robot's head controllers through ROS action servers.
    PointHeadClient head_client;

    // // Create an instance of `Footwork`. This class provides functionality for base movements, 
    // // such as linear motion and turning. It sends velocity commands to the robot's base.
    Footwork base_cmd;

    // // Create an instance of `FollowTrajectoryClient`. This class interacts with the 
    // // robot's arm, allowing it to move to specified joint positions through a trajectory action.
    FollowTrajectoryClient trajectory_client;

    // // Begin initial dance pose
    head_client.lookAt({1.0, 0.0, 1.2},1.0);
    trajectory_client.move_joints_to({.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 2);
    ros::Duration(1.0).sleep();

    // // Start dancing
    head_client.lookAt({1.0, -0.85, 1.2}, 0.5);
    trajectory_client.move_joints_to({.32, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 1); //,"None", {1.0, -0.85, 1.2});

    // // Start mobile base and head motions
    head_client.lookAt({1.0, 0.0, 1.2, 0.5});
    trajectory_client.move_joints_to({.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 1.5,"Forward");

    // // Begin turn to face opposite direction 
    base_cmd.turn(true, 48); // Might need to change the velocity on the footwork class
    ros::Duration(0.2).sleep();
    
    // // Forward base motion with torso height change
    trajectory_client.move_joints_to({.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 1.0, "Forward");
    ros::Duration(0.4).sleep();
    base_cmd.linear_motion(true,5);
    ros::Duration(0.4).sleep();
    trajectory_client.move_joints_to({.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5, "Forward");
    base_cmd.linear_motion(true,5);
    ros::Duration(0.4).sleep();
    trajectory_client.move_joints_to({.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5, "Forward");
    trajectory_client.move_joints_to({.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5, "Forward");
    trajectory_client.move_joints_to({.34, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5, "Forward");
    trajectory_client.move_joints_to({.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5, "Forward");
    ros::Duration(0.4).sleep();

    // // Backward motion/moonwalk
    base_cmd.linear_motion(false,5);
    trajectory_client.move_joints_to({.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5);
    base_cmd.linear_motion(false,5);
    trajectory_client.move_joints_to({.38, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5);
    base_cmd.linear_motion(false,10);

    return 0;
}
