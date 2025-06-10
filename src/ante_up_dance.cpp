#include <ros/ros.h>
#include "PointHeadClient.h"
#include "Footwork.h"
#include "FollowTrajectoryClient.h"
#include "GripperClient.h"
#include "MoveItClient.h"

/**
 * @brief Entry point of the `ante_up_dance` program.
 * 
 * @param argc Number of command-line arguments passed to the program.
 * @param argv Array of command-line arguments.
 *
 * @return int Returns 0 upon successful execution.
 */
int main(int argc, char** argv) {
    // // Initialize the ROS node
    ros::init(argc, argv, "ante_up_dance");

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

    // // Initialize the MoveGroupClient to interface with MoveIt for motion planning and execution.
    MoveGroupClient move_group_client;

    // // Begin initial dance pose
    head_client.lookAt({1.0, -0.5, 1.2},1.2);
    // trajectory_client.move_joints_to({.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 2);
    move_group_client.init_pose({.35, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0}, 0.5);
    ros::Duration(1.0).sleep();


    return 0;
}
