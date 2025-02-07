#include <ros/ros.h>
#include "PointHeadClient.h"
#include "Footwork.h"
#include "FollowTrajectoryClient.h"
#include "GripperClient.h"

/**
 * @brief Entry point of the `la_chona_dance` program.
 *
 * @param argc Number of command-line arguments passed to the program.
 * @param argv Array of command-line arguments.
 *
 * @return int Returns 0 upon successful execution.
 */
int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "la_chona_dance");

    // // // Create instances of `PointHeadClient`. This class provides functionality to interact
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
    head_client.lookAt({1.0, 0.0, 1.2});
    gripper_client.close_gripper(false);
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

    // // Small cumbia back steps
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn"); // duration=0.3, base_motion="Right Turn"
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");
        ros::Duration(0.2).sleep(); // make sure there is a pause before rotating in the other direction
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Left Turn"); // duration=0.3, base_motion="Right Turn"
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Left Turn");
        ros::Duration(0.2).sleep();
    }

    // // Turn to the right and face the camera in a 45 degree angle, them move forward
    ros::Duration(0.5).sleep();
    trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");
    trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");

    for (int i=0; i<3; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward");
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward");
    }

    // // Turn to the left and face the camera in a 45 degree angle, then move forward
    ros::Duration(0.2).sleep();
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.29, "Left Turn");
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.29, "Left Turn");
    }
   
    for (int i=0; i<3; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward");
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Forward");
    }

    // // Right turn to face the camera, then move backward.
    ros::Duration(0.2).sleep();
    trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");
    trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3, "Right Turn");
    // trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.25, "Right Turn");


    for (int i=0; i<4; i++){
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.35, "Backward");
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.35, "Backward");
    }

    // // Pause before the drop
    ros::Duration(1).sleep();

    // // Move torso with tempo
    for (int i=0; i<2; i++){
        trajectory_client.move_joints_to({0.30, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3);
        trajectory_client.move_joints_to({0.28, -1.3, 1.2, 0.29, 1.86, -0.02, 1.29, 0.0}, 0.3);
    }
    ros::Duration(1).sleep();

    // // Complete a wide right turn while moving the arm configuration
    trajectory_client.move_joints_to({0.30, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0}, 4, "Wide Right Turn"); //
    ros::Duration(1).sleep();

    // // Left turn to face the camera then move backward
    for (int i=0; i<4; i++){
        trajectory_client.move_joints_to({0.28, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0}, 0.29, "Left Turn");
        trajectory_client.move_joints_to({0.30, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0}, 0.29, "Left Turn");
    }

    for (int i=0; i<4; i++){
        trajectory_client.move_joints_to({0.28, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0}, 0.4, "Backward");
        trajectory_client.move_joints_to({0.30, -1.33, 0.21, 2.75, 1.84, 0.0, 1.03, 0.0}, 0.4, "Backward");
    }

    // // Move the arm to grasp the sombrero then pull it off Fetch's head.
    trajectory_client.move_joints_to({0.3, -1.33, 0.04, 2.64, 1.94, 0.0, 1.03, 0.69}, 1);
    gripper_client.close_gripper(true);
    trajectory_client.move_joints_to({0.3, -1.31, -0.29, 2.64, 1.33, 0.0, 0.59, 0.69}, 1);

    // // // Open gripper to restart dance routine
    // ros::Duration(5).sleep();
    // gripper_client.close_gripper(false);

    // Return 0 to indicate successful execution of the program.
    return 0;
}
