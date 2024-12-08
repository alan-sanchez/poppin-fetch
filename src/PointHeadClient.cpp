#include "PointHeadClient.h"

/**
 * @brief Constructor for the PointHeadClient class.
 * 
 * Initializes the action client for the `head_controller/point_head` action server
 * and waits for the server to become available. This ensures the client is ready
 * to send goals to control the robot's head movement.
 */
PointHeadClient::PointHeadClient() : client("head_controller/point_head", true) {
    // // Log a message indicating that we are waiting for the action server.
    ROS_INFO("Waiting for head_controller...");
    
    // // Wait until the action server for the head controller is available.
    client.waitForServer();

    // // Log a message notifying user that the action server has connected.
    ROS_INFO("head_controller action server has connected");
}

/**
 * @brief Sends a goal to the head controller to point the robot's head at a specified target.
 * 
 * This method constructs a `PointHeadGoal` message and sends it to the action server.
 * The goal specifies the 3D target coordinates, the reference frame for those coordinates,
 * and the minimum duration over which the motion should occur.
 * 
 * @param x Target's x-coordinate in the specified frame.
 * @param y Target's y-coordinate in the specified frame.
 * @param z Target's z-coordinate in the specified frame.
 * @param frame The reference frame for the target coordinates (e.g., "base_link").
 * @param duration The minimum duration (in seconds) to execute the motion.
 */
void PointHeadClient::lookAt(double x, double y, double z, const std::string& frame, double duration) {
    // // Create a PointHeadGoal message to specify the target.
    control_msgs::PointHeadGoal goal;

    // // Set the timestamp for the target's header to the current time.
    goal.target.header.stamp = ros::Time::now();

    // // Set the reference frame for the target's coordinates (e.g., "base_link").
    goal.target.header.frame_id = frame;

    // // Set the target's 3D coordinates.
    goal.target.point.x = x;
    goal.target.point.y = y;
    goal.target.point.z = z;

    // // Set the minimum duration for the head motion.
    goal.min_duration = ros::Duration(duration);

    // // Send the goal to the action server.
    client.sendGoal(goal);

    // // Wait for the result of the action (i.e., the head motion to complete).
    client.waitForResult();
}
