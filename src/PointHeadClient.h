#ifndef POINT_HEAD_CLIENT_H
#define POINT_HEAD_CLIENT_H

// // Include ROS core library for general ROS functionality
#include <ros/ros.h>

// // Include the ActionLib library for creating action clients
#include <actionlib/client/simple_action_client.h>

// // Include the PointHeadAction message type for sending goals to the head controller
#include <control_msgs/PointHeadAction.h>

/**
 * @class PointHeadClient
 * @brief A class to interface with the head controller of a robot using ROS actions.
 * 
 * This class provides functionality to send commands to a robot's head controller,
 * allowing it to point the head towards a specified target in 3D space. It utilizes
 * the ROS `PointHeadAction` to send goals and wait for their execution.
 */
class PointHeadClient {
public:
    /**
     * @brief Constructor for PointHeadClient
     * 
     * Initializes the action client and waits for the `head_controller/point_head` action server
     * to be ready.
     */
    PointHeadClient();

    /**
     * @brief Points the robot's head at a specified target in 3D space.
     * 
     * @param x Target's x-coordinate (in the specified frame).
     * @param y Target's y-coordinate (in the specified frame).
     * @param z Target's z-coordinate (in the specified frame).
     * @param frame The reference frame for the target coordinates (e.g., "base_link").
     * @param duration The minimum duration (in seconds) to execute the motion.
     * 
     * This function sends a goal to the head controller action server to point the
     * robot's head at the specified target. The motion will be executed over at least
     * the specified duration.
     */
    void lookAt(double x, double y, double z, const std::string& frame, double duration = 1.0);

private:
    /**
     * @brief Action client for sending goals to the head controller.
     * 
     * The `SimpleActionClient` connects to the `head_controller/point_head` action server
     * and manages sending goals, receiving feedback, and waiting for results.
     */
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> client;
};

#endif // POINT_HEAD_CLIENT_H
