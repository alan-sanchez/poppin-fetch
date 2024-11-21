#ifndef FOOT_WORK_H
#define FOOT_WORK_H

// // Include ROS core library for general ROS functionality
#include <ros/ros.h>

// // Imports the Twist message type from the geometry_msgs package. That way 
// // the velocity commands can be sent to the robot
#include <geometry_msgs/Twist.h>

/**
 * @class Footwork
 * @brief A class to move the base of the robot using Twist commands
 * 
 * This class provides functionality to send commands to the robot's base,
 * allowing it to move forward, backward, and turn in both directions. It utilizes
 * the Twist commands to control the mobile base.
 */
class Footwork {
public:
    // // constructor for Footwork
    Footwork();

    /**
     * @brief Moves the robot linearly.
     * 
     * Publishes Twist messages to move the robot forward or backward.
     * 
     * @param forward If true, moves the robot forward. If false, moves the robot backward.
     * @param num_publishes Number of times the command is published. 
     */
    void linear_motion(bool forward, int num_publishes=1);

    /**
     * @brief Rotates the robot in place or performs a wide turn.
     * 
     * Publishes Twist messages to rotate the robot clockwise or counterclockwise,
     * with an optional wide turn (adding forward motion).
     * 
     * @param clockwise If true, turns the robot clockwise. If false, turns counterclockwise.
     * @param num_publishes Number of times the command is published.
     * @param wide If true, adds forward motion to the turn for a wider turning arc.
     */
    void turn(bool clockwise, int num_publishes=1, bool wide=false);

private:
    // // ROs publisher to send Twsit messages to control the robot's base.
    ros::Publisher _footwork_pub;
    
    // // Limits the frequency of publshing Twist messages
    ros::Rate _rate;

    // // Delcare a Twist message type. 
    geometry_msgs::Twist _footwork_cmd;
};

#endif // FOOT_WORK_H