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
    /**
     * @brief Constructor for Footwork
     * 
     * [ADD HERE]
     */
    Footwork();

    /**
     * @brief
     * 
     * [ADD HERE]
     */
    void turn(int iterations, double z, bool clockwise);

    /**
     * @brief
     * 
     * [ADD HERE]
     */
    void move_base(int interations, bool forward);

private:
    /**
     * @brief
     *
     * [ADD HERE] 
     */
    ros::Publisher _footwork_pub;
    geometry_msgs::Twist _twist_cmd;
    ros::Rate _rate;
};

#endif // FOOT_WORK_H
