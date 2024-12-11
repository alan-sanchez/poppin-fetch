#include "Footwork.h"

/**
 * @brief Constructor for Footwork.
 * 
 * Initializes the ROS publisher for the `/cmd_vel` topic, sets the default Twist message
 * to zero values (stopping the robot), and sets the publishing rate.
 */
Footwork::Footwork() : _rate(10) {

    // // Log a message to indicate the constructor is being executed
    ROS_INFO("Initializing Footwork class...");

    ros::NodeHandle nh;

    // // Advertise the `/cmd_vel` topic to allow publishing velocity commands.
    _footwork_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // // Initialize the Twsit message with zero default values, so Fetch doesn't move its base
    _footwork_cmd.linear.x = 0.0;
    _footwork_cmd.linear.y = 0.0;
    _footwork_cmd.linear.z = 0.0;
    _footwork_cmd.angular.x = 0.0;
    _footwork_cmd.angular.y = 0.0;
    _footwork_cmd.angular.z = 0.0;
};


/**
 * @brief Moves the robot linearly.
 * 
 * Publishes Twist messages to move the robot forward or backward.
 * 
 * @param forward If true, moves the robot forward. If false, moves the robot backward.
 * @param num_publishes Number of times the command is published. 
 */
void Footwork::linear_motion(bool forward, int num_publishes){
    // // Set the linear x velocity based on the `forward` parameter
    if (forward==true){
        _footwork_cmd.linear.x = 0.2;
    } else{
        _footwork_cmd.linear.x = -0.2;
    }

    // // Publish the Twist message
    for (int j=0; j<num_publishes; j++){
        // // Log a message indicating that we are waiting for the action server.
        // ROS_INFO("Publishing Twist message...");
        _footwork_pub.publish(_footwork_cmd);
        _rate.sleep();
    }

    // // Reset the Twist message to stop the robot
    _footwork_cmd.linear.x = 0.0;
    _footwork_cmd.angular.z = 0.0;
    // _footwork_pub.publish(_footwork_cmd);  // Send a stop command
}


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
void Footwork::turn(bool clockwise, int num_publishes, bool wide){
    // // Add forward motion for a wide turn if `wide` is true.
    if (wide == true){
        _footwork_cmd.linear.x = 0.4;
    }

    // // Set the angular z velocity based on the `clockwise` parameter.
    if (clockwise == true){
        _footwork_cmd.angular.z = -0.9; // Causes the robot to turn in the right direction. 
    } else{
        _footwork_cmd.angular.z = 1.0; // In simulation it turns slower in this direction (left turn).
    }

    // // Publish the Twist message multiple times to ensure the command is received.
    for (int i=0; i<num_publishes; i++){
        // // Log a message indicating that we are waiting for the action server.
        // ROS_INFO("Publishing Twist message...");
        _footwork_pub.publish(_footwork_cmd);
        _rate.sleep();
    }

    // // Reset the Twist message to stop the robot.
    _footwork_cmd.linear.x = 0.0;
    _footwork_cmd.angular.z = 0.0;  
    // _footwork_pub.publish(_footwork_cmd);  // Send a stop command  
}



