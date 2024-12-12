#ifndef FOLLOW_TRAJECTORY_CLIENT_H
#define FOLLOW_TRAJECTORY_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>
#include <vector>
#include "Footwork.h" // Include the Footwork header
#include "PointHeadClient.h" // Include the Head client header

/**
 * @class FollowTrajectoryClient
 * @brief A class for controlling a robot's arm and torso using joint trajectory actions.
 * 
 * This class communicates with the `/arm_with_torso_controller/follow_joint_trajectory` action server
 * to send joint trajectory goals and monitor feedback.
 */
class FollowTrajectoryClient: public Footwork, public PointHeadClient {
public:
    // // Constructor for FollowTrajectoryClient
    FollowTrajectoryClient();

    /**
     * @brief Sends a joint trajectory goal to the action server.
     * 
     * This method allows the robot to move to a specified set of joint positions over a given duration.
     * Note: we use `const` and `&` for the parameters so they are not modified by the function (immutable)
     * and vectors and strings are potentially large objects, and copying would be inefficient, so we reference.
     * 
     * @param positions A vector of desired joint positions.
     * @param duration Duration for the movement (in seconds).
     * @param base_motion Optional parameter for base motion (e.g., "Forward").
     * @param head_motion Optional parameter for head motion (e.g., "Move").
     *
     * @return True if the goal is successfully sent and completed, otherwise false.
     */
    bool move_joints_to(const std::vector<double>& positions, 
                        double duration = 1.0, 
                        const std::string& base_motion="", 
                        const std::vector<double>& coordinates={});

private:
    /**
     * @brief Callback function to handle feedback from the action server.
     *  
     * @param feedback The feedback message from the action server.
     */
    void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);

    // // Action client for sending trajectory goals
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _client;

    // // Names of the joints controlled by the client
    std::vector<std::string> _joint_names;

    // // Coordiantes where the head will look at
    std::vector<double> _coordinates;

    // // Parameters for optional base motion
    std::string _base_motion;
    
};

#endif // FOLLOW_TRAJECTORY_CLIENT_H
