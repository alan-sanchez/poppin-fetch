#include "FollowTrajectoryClient.h"

/**
 * @brief Constructor for the FollowTrajectoryClient class.
 *
 * Initializes the action client for the "" action server and waits for 
 * the server to become available. 
 */
FollowTrajectoryClient::FollowTrajectoryClient() : _client("/arm_with_torso_controller/follow_joint_trajectory") {
    // // 
    ROS_INFO("Waiting for  ...");

    // // Wait until the action server for the .......
    _client.waitForServer();

    // // 
    ROS_INFO("..... action server has connected");

    // // 
    _joint_names = {"torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", 
                    "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};    
}


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
bool FollowTrajectoryClient::move_joints_to(const std::vector<double>& positions, double duration, const std::string& base_motion, const std::string& head_motion) {
    // //
    _base_motion = base_motion;
    _head_motion = head_motion;

    // // Validate positions length with conditional statement
    // // Note:`.size()` returns an unsigned int of the number of elements in the stored container.
    if (positions.size() != _joint_names.size()) {
        ROS_ERROR("Invalid trajectory position: expected %ld values, but got %ld", 
                  _joint_names.size(), positions.size());
        return false;
    }

    // // Create a JointTrajectory message (pulled from the library from the header file)
    trajectory_msgs::JointTrajectory trajectory;

    // // Assign the names of the joints being controlled to the trajectory message.
    // // `_joint_names` is a vector of strings containing the names of the joints (e.g., "shoulder_pan_joint").
    trajectory.joint_names = _joint_names;

    // // Create a JointTrajectoryPoint message to represent a single waypoint in the trajectory.
    // // Each point specifies the desired state (position, velocity, acceleration) for each joint at a specific time.
    trajectory_msgs::JointTrajectoryPoint point;

    // // Assign the desired positions for the joints to the `positions` field of the point.
    // // The `positions` vector is an input parameter containing target joint angles or positions.
    point.positions = positions;

    // // Assign zero velocities for all joints.
    // // A vector of the same size as `positions` is created and initialized with 0.0 for each joint.
    point.velocities = std::vector<double>(positions.size(), 0.0);

    // // Assign zero accelerations for all joints.
    // // Similar to velocities, a vector of the same size as `positions` is created and initialized with 0.0.
    point.accelerations = std::vector<double>(positions.size(), 0.0);

    // // Set the time from the start of the trajectory for reaching this point.
    // // `ros::Duration(duration)` specifies how long (in seconds) it should take to reach this waypoint
    // // from the beginning of the trajectory.
    point.time_from_start = ros::Duration(duration);

    // // Add the configured trajectory point to the `points` field of the trajectory message.
    trajectory.points.push_back(point);


    // // Create and send a goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;

    // // goal: Specifies what the robot should do.
    // // SimpleDoneCallback: Handles goal completion (sucess, failure, or preemption).
    // // SimpleActiveCallback: Handles the activation of the goal.
    // // Process periodic feedback from the server
    _client.sendGoal(goal,
                     actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>::SimpleDoneCallback(),
                     actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>::SimpleActiveCallback(),
                     boost::bind(&FollowTrajectoryClient::feedbackCallback, this, _1));

    // // Wait for the result
    bool success = _client.waitForResult();
    if (!success) {
        ROS_ERROR("Failed to execute trajectory");
    }
    return success;
}


/**
 * @brief Callback function to handle feedback from the action server.
 *  
 * This feedback also publishes the base of the robot to move while the robot is executing an action 
 *
 * @param feedback The feedback message from the action server.
 */
void FollowTrajectoryClient::feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback) {
    if (_base_motion == "Forward") {
        this->linear_motion(true); //

    } else if (_base_motion == "Backward") {
        this->linear_motion(false);

    } else if (_base_motion == "Right Turn") {
        this->turn(true);

    } else if (_base_motion == "Left Turn") {
        this->turn(false);

    } else if (_base_motion == "Wide Right Turn") {
        this->turn(true, 1, true);

    } else if (_base_motion == "Wide Left Turn") {
        this->turn(false, 1, true);
    }
}