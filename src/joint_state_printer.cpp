#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <algorithm> // for std::find

// // Callback function to handle incoming messages
void callback(const sensor_msgs::JointState::ConstPtr& msg) {
    // // Joint names and their positions
    std::vector<std::string> names = msg->name;
    std::vector<double> positions = msg->position;

    // Define the joints we're interested in
    std::vector<std::string> joints = {
        "torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint",
        "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint"
    };

    // Store the joint positions for the specified joints
    std::vector<double> joint_positions;

    for (const auto& joint : joints) {
        // Find the index of each joint name in the names vector
        auto it = std::find(names.begin(), names.end(), joint);
        if (it != names.end()) {
            int index = std::distance(names.begin(), it);
            // Round the position to 2 decimal places
            joint_positions.push_back(std::round(positions[index] * 100) / 100.0);
        }
    }

    // Print the joint positions
    for (const auto& pos : joint_positions) {
        std::cout << pos << " ";
    }
    std::cout << std::endl;

    // Shut down ROS node after printing
    ros::shutdown();
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "joint_state_printer");
    ros::NodeHandle nh;

    // Subscribe to the "joint_states" topic and specify the callback function
    ros::Subscriber subscriber = nh.subscribe("joint_states", 1000, callback);

    // Give control to ROS to process incoming messages
    ros::spin();

    return 0;
}
