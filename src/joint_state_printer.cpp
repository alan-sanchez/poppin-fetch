// // Imports the ROS client library for C++. This provides the necessary 
// // functions and classes for creating nodes, subscribing to topics, logging, 
// // and managing node execution
#include <ros/ros.h>

// // Imports the JointState message type, which is part of the sensor_msgs package.
#include <sensor_msgs/JointState.h>

// // #include <vector> and #include <string>: Includes the std::vector and std::string 
// // classes from the C++ standard library, used to handle dynamic arrays of strings 
// // and doubles
#include <vector>
#include <string>

// // This library provides standard functions, includiong std::find, which is used to
// // search for an element within a container
#include <algorithm> // for std::find

/*
* Function: callback
*
* Description: Callback function to handle incoming `JointState` messages
* and printing those values in the terminal
* 
* Parameters:
*   msg (JointState): The JoinState message type
*/
void callback(const sensor_msgs::JointState::ConstPtr& msg) {
    // // We use -> to access the pointed-to object (ConstPtr&) members of the object. 
    // // In this instance, the code is acessing the name and position members
    std::vector<std::string> names = msg->name;
    std::vector<double> positions = msg->position;

    // // Define the joints we're interested as a string vector
    std::vector<std::string> joints = {
        "torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint",
        "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint"
    };

    // // Store the joint positions for the specified joints
    std::vector<double> joint_positions;

    // // Loop iterates through each joint in joints, checking if it exists in names.
    // // const: This ensures that each element is read-only inside the loop (We don't 
    // //   accidentally modify the joint name)
    // // auto&: auto deduces the type automatically (std::string). The & makes joint a reference to 
    // //   avoid copying the string, which is more efficient, especially for larger objects
    for (const auto& joint : joints) {
        // // Find the index of each joint name in the names vector
        // // std::find returns an iterator first occurrence of the specified element within a range.
        // // 
        auto it = std::find(names.begin(), names.end(), joint);
        if (it != names.end()) {
            int index = std::distance(names.begin(), it);
            // // Round the position to 2 decimal places
            joint_positions.push_back(std::round(positions[index] * 100) / 100.0);
        }
    }

    // // Print the joint positions
    for (const auto& pos : joint_positions) {
        std::cout << pos << " ";
    }
    std::cout << std::endl;

    // // Shut down ROS node after printing
    ros::shutdown();
}

int main(int argc, char** argv) {
    // // Initialize the ROS node
    ros::init(argc, argv, "joint_state_printer");
    ros::NodeHandle nh;

    // Subscribe to the "joint_states" topic and specify the callback function
    ros::Subscriber subscriber = nh.subscribe("joint_states", 1000, callback);

    // Give control to ROS to process incoming messages
    ros::spin();

    return 0;
}
