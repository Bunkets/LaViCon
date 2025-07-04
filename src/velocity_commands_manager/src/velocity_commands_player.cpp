#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "velocity_commands_manager/velocity_commands_player.h"

VelocityCommandsPlayer::VelocityCommandsPlayer(ros::NodeHandle* nh, double play_rate): rate(play_rate) {
    canMove = false;

    // Publisher for cmd_vel topic
    cmdVelPublisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    ROS_INFO("Publishing to /cmd_vel");
    
   // Subscribe to emergency stop topic
    emergencyStopSubscriber = nh->subscribe("/emergency_stop", 1, &VelocityCommandsPlayer::emergencyStopCallback, this);

    // Wait for the emergency stop to be released
    ROS_INFO("Waiting for emergency stop release to play control commands");
    do {
        ros::spinOnce();  // Process incoming messages
        if (canMove) {
            break;
        }
        rate.sleep();
    } while(ros::ok());
}

void VelocityCommandsPlayer::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg){
    canMove = msg->data;
    // if (canMove) {
    //     ROS_INFO("Emergency stop released, resuming command playback.");
    // } else {
    //     ROS_WARN("Emergency stop triggered, pausing command playback.");
    // }
}

void VelocityCommandsPlayer::playCommandsFromFile(const std::string& file_name, const std::string& file_directory) {
    
    // Combine file_directory and file_name into a full file path
    std::string full_file_path;
    if (file_directory.back() == '/') {
        full_file_path = file_directory + file_name;
    } else {
        full_file_path = file_directory + "/" + file_name;
    }

    std::ifstream inFile(full_file_path.c_str());
    if (!inFile.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_name.c_str());
        return;
    }

    std::string line;
    bool first_line = true;

    // Read each line from the file
    while (std::getline(inFile, line) && ros::ok()) {
        
        ros::spinOnce();  // Process incoming messages, such as emergency stop

        // Skip the first line (header)
        if (first_line) {
            first_line = false;
            continue;
        }

        // Parse the line
        std::istringstream lineStream(line);
        std::string value;
        geometry_msgs::Twist twist_msg;

        // Parse linear velocities
        std::getline(lineStream, value, ',');
        twist_msg.linear.x = std::stod(value);
        std::getline(lineStream, value, ',');
        twist_msg.linear.y = std::stod(value);
        std::getline(lineStream, value, ',');
        twist_msg.linear.z = std::stod(value);

        // Parse angular velocities
        std::getline(lineStream, value, ',');
        twist_msg.angular.x = std::stod(value);
        std::getline(lineStream, value, ',');
        twist_msg.angular.y = std::stod(value);
        std::getline(lineStream, value, ',');
        twist_msg.angular.z = std::stod(value);

        // Stop if emergency stop is triggered
        if (!canMove) {
            ROS_WARN("Emergency stop triggered, stopping command playback.");
            break;
        }
        
        // Publish the Twist message to the /cmd_vel topic
        cmdVelPublisher.publish(twist_msg);

        ROS_INFO("Published velocity command: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)",
                 twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                 twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);

        // Sleep for the specified interval before reading the next command
        rate.sleep();
    }

    inFile.close();
}

VelocityCommandsPlayer::~VelocityCommandsPlayer() {
    ROS_INFO("VelocityCommandsPlayer shutting down");
}
