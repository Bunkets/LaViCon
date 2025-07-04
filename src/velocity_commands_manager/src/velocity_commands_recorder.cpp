#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <iomanip>  // For formatting the timestamp
#include <ctime>
#include <sys/stat.h>  // For checking/creating directories
#include "velocity_commands_manager/velocity_commands_recorder.h"

VelocityCommandsRecorder::VelocityCommandsRecorder(ros::NodeHandle* nh, double record_rate, const std::string& file_directory,
                                                   const std::string& file_name, const std::string& extension)
    : rate(record_rate){

    cmdVelSubscriber = nh->subscribe("cmd_vel", 10, &VelocityCommandsRecorder::cmdVelCallback, this);
    ROS_INFO("Subscribed to cmd_vel");

    // Check if the directory exists, create it if it doesn't
    struct stat info;
    if (stat(file_directory.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        if (mkdir(file_directory.c_str(), 0777) == -1) {
            ROS_ERROR("Error creating directory: %s", file_directory.c_str());
            return;
        } else {
            ROS_INFO("Created directory: %s", file_directory.c_str());
        }
    }
    
    // Get current time and create a file name with a timestamp
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    std::ostringstream fileNameStream;
    fileNameStream << file_directory << "/" << file_name << "-"
                   << std::put_time(localTime, "%Y-%m-%d-%H-%M-%S") << "." << extension;
    fileName = fileNameStream.str();

    // Open the file and write the header for the CSV
    outFile.open(fileName.c_str());
    if (outFile.is_open()) {
        outFile << "linear_x,linear_y,linear_z,angular_x,angular_y,angular_z\n";
    } else {
        ROS_ERROR("Failed to open file: %s", fileName.c_str());
    }
}

void VelocityCommandsRecorder::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!outFile.is_open()) {
        ROS_ERROR("File is not open for writing.");
        return;
    }

    // Set the output to fixed-point notation to prevent scientific notation
    //outFile << std::fixed;

    // Write the linear and angular velocities to the CSV file
    outFile << msg->linear.x << ","
            << msg->linear.y << ","
            << msg->linear.z << ","
            << msg->angular.x << ","
            << msg->angular.y << ","
            << msg->angular.z << "\n";

    ROS_INFO("Recorded velocity command: linear(%.6f, %.6f, %.6f), angular(%.6f, %.6f, %.6f)",
             msg->linear.x, msg->linear.y, msg->linear.z,
             msg->angular.x, msg->angular.y, msg->angular.z);

    // Sleep to maintain the record rate
    rate.sleep();
}

VelocityCommandsRecorder::~VelocityCommandsRecorder() {
    // Close the file when the object is destroyed
    if (outFile.is_open()) {
        outFile.close();
        ROS_INFO("Closed file: %s", fileName.c_str());
    }
}
