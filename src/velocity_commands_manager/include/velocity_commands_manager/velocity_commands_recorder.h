#ifndef VELOCITY_COMMANDS_RECORDER_H
#define VELOCITY_COMMANDS_RECORDER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fstream>

class VelocityCommandsRecorder {
public:
    VelocityCommandsRecorder(ros::NodeHandle* nh, double record_rate, const std::string& file_directory,
                             const std::string& file_name, const std::string& extension);
    ~VelocityCommandsRecorder();

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::Subscriber cmdVelSubscriber;  // Subscriber to the cmd_vel topic
    std::ofstream outFile;             // File to write the commands
    std::string fileName;              // Full file name (with timestamp and extension)
    ros::Rate rate;                 // Rate object to control recording rate
    double record_rate;                // Recording rate from the parameter server
};

#endif  // VELOCITY_COMMANDS_RECORDER_H
