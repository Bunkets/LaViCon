#ifndef VELOCITY_COMMANDS_PLAYER_H
#define VELOCITY_COMMANDS_PLAYER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <string>

class VelocityCommandsPlayer {
public:
    VelocityCommandsPlayer(ros::NodeHandle* nh, double play_rate);
    ~VelocityCommandsPlayer();

    // Method to read commands from a file and play them
    void playCommandsFromFile(const std::string& file_name, const std::string& file_directory);
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);

private:
    ros::Publisher cmdVelPublisher;  // Publisher for cmd_vel topic
    ros::Subscriber emergencyStopSubscriber;
    bool canMove;

    ros::Rate rate;
};

#endif  // VELOCITY_COMMANDS_PLAYER_H
