#include <ros/ros.h>
#include "velocity_commands_manager/velocity_commands_player.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_commands_player_node");
    ros::NodeHandle nh;
    std::string file_directory;
    std::string file_name;
    double play_rate;

    

    // Path to the recorded CSV file
    // Interval between playing each command (in seconds)
    // Set default values if parameters are not provided
    nh.param<std::string>("file_directory", file_directory, "../velocity_commands_files");
    nh.param<std::string>("file_name", file_name, "cmdvel-default.csv");
    nh.param<double>("play_rate", play_rate, 20);

    // Create the player object
    VelocityCommandsPlayer player(&nh, play_rate);
   
    // Play the commands from the file
    player.playCommandsFromFile(file_name, file_directory);
    return 0;
}
