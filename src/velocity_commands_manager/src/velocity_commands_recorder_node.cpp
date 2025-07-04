#include <ros/ros.h>
#include <memory> //std::unique_ptr
#include "velocity_commands_manager/velocity_commands_recorder.h"

// Global pointer to manage the recorder object and ensure destructor is called
std::unique_ptr<VelocityCommandsRecorder> recorderPtr;

void shutdonwRecorder(){
    ROS_INFO("Shutting down velocity_commands_recorder_node...");
    recorderPtr.reset();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_commands_recorder_node");
    ros::NodeHandle nh;

    ROS_INFO("Launched velocity_commands_recorder_node...");
    
    double record_rate;
    std::string file_directory;
    std::string file_name;
    std::string extension;

    nh.param<double>("record_rate", record_rate, 20.0);
    nh.param<std::string>("file_directory", file_directory, "../velocity_commands_files/");
    nh.param<std::string>("file_name", file_name, "cmdvel");
    nh.param<std::string>("extension", extension, "csv");

    ROS_INFO("Parameters:\n");
    ROS_INFO("record_rate: %.2f", record_rate);
    ROS_INFO("file_directory: %s", file_directory.c_str());
    ROS_INFO("file_name: %s", file_name.c_str());
    ROS_INFO("extension: ", extension.c_str());


    recorderPtr = std::make_unique<VelocityCommandsRecorder>(&nh, record_rate, file_directory, file_name, extension);

    ros::spin();  // Keep the node running

    //shutdonwRecorder();

    return 0;
}
