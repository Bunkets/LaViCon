#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <dashboard/VelocityStamped.h>
#include"dashboard/velocity_calculator.h"

VelocityCalculator::VelocityCalculator(ros::NodeHandle* nh, VelocityContext& context):  context(context) {
    sequenceNumber = 0;
    
    nh->param<int>("velocity_rate", velocityRate, 10);
    
    // Subscribe to the odometry topic
    odomSubscriber = nh->subscribe("odom", 1, &VelocityCalculator::odomCallback, this);
    ROS_INFO("Subscribed to /odom");

    // Advertise the velocity topic
    velocityPublisher = nh->advertise<dashboard::VelocityStamped>("velocity", 1, true);
}

void VelocityCalculator::setStrategy(IVelocityStrategy* strategy){            
    context.setStrategy(strategy);
}

void VelocityCalculator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ros::Rate rate(velocityRate);
    // Get the linear velocity components
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;

    // Calculate the magnitude of the velocity vector
    double velocity =  context.calculateVelocity(vx, vy, 0); // ignore vz

    // publish the velocity message
    velocityMsg.header.seq = sequenceNumber;
    velocityMsg.header.stamp = ros::Time::now();
    velocityMsg.header.frame_id = msg->header.frame_id;
    velocityMsg.velocity = velocity;
    velocityPublisher.publish(velocityMsg);
    sequenceNumber++;
    rate.sleep();
}
