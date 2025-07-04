#ifndef VELOCITY_CALCULATOR_H
#define VELOCITY_CALCULATOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <dashboard/VelocityStamped.h>  //custom Velocity message
#include "velocity_context.h"


class VelocityCalculator {
    private:
        ros::Subscriber odomSubscriber;
        ros::Publisher velocityPublisher;
        VelocityContext& context;
        dashboard::VelocityStamped velocityMsg;
        int sequenceNumber;
        int velocityRate;
        
    public:
        VelocityCalculator(ros::NodeHandle* nh,  VelocityContext& context);

        // Callback for odometry messages
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void setStrategy(IVelocityStrategy* strategy);
};

#endif  // VELOCITY_CALCULATOR_H
