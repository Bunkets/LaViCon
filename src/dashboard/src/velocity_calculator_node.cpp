#include <ros/ros.h>
#include "dashboard/velocity_calculator.h"
#include "dashboard/low_pass_filter.h"
#include "dashboard/moving_average_filter.h"
#include "dashboard/threshold_filter.h"
#include "dashboard/dummy_filter.h"


int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "velocity_calculator");
    ros::NodeHandle nh;

    //IVelocityStrategy* strategy = new LowPassFilter(0.1);
    //IVelocityStrategy* strategy = new MovingAverageFilter(5);
    //IVelocityStrategy* strategy = new ThresholdFilter(0.0001);
    IVelocityStrategy* strategy = new DummyFilter();
    VelocityContext context(strategy);
    // Create an instance of the VelocityCalculator class
    VelocityCalculator velocityCalculator(&nh, context);

    // Keep the node running
    ros::spin();

    return 0;
}
