#include "dashboard/moving_average_filter.h"
#include <cmath>
#include <deque>

MovingAverageFilter:: MovingAverageFilter(int windowSize){
    this->windowSize = windowSize;
    this->velocitySum = 0.0;
}


// Implement the velocity calculation using moving average
double MovingAverageFilter::calculateVelocity(double vx, double vy, double vz) {
        double rawVelocity = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
        
        if (velocityWindow.size() >= windowSize) {
            velocitySum -= velocityWindow.front();  // Remove the oldest value from the sum
            velocityWindow.pop_front();  // Remove the oldest value from the window
        }
        velocityWindow.push_back(rawVelocity);  // Add the new velocity
        velocitySum += rawVelocity;

        return velocitySum / velocityWindow.size();  // Return the average
}
