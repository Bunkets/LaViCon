#include "dashboard/threshold_filter.h"
#include <cmath>



ThresholdFilter::ThresholdFilter(double threshold = 0.01){
    this->threshold = threshold;
    this->lastVelocity = 0.0;
}

double ThresholdFilter::calculateVelocity(double vx, double vy, double vz) {
    double rawVelocity = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
    // If the change is below the threshold, use the last velocity
    if (fabs(rawVelocity - lastVelocity) < threshold) {
        return lastVelocity;
    } else {
        lastVelocity = rawVelocity;
        return rawVelocity;
    }
}
