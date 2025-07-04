
#include "dashboard/low_pass_filter.h"

#include <cmath>

LowPassFilter::LowPassFilter(double alpha = 0.1){
    this->alpha = alpha;
}

double LowPassFilter::calculateVelocity(double vx, double vy, double vz) {
    double rawVelocity = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
        filteredVelocity = alpha * rawVelocity + (1 - alpha) * filteredVelocity;
        return filteredVelocity;
}
