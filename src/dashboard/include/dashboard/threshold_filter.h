#ifndef THRESHOLD_FILTER_H
#define THRESHOLD_FILTER_H

#include "IVelocityStrategy.h"
#include <cmath>

class ThresholdFilter : public IVelocityStrategy {
    private:
        double threshold;
        double lastVelocity;

    public:
        ThresholdFilter(double threshold);
        double calculateVelocity(double vx, double vy, double vz);
};

#endif // THRESHOLD_FILTER_H
