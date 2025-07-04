#ifndef MOVING_AVERAGE_FILTER_H
#define MOVING_AVERAGE_FILTER_H

#include "IVelocityStrategy.h"
#include <cmath>
#include <deque>

class MovingAverageFilter : public IVelocityStrategy {
    private:
        int windowSize;
        double velocitySum;
        std::deque<double> velocityWindow;
    public:
        MovingAverageFilter(int windowSize = 10);
        double calculateVelocity(double vx, double vy, double vz);
};

#endif // MOVING_AVERAGE_FILTER_H
