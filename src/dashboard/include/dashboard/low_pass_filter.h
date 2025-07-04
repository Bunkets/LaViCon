#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include "IVelocityStrategy.h"
#include <cmath>

class LowPassFilter : public IVelocityStrategy {
    private:
        double alpha;  // Smoothing factor
        double filteredVelocity;
    public:
        LowPassFilter(double alpha);
        double calculateVelocity(double vx, double vy, double vz);

};

#endif // LOW_PASS_FILTER_H
