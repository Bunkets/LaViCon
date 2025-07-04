#ifndef DUMMY_FILTER_H
#define DUMMY_FILTER_H

#include "IVelocityStrategy.h"
#include <cmath>

class DummyFilter : public IVelocityStrategy {
    public:
        DummyFilter();
        double calculateVelocity(double vx, double vy, double vz);
};

#endif // DUMMY_FILTER_H
