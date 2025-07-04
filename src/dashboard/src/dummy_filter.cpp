
#include "dashboard/dummy_filter.h"

#include <cmath>

DummyFilter::DummyFilter(){}

double DummyFilter::calculateVelocity(double vx, double vy, double vz) {
    return sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
}
