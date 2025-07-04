
#include "dashboard/velocity_context.h"


VelocityContext::VelocityContext(IVelocityStrategy* strategy){
    this->strategy = strategy;
}

// Set the current strategy
void VelocityContext::setStrategy(IVelocityStrategy* new_strategy) {
        strategy = new_strategy;
}

// Calculate velocity using the current strategy
double VelocityContext::calculateVelocity(double vx, double vy, double vz) {
    if (strategy) {
        return strategy->calculateVelocity(vx, vy, vz);
    }
    return 0.0;  // Default velocity if no strategy is set
}
