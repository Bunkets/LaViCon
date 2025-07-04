#ifndef IVELOCITY_STRATEGY_H
#define IVELOCITY_STRATEGY_H

class IVelocityStrategy {
public:
    virtual ~IVelocityStrategy() {}
    
    // Pure virtual function to calculate velocity based on the strategy
    virtual double calculateVelocity(double vx, double vy, double vz) = 0;
};

#endif // IVELOCITY_STRATEGY_H
