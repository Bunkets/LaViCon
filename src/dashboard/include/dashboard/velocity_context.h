#ifndef VELOCITY_CONTEXT_H
#define VELOCITY_CONTEXT_H

#include "IVelocityStrategy.h"

class VelocityContext {
    private:
        IVelocityStrategy* strategy;
    public:
        VelocityContext(IVelocityStrategy* strategy);
        void setStrategy(IVelocityStrategy* new_strategy);
        double calculateVelocity(double vx, double vy, double vz);
};

#endif // VELOCITY_CONTEXT_H
