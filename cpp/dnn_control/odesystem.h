#ifndef ODESYSTEM_H
#define ODESYSTEM_H

#include "asteroid.h"
#include "vector.h"

typedef double State[7];

class ODESystem
{
public:
    ODESystem(Asteroid &asteroid);

    void operator()(const State &state, State &d_state_dt, double time) const;

    Asteroid &asteroid_;
    double coef_earth_acceleration_mul_specific_impulse_;
    Vector3D thrust_;
    Vector3D perturbations_acceleration_;
    State state_;
};

#endif // ODESYSTEM_H
