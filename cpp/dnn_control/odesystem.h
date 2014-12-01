#ifndef ODESYSTEM_H
#define ODESYSTEM_H

#include "asteroid.h"

typedef double State[7];

class ODESystem
{
public:
    ODESystem(const Asteroid &asteroid_, const double *spacecraft_position, const double *spacecraft_velocity, const double &spacecraft_mass, const double &earth_acceleration_mul_spacecraft_specific_impulse);

    void operator()(const State &state_, State &d_state_dt, double time) const;

    const Asteroid &asteroid_;
    const double coef_earth_acceleration_mul_specific_impulse_;
    double thrust_[3];
    double perturbations_acceleration_[3];
    State state_;
};

#endif // ODESYSTEM_H
