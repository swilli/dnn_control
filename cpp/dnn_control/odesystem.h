#ifndef ODESYSTEM_H
#define ODESYSTEM_H

#include "vector.h"
#include "systemstate.h"
#include "asteroid.h"

#define ODESYS_FUEL_ENABLED true

class ODESystem {
public:
    ODESystem(const Asteroid &asteroid, const Vector3D &perturbations_acceleration, const Vector3D &thrust, const double &spacecraft_specific_impulse, const double &spacecraft_minimum_mass, const double &engine_noise);
    ODESystem(const ODESystem &other);

    ~ODESystem();

    void operator () (const SystemState &state, SystemState &d_state_dt, const double &time);

    class Exception {};
    class OutOfFuelException : public Exception {};

private:
    double spacecraft_specific_impulse_;
    double spacecraft_minimum_mass_;
    double engine_noise_;

    Vector3D thrust_;
    Vector3D perturbations_acceleration_;

    const Asteroid &asteroid_;
};

#endif // ODESYSTEM_H
