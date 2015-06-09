#ifndef ODESYSTEM_H
#define ODESYSTEM_H

#include "vector.h"
#include "systemstate.h"
#include "asteroid.h"

class ODESystem {
    /*
    * This class represents an ordinary differential equation system.
    */
public:
    ODESystem(const Asteroid &asteroid, const Vector3D &perturbations_acceleration, const Vector3D &thrust, const double &spacecraft_specific_impulse, const double &spacecraft_minimum_mass, const double &engine_noise, const bool &fuel_usage_enabled);
    ODESystem(const ODESystem &other);


    void operator () (const SystemState &state, SystemState &d_state_dt, const double &time);

    // ODESystem can throw the following exceptions
    class Exception {};
    class OutOfFuelException : public Exception {};

private:
    // Is fuel usage enabled
    bool fuel_usage_enabled_;

    // The spacecraft's Isp
    double spacecraft_specific_impulse_;

    // The spacecraft's minimum mass
    double spacecraft_minimum_mass_;

    /// The spacecraft's Isp
    double engine_noise_;

    // The thrust the ode system will be integrated with
    Vector3D thrust_;

    // The perturbation accelerations the ode system will be integrated with
    Vector3D perturbations_acceleration_;

    // The asteroid the ode system works with
    const Asteroid &asteroid_;
};

#endif // ODESYSTEM_H
