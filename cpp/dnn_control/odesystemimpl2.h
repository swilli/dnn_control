#ifndef ODESYSTEMIMPL2_H
#define ODESYSTEMIMPL2_H

#include "vector.h"
#include "systemstate.h"
#include "asteroid.h"

class ODESystemImpl2 {
public:
    ODESystemImpl2(const Asteroid &asteroid, const Vector3D &perturbations_acceleration, const Vector3D &thrust, const double &spacecraft_specific_impulse, const double &engine_noise);
    ODESystemImpl2(const ODESystemImpl2 &other);

    ~ODESystemImpl2();

    void operator () (const SystemState &state, SystemState &d_state_dt, const double &time);

    class Exception {};
    class OutOfFuelException : public Exception {};

private:
    double spacecraft_specific_impulse_;
    double engine_noise_;

    Vector3D thrust_;
    Vector3D perturbations_acceleration_;

    const Asteroid &asteroid_;
};

#endif // ODESYSTEMIMPL2_H
