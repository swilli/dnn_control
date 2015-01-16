#ifndef ODESYSTEMCONTROLLED_H
#define ODESYSTEMCONTROLLED_H

#include "vector.h"
#include "asteroid.h"
#include "samplefactory.h"
#include "systemstate.h"
#include "sensorsimulator.h"
#include "controller.h"

class ODESystem {
public:
    ODESystem();
    ODESystem(SampleFactory *sample_factory, const Asteroid &asteroid, SensorSimulator *sensor_simulator, Controller *controller, const double &spacecraft_specific_impulse, const double &perturbation_noise, const double &engine_noise);
    ~ODESystem();

    void operator () (const SystemState &state, SystemState &d_state_dt, const double &time);

    void SetThrust(const Vector3D &thrust);
    Vector3D PerturbationsAcceleration() const;

    class Exception {};
    class OutOfFuelException : public Exception {};

private:
    double spacecraft_specific_impulse_;
    double engine_noise_;
    double latest_control_input_time_;
    double min_control_interval_;

    Vector3D thrust_;
    Vector3D perturbations_acceleration_;

    Asteroid asteroid_;

    SampleFactory *sample_factory_;

    Controller *controller_;
    SensorSimulator *sensor_simulator_;
};

#endif // ODESYSTEMCONTROLLED_H
