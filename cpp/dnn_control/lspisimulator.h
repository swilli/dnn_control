#ifndef LSPISIMULATOR_H
#define LSPISIMULATOR_H

#include "vector.h"

#include "systemstate.h"
#include "asteroid.h"
#include "samplefactory.h"

class LSPISimulator {
public:
    LSPISimulator(const unsigned int &random_seed);


    // (s', t', end_of_sim) = NextState(s, t, a)
    boost::tuple<SystemState, double, bool> NextState(const SystemState &state, const double &time, const Vector3D &thrust);

    // Returns the asteroid the simulator works with
    Asteroid &AsteroidOfSystem();

    // Returns the SampleFactory the simulator works with
    SampleFactory &SampleFactoryOfSystem();

    // The frequency the controller get's triggered for control 
    double ControlFrequency() const;

    // The spacecraft's maximum mass
    double SpacecraftMaximumMass() const;

private:
    // This class is used to observe the actual simulated time in case of an exception (out of fuel, crash)
    class Observer {
    public:
        Observer(double &time) : time_(time){}
        void operator () (const SystemState &, const double &current_time) {
            time_ = current_time;
        }
    private:
        double &time_;
    };

    // The seed with which the simulator was created
    unsigned int random_seed_;

    // Minimum step size of the adaptive integrator
    double minimum_step_size_;

    // Controller trigger frequency
    double control_frequency_;

    // Spacecraft's Isp noise
    double spacecraft_engine_noise_;

    // Spacecraft's Isp
    double spacecraft_specific_impulse_;

    // Spacecraft's maximum thrust
    double spacecraft_maximum_thrust_;

    // Spacecraft's minimum mass
    double spacecraft_minimum_mass_;

    // Spacecraft's maximum mass
    double spacecraft_maximum_mass_;

    // Random perturbation mean during the simulation
    double perturbation_mean_;

    // Random perturbation standard deviation during the simulation
    double perturbation_noise_;

    // The asteroid the simulator works with
    Asteroid asteroid_;

    // The SampleFactory the simulator works with
    SampleFactory sample_factory_;
};

#endif // LSPISIMULATOR_H
