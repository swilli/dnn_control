#ifndef SIMULATION_H
#define SIMULATION_H

#include "samplefactory.h"
#include "asteroid.h"
#include "systemstate.h"

class Simulation {
public:
    Simulation(const unsigned int &random_seed);

    virtual ~Simulation();

    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> >  Evaluate() = 0;

    Asteroid& AsteroidOfSystem();

    class Exception {};
    class SizeMismatchException : public Exception {};

protected:
    unsigned int random_seed_;

    Asteroid asteroid_;

    double simulation_time_;

    double perturbation_noise_;

    double engine_noise_;

    double spacecraft_specific_impulse_;

    double spacecraft_maximum_thrust_;

    SystemState initial_system_state_;

    Vector3D target_position_;
};

#endif // SIMULATION_H
