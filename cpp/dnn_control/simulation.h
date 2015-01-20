#ifndef SIMULATION_H
#define SIMULATION_H

#include "samplefactory.h"
#include "asteroid.h"
#include "sensorsimulator.h"
#include "controller.h"
#include "systemstate.h"

class Simulation {
public:
    Simulation(const unsigned int &random_seed);
    Simulation(const Simulation &other);

    virtual ~Simulation();

    virtual boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > Evaluate() = 0;

    Asteroid& AsteroidOfSystem();

protected:
    unsigned int random_seed_;

    double simulation_time_;

    double perturbation_noise_;

    double engine_noise_;

    double spacecraft_specific_impulse_;

    SampleFactory sample_factory_;

    Asteroid asteroid_;

    SensorSimulator *sensor_simulator_;
    Controller *controller_;

    SystemState initial_system_state_;
};

#endif // SIMULATION_H
