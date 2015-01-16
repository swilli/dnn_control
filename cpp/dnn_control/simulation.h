#ifndef SIMULATION_H
#define SIMULATION_H

#include "samplefactory.h"
#include "asteroid.h"
#include "odesystem.h"
#include "sensorsimulator.h"
#include "controller.h"
#include "systemstate.h"

class Simulation {
public:
    Simulation(const unsigned int &random_seed);
    virtual ~Simulation();

    virtual boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > Evaluate() = 0;

    Asteroid& AsteroidOfSystem();

protected:
    double simulation_time_;

    SampleFactory sample_factory_;

    Asteroid asteroid_;
    ODESystem system_;

    SensorSimulator *sensor_simulator_;
    Controller *controller_;

    SystemState initial_system_state_;
};

#endif // SIMULATION_H
