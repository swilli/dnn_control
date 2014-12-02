#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "asteroid.h"
#include "sensorsimulator.h"
#include "spacecraftcontroller.h"
#include "odesystem.h"

#include <fstream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


class Simulator
{
public:
    Simulator(const Asteroid &asteroid, const double *spacecraft_position, const double *spacecraft_velocity, const double &spacecraft_mass, const double &spacecraft_specific_impulse,
    const SensorSimulator &sensor_simulator, const SpacecraftController &spacecraft_controller, const double &control_frequency, std::ostream *result_file);

    void Run(const double &time);

private:
    void SimulatePerturbations(double *perturbations);

    ODESystem system_;
    const SensorSimulator &sensor_simulator_;
    const SpacecraftController &spacecraft_controller_;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
    double control_interval_;
    std::ostream *result_file_;
};

#endif // SIMULATOR_H
