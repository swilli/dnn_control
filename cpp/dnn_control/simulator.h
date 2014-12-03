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
    Simulator(Asteroid &asteroid, SensorSimulator &sensor_simulator, SpacecraftController &spacecraft_controller, const double &control_frequency, const double &perturbation_noise);

    void InitSpacecraft(const double *position, const double *velocity, const double &mass, const double &specific_impulse);
    void InitSpacecraftSpecificImpulse(const double &specific_impulse);

    void NextState(const State &state, const double *thrust, const double &time, State &next_state);

    int Run(const double &time, const bool &collect_data,  std::vector<std::vector<double> > **positions, std::vector<std::vector<double> > **heights);

    double ControlInterval() const;

private:
    void SimulatePerturbations(double *perturbations);

    ODESystem system_;
    SensorSimulator &sensor_simulator_;
    SpacecraftController &spacecraft_controller_;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
    double control_interval_;
};

#endif // SIMULATOR_H
