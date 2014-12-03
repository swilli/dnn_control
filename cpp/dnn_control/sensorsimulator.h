#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

#include "asteroid.h"
#include "odesystem.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

typedef double SensorData[5];

class SensorSimulator
{
public:
    SensorSimulator(const Asteroid &asteroid, const double &sensor_noise);

    void Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data);

private:
    const Asteroid &asteroid_;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
};

#endif // SENSORSIMULATOR_H
