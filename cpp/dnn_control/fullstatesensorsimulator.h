#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>

class FullStateSensorSimulator : public SensorSimulator
{
public:
    FullStateSensorSimulator(const Asteroid &asteroid, const double &sensor_noise);
    virtual ~FullStateSensorSimulator();

    virtual void Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data);

private:
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
};

#endif // FULLSTATESENSORSIMULATOR_H
