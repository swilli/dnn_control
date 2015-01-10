#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>

class SensorSimulatorFullState : public SensorSimulator
{
public:
    SensorSimulatorFullState(const Asteroid &asteroid, const double &sensor_noise);
    virtual ~SensorSimulatorFullState();

    virtual SensorData Simulate(const State &state, const Vector3D  &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
};

#endif // FULLSTATESENSORSIMULATOR_H
