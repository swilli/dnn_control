#ifndef SENSORSIMULATOR5D_H
#define SENSORSIMULATOR5D_H

#include "sensorsimulator.h"

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>

class SensorSimulator5D : public SensorSimulator
{
public:
    SensorSimulator5D(const Asteroid &asteroid, const double &sensor_noise);
    virtual ~SensorSimulator5D();

    virtual void Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data);

private:
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
};

#endif // SENSORSIMULATOR5D_H
