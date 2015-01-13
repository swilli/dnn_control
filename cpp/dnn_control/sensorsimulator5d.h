#ifndef SENSORSIMULATOR5D_H
#define SENSORSIMULATOR5D_H

#include "sensorsimulator.h"

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>


class SensorSimulator5D : public SensorSimulator
{
public:
    typedef boost::array<double, 5> SensorNoiseConfiguration;

    SensorSimulator5D(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    virtual ~SensorSimulator5D();

    virtual SensorData Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    // Normal distributions for each sensor dimension
    std::vector<boost::variate_generator<boost::mt19937, boost::normal_distribution<> > > normal_distributions_;
};

#endif // SENSORSIMULATOR5D_H
