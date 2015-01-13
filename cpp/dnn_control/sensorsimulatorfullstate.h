#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>


class SensorSimulatorFullState : public SensorSimulator
{
public:
    typedef boost::array<double, 7> SensorNoiseConfiguration;

    SensorSimulatorFullState(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    virtual ~SensorSimulatorFullState();

    virtual SensorData Simulate(const State &state, const Vector3D  &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    // Normal distributions for each sensor dimension
    std::vector<boost::variate_generator<boost::mt19937, boost::normal_distribution<> > > normal_distributions_;
};

#endif // FULLSTATESENSORSIMULATOR_H
