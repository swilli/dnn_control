#ifndef ACCELEROMETERSENSORSIMULATOR_H
#define ACCELEROMETERSENSORSIMULATOR_H

#include "sensorsimulator.h"


class SensorSimulatorAcceleration : public SensorSimulator
{
public:
    typedef boost::array<double, 4> SensorNoiseConfiguration;

    SensorSimulatorAcceleration(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    ~SensorSimulatorAcceleration();

    virtual SensorData Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    // Normal distributions for each sensor dimension
    std::vector<boost::variate_generator<boost::mt19937, boost::normal_distribution<> > > normal_distributions_;
};

#endif // ACCELEROMETERSENSORSIMULATOR_H
