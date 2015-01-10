#ifndef ACCELEROMETERSENSORSIMULATOR_H
#define ACCELEROMETERSENSORSIMULATOR_H

#include "sensorsimulator.h"

class SensorSimulatorAcceleration : public SensorSimulator
{
public:
    SensorSimulatorAcceleration(const Asteroid &asteroid, const double &sensor_noise);
    ~SensorSimulatorAcceleration();

    virtual SensorData Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
};

#endif // ACCELEROMETERSENSORSIMULATOR_H
