#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

class FullStateSensorSimulator : public SensorSimulator
{
public:
    FullStateSensorSimulator(const Asteroid &asteroid, const double &sensor_noise);
    virtual ~FullStateSensorSimulator();

    virtual void Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data);
};

#endif // FULLSTATESENSORSIMULATOR_H
