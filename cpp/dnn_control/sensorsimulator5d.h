#ifndef SENSORSIMULATOR5D_H
#define SENSORSIMULATOR5D_H

#include "sensorsimulator.h"


class SensorSimulator5D : public SensorSimulator
{
public:
    SensorSimulator5D(const Asteroid &asteroid, const double &sensor_noise);
    virtual ~SensorSimulator5D();

    virtual void Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data);
};

#endif // SENSORSIMULATOR5D_H
