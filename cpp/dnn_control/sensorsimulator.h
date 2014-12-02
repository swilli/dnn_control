#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

#include "asteroid.h"

class SensorSimulator
{
public:
    SensorSimulator(const Asteroid &asteroid);

    void Simulate(double *state, double *perturbations_acceleration, const double &time, double *sensor_data) const;

private:
    const Asteroid &asteroid_;
};

#endif // SENSORSIMULATOR_H
