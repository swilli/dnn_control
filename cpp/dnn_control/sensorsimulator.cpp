#include "sensorsimulator.h"

SensorSimulator::SensorSimulator(const Asteroid &asteroid) : asteroid_(asteroid)
{
}

void SensorSimulator::Simulate(double *state, double *perturbations_acceleration, const double &time, double *sensor_data) const
{
    const double position[3] = {state[0], state[1], state[2]};
    double surface_point[3];
    double distance = 0.0;
    asteroid_.NearestPointOnSurface(position, surface_point, &distance);
}
