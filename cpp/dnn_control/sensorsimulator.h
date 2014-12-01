#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

class SensorSimulator
{
public:
    SensorSimulator();

    void Simulate(double *state, double *perturbations_acceleration, const double &time, double *sensor_data) const;
};

#endif // SENSORSIMULATOR_H
