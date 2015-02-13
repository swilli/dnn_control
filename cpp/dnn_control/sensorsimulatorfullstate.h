#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

class SensorSimulatorFullState : public SensorSimulator {
public:
    static const unsigned int kDimensions;

    SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid);

    virtual ~SensorSimulatorFullState();

    virtual SensorData Simulate(const SystemState &state, const Vector3D  &, const Vector3D &, const double &time);
};

#endif // FULLSTATESENSORSIMULATOR_H
