#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

class SensorSimulatorFullState : public SensorSimulator {
public:
    static const unsigned int kDimensions;

    SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid, const Vector3D &target_position);

    virtual ~SensorSimulatorFullState();

    virtual SensorData Simulate(const SystemState &state, const Vector3D  &, const Vector3D &, const double &time);

private:
    Vector3D target_position_;
};

#endif // FULLSTATESENSORSIMULATOR_H
