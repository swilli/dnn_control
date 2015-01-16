#ifndef ACCELEROMETERSENSORSIMULATOR_H
#define ACCELEROMETERSENSORSIMULATOR_H

#include "sensorsimulator.h"


class SensorSimulatorAcceleration : public SensorSimulator {
public:
    typedef boost::array<double, 4> SensorNoiseConfiguration;

    SensorSimulatorAcceleration(SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    virtual ~SensorSimulatorAcceleration();

    virtual SensorData Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    SensorNoiseConfiguration sensor_noise_configuration_;
};

#endif // ACCELEROMETERSENSORSIMULATOR_H
