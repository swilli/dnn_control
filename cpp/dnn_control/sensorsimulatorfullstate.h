#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>


class SensorSimulatorFullState : public SensorSimulator {
public:
    typedef boost::array<double, 7> SensorNoiseConfiguration;

    SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    virtual ~SensorSimulatorFullState();

    virtual SensorData Simulate(const SystemState &state, const Vector3D  &, const Vector3D &, const double &time);

private:
    SensorNoiseConfiguration sensor_noise_configuration_;
};

#endif // FULLSTATESENSORSIMULATOR_H
