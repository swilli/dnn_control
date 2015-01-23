#ifndef FULLSTATESENSORSIMULATOR_H
#define FULLSTATESENSORSIMULATOR_H

#include "sensorsimulator.h"

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>

class SensorSimulatorFullState : public SensorSimulator {
public:
    static const unsigned int kDimensions;

    SensorSimulatorFullState(SampleFactory sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    SensorSimulatorFullState(const SensorSimulatorFullState &other);

    virtual ~SensorSimulatorFullState();

    virtual SensorSimulator* Clone() const;

    virtual SensorData Simulate(const SystemState &state, const Vector3D  &, const Vector3D &, const double &time);

};

#endif // FULLSTATESENSORSIMULATOR_H
