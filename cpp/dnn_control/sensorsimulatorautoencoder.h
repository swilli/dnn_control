#ifndef SENSORSIMULATORAUTOENCODER_H
#define SENSORSIMULATORAUTOENCODER_H

#include "sensorsimulator.h"

class SensorSimulatorAutoencoder : public SensorSimulator {
public:
    static const unsigned int kDimensions;

    SensorSimulatorAutoencoder(SampleFactory &sample_factory, const Asteroid &asteroid);
    virtual ~SensorSimulatorAutoencoder();

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

    void Reset();

private:
    unsigned int cache_index_;

    bool first_simulation_;

    std::vector<double> sensor_maximum_absolute_ranges_;

    std::vector<double> sensor_values_cache_;
};

#endif // SENSORSIMULATORAUTOENCODER_H
