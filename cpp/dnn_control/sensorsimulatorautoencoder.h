#ifndef SENSORSIMULATORAUTOENCODER_H
#define SENSORSIMULATORAUTOENCODER_H

#include "sensorsimulator.h"

class SensorSimulatorAutoencoder : public SensorSimulator {
    /*
    * This class generates the artificial sensor data required for a neural network controller which uses an Autoencoder to compress this generated sensor data.
    */
public:
    // The number of output dimensions the sensor simulator will generate
    static const unsigned int kDimensions;

    SensorSimulatorAutoencoder(SampleFactory &sample_factory, const Asteroid &asteroid);
    virtual ~SensorSimulatorAutoencoder();

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

    // As the SensorSimulator can generate histories of sensor streams, resetting will reset the previous history
    void Reset();

    // SensorSimulatorAutoencoder can throw the following exceptions
    class RangeMalConfigurationException : public Exception {};

private:
    // The history is implemented with a circular buffer. This index points to the first element in the history.
    unsigned int cache_index_;

    // The number of calls to Simulate since the last call to Reset or construction
    unsigned int num_simulations_;

    // The history of sensor data values
    std::vector<double> sensor_values_cache_;
};

#endif // SENSORSIMULATORAUTOENCODER_H
