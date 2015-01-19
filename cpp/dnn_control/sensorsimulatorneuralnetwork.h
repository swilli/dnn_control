#ifndef SENSORSIMULATORNEURALNETWORK_H
#define SENSORSIMULATORNEURALNETWORK_H

#include "sensorsimulator.h"

#define SENSOR_SIMULATOR_DIMENSION      7

class SensorSimulatorNeuralNetwork : public SensorSimulator {
public:
    typedef boost::array<double, SENSOR_SIMULATOR_DIMENSION> SensorNoiseConfiguration;

    SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    virtual ~SensorSimulatorNeuralNetwork();

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    SensorNoiseConfiguration sensor_noise_configuration_;
};

#endif // SENSORSIMULATORNEURALNETWORK_H
