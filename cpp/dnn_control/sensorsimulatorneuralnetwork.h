#ifndef SENSORSIMULATORNEURALNETWORK_H
#define SENSORSIMULATORNEURALNETWORK_H

#include "sensorsimulator.h"

class SensorSimulatorNeuralNetwork : public SensorSimulator {
public:
    static const unsigned int kDimensions;

    SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid, const Vector3D &target_position);

    virtual ~SensorSimulatorNeuralNetwork();

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

private:
    Vector3D target_position_;
};

#endif // SENSORSIMULATORNEURALNETWORK_H
