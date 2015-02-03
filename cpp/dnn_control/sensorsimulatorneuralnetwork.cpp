#include "sensorsimulatorneuralnetwork.h"

const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 3;

SensorSimulatorNeuralNetwork::SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid, const Vector3D &target_position)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

    noise_configurations_ = std::vector<double>(dimensions_, 1e-20);
    target_position_ = target_position;
}

SensorSimulatorNeuralNetwork::~SensorSimulatorNeuralNetwork() {

}

SensorData SensorSimulatorNeuralNetwork::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = target_position_[i] - state[i];
        //sensor_data[i] = sensor_data[i] + sensor_data[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
    }

    return sensor_data;
}


