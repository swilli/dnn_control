#include "sensorsimulatorneuralnetwork.h"

const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 3;

SensorSimulatorNeuralNetwork::SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration, const Vector3D &target_position) : SensorSimulator(kDimensions, sample_factory, asteroid, configuration) {
    target_position_ = target_position;
}

SensorSimulatorNeuralNetwork::SensorSimulatorNeuralNetwork(const SensorSimulatorNeuralNetwork &other) : SensorSimulator(kDimensions, other.sample_factory_, other.asteroid_, other.noise_configuration_) {
    target_position_ = other.target_position_;
}

SensorSimulatorNeuralNetwork::~SensorSimulatorNeuralNetwork() {

}

SensorSimulator* SensorSimulatorNeuralNetwork::Clone() const {
    return static_cast<SensorSimulator*>(new SensorSimulatorNeuralNetwork(*this));
}

SensorData SensorSimulatorNeuralNetwork::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = target_position_[i] - state[i]; // + state[i] * sample_factory_.SampleNormal(0.0, sensor_noise_configuration_.at(i));
    }

    return sensor_data;
}


