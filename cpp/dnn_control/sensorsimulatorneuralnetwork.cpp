#include "sensorsimulatorneuralnetwork.h"

const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 7;

SensorSimulatorNeuralNetwork::SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : SensorSimulator(kDimensions, sample_factory, asteroid){
    noise_configuration_ = configuration;
}

SensorSimulatorNeuralNetwork::~SensorSimulatorNeuralNetwork() {

}

SensorSimulator* SensorSimulatorNeuralNetwork::Clone(SampleFactory &sample_factory) const {
    return static_cast<SensorSimulator*>(new SensorSimulatorNeuralNetwork(sample_factory, asteroid_, noise_configuration_));
}

SensorData SensorSimulatorNeuralNetwork::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < dimensions_; ++i) {
        sensor_data[i] = state[i]; // + state[i] * sample_factory_.SampleNormal(0.0, sensor_noise_configuration_.at(i));
    }
    return sensor_data;
}


