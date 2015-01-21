#include "sensorsimulatorfullstate.h"

const unsigned int SensorSimulatorFullState::kDimensions = 8;

SensorSimulatorFullState::SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : SensorSimulator(kDimensions, sample_factory, asteroid, configuration) {

}

SensorSimulatorFullState::SensorSimulatorFullState(SampleFactory &sample_factory, const SensorSimulatorFullState &other) : SensorSimulator(kDimensions, sample_factory, other.asteroid_, other.noise_configuration_) {

}

SensorSimulatorFullState::~SensorSimulatorFullState() {

}

SensorSimulator *SensorSimulatorFullState::Clone(SampleFactory &sample_factory) const {
    return static_cast<SensorSimulator*>(new SensorSimulatorFullState(sample_factory, *this));
}

SensorData SensorSimulatorFullState::Simulate(const SystemState &state, const Vector3D &, const Vector3D &, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < dimensions_ - 1; ++i) {
        sensor_data[i] = state[i]; // + state[i] * sample_factory_.SampleNormal(0.0, sensor_noise_configuration_.at(i));
    }
    sensor_data[7] = time;
    return sensor_data;
}
