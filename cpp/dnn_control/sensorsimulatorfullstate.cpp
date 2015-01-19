#include "sensorsimulatorfullstate.h"

SensorSimulatorFullState::SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : SensorSimulator(SENSOR_SIMULATOR_DIMENSION, sample_factory, asteroid) {
    sensor_noise_configuration_ = configuration;
}

SensorSimulatorFullState::~SensorSimulatorFullState() {

}

SensorData SensorSimulatorFullState::Simulate(const SystemState &state, const Vector3D &, const Vector3D &, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < dimensions_ - 1; ++i) {
        sensor_data[i] = state[i]; // + state[i] * sample_factory_.SampleNormal(0.0, sensor_noise_configuration_.at(i));
    }
    sensor_data[7] = time;
    return sensor_data;
}
