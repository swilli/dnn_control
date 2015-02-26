#include "sensorsimulatorfullstate.h"
#include "configuration.h"

const unsigned int SensorSimulatorFullState::kDimensions = 6;

SensorSimulatorFullState::SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid, const Vector3D &target_position)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

    noise_configurations_ = std::vector<double>(dimensions_, 0.05);
    target_position_ = target_position;
}

SensorData SensorSimulatorFullState::Simulate(const SystemState &state, const Vector3D &, const Vector3D &, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = target_position_[i] - state[i];
        sensor_data[3+i] = -state[3+i];

#if SSFS_WITH_NOISE
        sensor_data[i] += sensor_data[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
        sensor_data[3+i] += sensor_data[3+i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(3+i));
#endif
    }

    return sensor_data;
}
