#include "sensorsimulatorfullstate.h"
#include "configuration.h"

const unsigned int SensorSimulatorFullState::kDimensions = 8;

SensorSimulatorFullState::SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

    noise_configurations_ =  std::vector<double>(dimensions_, 0.05);
}

SensorSimulatorFullState::~SensorSimulatorFullState() {

}

SensorData SensorSimulatorFullState::Simulate(const SystemState &state, const Vector3D &, const Vector3D &, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < dimensions_ - 1; ++i) {
        sensor_data[i] = state[i];

#if SSFS_WITH_NOISE
        sensor_data[i] = sensor_data[i] + sensor_data[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
#endif
    }
    sensor_data[7] = time;
    return sensor_data;
}
