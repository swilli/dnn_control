#include "sensorsimulatorfullstate.h"

const unsigned int SensorSimulatorFullState::kDimensions = 8;

SensorSimulatorFullState::SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid)
    : SensorSimulator(kDimensions, sample_factory, asteroid), noise_configuration_(dimensions_, 0.05) {

}

SensorSimulatorFullState::~SensorSimulatorFullState() {

}

SensorData SensorSimulatorFullState::Simulate(const SystemState &state, const Vector3D &, const Vector3D &, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < dimensions_ - 1; ++i) {
        //const double sample = sample_factory_.SampleNormal(time, 0.0, noise_configuration_.at(i), i);
        //std::cout << "time " << time << " dim " << i << " sample " << sample << std::endl;
        sensor_data[i] = state[i]; // + state[i] * sample_factory_.SampleNormal(time, 0.0, noise_configuration_.at(i), i);
    }
    sensor_data[7] = time;
    return sensor_data;
}
