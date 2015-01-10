#include "sensorsimulatorfullstate.h"

SensorSimulatorFullState::SensorSimulatorFullState(const Asteroid &asteroid, const double &sensor_noise) : SensorSimulator(7, asteroid), normal_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, sensor_noise)) {

}

SensorSimulatorFullState::~SensorSimulatorFullState() {

}

SensorData SensorSimulatorFullState::Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < dimensions_; ++i) {
        sensor_data[i] = state[i] + state[i] * normal_distribution_();
    }

    return sensor_data;
}
