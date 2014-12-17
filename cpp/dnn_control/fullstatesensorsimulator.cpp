#include "fullstatesensorsimulator.h"

FullStateSensorSimulator::FullStateSensorSimulator(const Asteroid &asteroid, const double &sensor_noise) : SensorSimulator(asteroid), normal_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, sensor_noise))
{
    dimensions_ = 7;
}

FullStateSensorSimulator::~FullStateSensorSimulator()
{

}

void FullStateSensorSimulator::Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data)
{
    for (int i = 0; i < dimensions_; ++i) {
        sensor_data[i] = state[i] + state[i] * normal_distribution_();
    }
}
