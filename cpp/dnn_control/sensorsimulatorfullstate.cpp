#include "sensorsimulatorfullstate.h"
#include "samplefactory.h"

SensorSimulatorFullState::SensorSimulatorFullState(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : SensorSimulator(7, asteroid) {
    for (unsigned int i = 0; i < dimensions_; ++i) {
        boost::normal_distribution<> normal(0.0, configuration.at(i));
        boost::variate_generator<boost::mt19937, boost::normal_distribution<> > distribution(SampleFactory::RandomNumberGenerator(), normal);

        normal_distributions_.push_back(distribution);
    }
}

SensorSimulatorFullState::~SensorSimulatorFullState() {

}

SensorData SensorSimulatorFullState::Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    for (unsigned int i = 0; i < dimensions_; ++i) {
        sensor_data[i] = state[i] + state[i] * normal_distributions_.at(i)();
    }

    return sensor_data;
}
