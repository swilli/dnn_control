#include "sensorsimulatoracceleration.h"
#include "utility.h"
#include "samplefactory.h"

SensorSimulatorAcceleration::SensorSimulatorAcceleration(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : SensorSimulator(4, asteroid) {
    for (unsigned int i = 0; i < dimensions_; ++i) {
        boost::normal_distribution<> normal(0.0, configuration.at(i));
        boost::variate_generator<boost::mt19937, boost::normal_distribution<> > distribution(SampleFactory::RandomNumberGenerator(), normal);

        normal_distributions_.push_back(distribution);
    }
}

SensorSimulatorAcceleration::~SensorSimulatorAcceleration() {

}

SensorData SensorSimulatorAcceleration::Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};

    const Vector3D gravity_acceleration = asteroid_.GravityAccelerationAtPosition(position);

    const boost::tuple<Vector3D, Vector3D> result_angular = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    const Vector3D angular_velocity = boost::get<0>(result_angular);
    const Vector3D angular_acceleration = boost::get<1>(result_angular);

    const Vector3D euler_acceleration = CrossProduct(angular_acceleration, position);
    const Vector3D centrifugal_acceleration = CrossProduct(angular_velocity, CrossProduct(angular_velocity, position));

    Vector3D tmp;
    for(unsigned int i = 0; i < 3; ++i) {
        tmp[i] = angular_velocity[i] * 2.0;
    }

    const Vector3D coriolis_acceleration = CrossProduct(tmp, velocity);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = perturbations_acceleration[i]
                + gravity_acceleration[i]
                - coriolis_acceleration[i]
                - euler_acceleration[i]
                - centrifugal_acceleration[i];

        sensor_data[i] += sensor_data[i] * normal_distributions_.at(i)();
    }
    sensor_data[3] = state[6] + state[6] * normal_distributions_.at(3)();

    return sensor_data;
}

