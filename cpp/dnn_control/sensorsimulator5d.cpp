#include "sensorsimulator5d.h"
#include "utility.h"
#include "samplefactory.h"

SensorSimulator5D::SensorSimulator5D(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : SensorSimulator(5, asteroid) {
    for (unsigned int i = 0; i < dimensions_; ++i) {
        boost::normal_distribution<> normal(0.0, configuration.at(i));
        boost::variate_generator<boost::mt19937, boost::normal_distribution<> > distribution(SampleFactory::RandomNumberGenerator(), normal);

        normal_distributions_.push_back(distribution);
    }
}

SensorSimulator5D::~SensorSimulator5D() {

}

SensorData SensorSimulator5D::Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(5, 0.0);

    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};
    const double coef_mass = 1.0 / state[6];

    double norm_height_pow2 = 0.0;
    for (unsigned int i = 0; i < 3; ++i) {
        norm_height_pow2 += height[i] * height[i];
    }
    const double norm_height = sqrt(norm_height_pow2);

    const double vel_dot_height = velocity[0] * height[0] + velocity[1] * height[1] + velocity[2] * height[2];
    const double scaling = vel_dot_height / norm_height_pow2;

    const Vector3D velocity_vertical = {scaling * height[0], scaling * height[1], scaling * height[2]};
    const Vector3D velocity_remaining = {velocity[0] - velocity_vertical[0],
                                         velocity[1] - velocity_vertical[1],
                                         velocity[2] - velocity_vertical[2]};
    double norm_vel_vert = 0.0;
    double norm_vel_rem = 0.0;
    for (unsigned int i = 0; i < 3; ++i) {
        norm_vel_vert += velocity_vertical[i] * velocity_vertical[i];
        norm_vel_rem += velocity_remaining[i] * velocity_remaining[i];
    }
    norm_vel_vert = sqrt(norm_vel_vert);
    norm_vel_rem = sqrt(norm_vel_rem);

    sensor_data[0] = norm_vel_vert / norm_height;
    sensor_data[1] = norm_vel_rem / norm_height;


    Vector3D gravity_acceleration = asteroid_.GravityAtPosition(position);
    gravity_acceleration[0] *= coef_mass;
    gravity_acceleration[1] *= coef_mass;
    gravity_acceleration[2] *= coef_mass;

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
        sensor_data[i] += sensor_data[i] * normal_distributions_.at(i)();
        sensor_data[2+i] = perturbations_acceleration[i]
                + gravity_acceleration[i]
                - coriolis_acceleration[i]
                - euler_acceleration[i]
                - centrifugal_acceleration[i];
        sensor_data[2+i] += sensor_data[2+i] * normal_distributions_.at(2+i)();
    }

    return sensor_data;
}
