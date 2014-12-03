#include "sensorsimulator.h"
#include "utility.h"
#include <math.h>

SensorSimulator::SensorSimulator(const Asteroid &asteroid, const double &sensor_noise) : asteroid_(asteroid), normal_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, sensor_noise))
{
}

void SensorSimulator::Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data)
{
    sensor_data[0] = 0.0; sensor_data[1] = 0.0; sensor_data[2] = 0.0; sensor_data[3] = 0.0; sensor_data[4] = 0.0;

    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};
    const double mass = state[6];

    Vector3D surface_point;
    double norm_height = 0.0;
    asteroid_.NearestPointOnSurface(position, surface_point, &norm_height);

    const Vector3D height = {position[0] - surface_point[0],
                              position[1] - surface_point[1],
                              position[2] - surface_point[2]};

    const double norm_height_pow2 = norm_height * norm_height;
    const double vel_dot_height = velocity[0] * height[0] + velocity[1] * height[1] + velocity[2] * height[2];
    const double scaling = vel_dot_height / norm_height_pow2;

    const Vector3D velocity_vertical = {scaling * height[0], scaling * height[1], scaling * height[2]};
    const Vector3D velocity_remaining = {velocity[0] - velocity_vertical[0],
                                         velocity[1] - velocity_vertical[1],
                                         velocity[2] - velocity_vertical[2]};
    double norm_vel_vert = 0.0;
    double norm_vel_rem = 0.0;
    for (int i = 0; i < 3; ++i) {
        norm_vel_vert += velocity_vertical[i] * velocity_vertical[i];
        norm_vel_rem += velocity_remaining[i] * velocity_remaining[i];
    }
    norm_vel_vert = sqrt(norm_vel_vert);
    norm_vel_rem = sqrt(norm_vel_rem);

    sensor_data[0] = norm_vel_vert / norm_height;
    sensor_data[1] = norm_vel_rem / norm_height;

    Vector3D gravity_acceleration;
    Vector3D euler_acceleration;
    Vector3D centrifugal_acceleration;
    Vector3D coriolis_acceleration;
    Vector3D angular_velocity;
    Vector3D angular_acceleration;
    Vector3D tmp;

    asteroid_.GravityAtPosition(position, gravity_acceleration);
    gravity_acceleration[0] /= mass;
    gravity_acceleration[1] /= mass;
    gravity_acceleration[2] /= mass;

    asteroid_.AngularVelocityAndAccelerationAtTime(time, angular_velocity, angular_acceleration);

    CrossProduct(angular_acceleration, position, euler_acceleration);
    CrossProduct(angular_velocity, position, tmp);
    CrossProduct(angular_velocity, tmp, centrifugal_acceleration);

    for(int i = 0; i < 3; ++i) {
        tmp[i] = angular_velocity[i] * 2.0;
    }

    CrossProduct(tmp, velocity, coriolis_acceleration);


    for (int i = 0; i < 3; ++i) {
        sensor_data[i] += sensor_data[i] * normal_distribution_();
        sensor_data[2+i] = perturbations_acceleration[i]
                + gravity_acceleration[i]
                - coriolis_acceleration[i]
                - euler_acceleration[i]
                - centrifugal_acceleration[i];
        sensor_data[2+i] += sensor_data[2+i] * normal_distribution_();
    }
}
