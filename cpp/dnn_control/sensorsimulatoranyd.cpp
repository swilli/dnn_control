#include "sensorsimulatoranyd.h"
#include "utility.h"
#include "samplefactory.h"

SensorSimulatorAnyD::SensorSimulatorAnyD(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : SensorSimulator(SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER * SENSOR_SIMULATOR_DATA_HISTORY, asteroid) {
    for (unsigned int i = 0; i < SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER; ++i) {
        boost::normal_distribution<> normal(0.0, configuration.at(i));
        boost::variate_generator<boost::mt19937, boost::normal_distribution<> > distribution(SampleFactory::RandomNumberGenerator(), normal);

        normal_distributions_.push_back(distribution);
    }

    first_simulation_ = true;
}

SensorSimulatorAnyD::~SensorSimulatorAnyD() {

}

SensorData SensorSimulatorAnyD::Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    boost::array<double, SENSOR_SIMULATOR_DATA_DIMENSIONS> sensor_seed;

    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};

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

    sensor_seed[0] = norm_vel_vert / norm_height;
    sensor_seed[1] = norm_vel_rem / norm_height;


    const Vector3D gravity_acceleration = asteroid_.GravityAccelerationAtPosition(position);

    const boost::tuple<Vector3D, Vector3D> result_angular = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    const Vector3D angular_velocity = boost::get<0>(result_angular);
    const Vector3D angular_acceleration = boost::get<1>(result_angular);

    const Vector3D euler_acceleration = CrossProduct(angular_acceleration, position);
    const Vector3D centrifugal_acceleration = CrossProduct(angular_velocity, CrossProduct(angular_velocity, position));

    Vector3D tmp;
    for (unsigned int i = 0; i < 3; ++i) {
        tmp[i] = angular_velocity[i] * 2.0;
    }

    const Vector3D coriolis_acceleration = CrossProduct(tmp, velocity);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_seed[2+i] = perturbations_acceleration[i]
                + gravity_acceleration[i]
                - coriolis_acceleration[i]
                - euler_acceleration[i]
                - centrifugal_acceleration[i];
    }

    // Every initial sensor value gets tripled and added with normal distributed noise
    boost::array<double, SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER> sensor_duplicates;
    for (unsigned int i = 0; i < SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER; ++i) {
        const int sensor_data_index = i % SENSOR_SIMULATOR_DATA_DIMENSIONS;
        sensor_duplicates[i] = sensor_seed[sensor_data_index] + sensor_seed[sensor_data_index] * normal_distributions_.at(i)();
    }

    if (first_simulation_) {
        first_simulation_ = false;
        // Assume spacecraft was standing still at current location with no sensor noise...
        for (unsigned int i = 0; i < SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER * SENSOR_SIMULATOR_DATA_HISTORY; ++i) {
            sensor_data_cache_[i] = sensor_duplicates[i % (SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER)];
        }
    } else {
        // Move sensor data down the cache
        for (unsigned int i = SENSOR_SIMULATOR_DATA_DIMENSIONS*SENSOR_SIMULATOR_DATA_MULTIPLIER*SENSOR_SIMULATOR_DATA_HISTORY - 1; i >= SENSOR_SIMULATOR_DATA_DIMENSIONS*SENSOR_SIMULATOR_DATA_MULTIPLIER; --i) {
            sensor_data_cache_[i] = sensor_data_cache_[i-SENSOR_SIMULATOR_DATA_DIMENSIONS*SENSOR_SIMULATOR_DATA_MULTIPLIER];
        }
        for (unsigned int i = 0; i < SENSOR_SIMULATOR_DATA_DIMENSIONS*SENSOR_SIMULATOR_DATA_MULTIPLIER; ++i) {
            sensor_data_cache_[i] = sensor_duplicates[i];
        }
    }

    for (unsigned int i = 0; i < 45; ++i) {
        sensor_data[i] = sensor_data_cache_[i];
    }

    return sensor_data;
}

void SensorSimulatorAnyD::ResetSimulator() {
    first_simulation_ = true;
}

