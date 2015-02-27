#include "sensorsimulatorpartialstate.h"
#include "configuration.h"

const unsigned int SensorSimulatorPartialState::kDimensions = PGMOS_ENABLE_OPTICAL_FLOW * 3 + PGMOS_ENABLE_DIRECTION_SENSOR * 3 + PGMOS_ENABLE_ACCELEROMETER * 3;

SensorSimulatorPartialState::SensorSimulatorPartialState(SampleFactory &sample_factory, const Asteroid &asteroid)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

#if PGMOS_ENABLE_OPTICAL_FLOW
    noise_configurations_ = std::vector<double>(3, 0.05);
#endif
#if PGMOS_ENABLE_DIRECTION_SENSOR
    surface_point_ = sample_factory.SamplePointOnEllipsoidSurface(asteroid.SemiAxis());
    const std::vector<double> noise_direction_sensor(3, 0.05);
    noise_configurations_.insert(noise_configurations_.end(), noise_direction_sensor.begin(), noise_direction_sensor.end());
#endif
#if PGMOS_ENABLE_ACCELEROMETER
    const std::vector<double> noise_accelerometer(3, 0.05);
    noise_configurations_.insert(noise_configurations_.end(), noise_accelerometer.begin(), noise_accelerometer.end());
#endif

}

SensorData SensorSimulatorPartialState::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

#if PGMOS_ENABLE_OPTICAL_FLOW
    const Vector3D &velocity = {state[3], state[4], state[5]};

    const double coef_norm_height = 1e5 * sqrt(3) / VectorNorm(height);
    sensor_data[0] = velocity[0] * coef_norm_height;
    sensor_data[1] = velocity[1] * coef_norm_height;
    sensor_data[2] = velocity[2] * coef_norm_height;


    /*
    const double norm_height_pow2 = VectorDotProduct(height, height);
    const double norm_height = sqrt(norm_height_pow2);
    const double coef_norm_height = sqrt(3) / norm_height;

    const double velocity_dot_height = VectorDotProduct(velocity, height);
    const double scaling = velocity_dot_height / norm_height_pow2;

    const Vector3D &velocity_vertical = {scaling * height[0], scaling * height[1], scaling * height[2]};
    const Vector3D velocity_horizontal = VectorSub(velocity, velocity_vertical);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = velocity_vertical[i] * coef_norm_height;
        sensor_data[3+i] = velocity_horizontal[i] * coef_norm_height;
    }
    */

#if SSPS_WITH_NOISE
    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] += sensor_data[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
    }
#endif

    /*
    const double max_abs_sensor_value = 1e-4;
    for (unsigned int i = 0; i < 6; ++i) {
        double &sensor_value = sensor_data[i];
        if (sensor_value > max_abs_sensor_value) {
            sensor_value = max_abs_sensor_value;
        } else if (sensor_value < -max_abs_sensor_value) {
            sensor_value = -max_abs_sensor_value;
        }
        sensor_value = sensor_value * 0.5 / max_abs_sensor_value + 0.5;
    }
    */
#endif

#if PGMOS_ENABLE_DIRECTION_SENSOR
    const Vector3D &position = {state[0], state[1], state[2]};
    Vector3D direction = VectorSub(surface_point_, position);

#if SSPS_WITH_NOISE
    for (unsigned int i = 0; i < 3; ++i) {
        direction[i] += direction[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(PGMOS_ENABLE_OPTICAL_FLOW * 3 + i));
    }
#endif

    direction = VectorNormalize(direction);
    sensor_data[3] = direction[0];
    sensor_data[4] = direction[1];
    sensor_data[5] = direction[2];
#endif



#if PGMOS_ENABLE_ACCELEROMETER
    const Vector3D &position = {state[0], state[1], state[2]};
    const Vector3D gravity_acceleration = asteroid_.GravityAccelerationAtPosition(position);

    const boost::tuple<Vector3D, Vector3D> result_angular = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    const Vector3D angular_velocity = boost::get<0>(result_angular);
    const Vector3D angular_acceleration = boost::get<1>(result_angular);

    const Vector3D euler_acceleration = VectorCrossProduct(angular_acceleration, position);
    const Vector3D centrifugal_acceleration = VectorCrossProduct(angular_velocity, VectorCrossProduct(angular_velocity, position));

    Vector3D tmp;
    for (unsigned int i = 0; i < 3; ++i) {
        tmp[i] = angular_velocity[i] * 2.0;
    }

    const Vector3D coriolis_acceleration = VectorCrossProduct(tmp, velocity);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[4+i] = perturbations_acceleration[i]
                + gravity_acceleration[i]
                - coriolis_acceleration[i]
                - euler_acceleration[i]
                - centrifugal_acceleration[i];
    }
#endif

    return sensor_data;
}

