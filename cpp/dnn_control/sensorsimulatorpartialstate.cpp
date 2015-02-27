#include "sensorsimulatorpartialstate.h"
#include "configuration.h"

const unsigned int SensorSimulatorPartialState::kDimensions = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3 + PGMOS_ENABLE_VELOCITY_OVER_HEIGHT * 3 + PGMOS_ENABLE_DIRECTION_SENSOR * 3 + PGMOS_ENABLE_ACCELEROMETER * 3;

SensorSimulatorPartialState::SensorSimulatorPartialState(SampleFactory &sample_factory, const Asteroid &asteroid)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

#if PGMOS_ENABLE_OPTICAL_FLOW
    noise_configurations_ = std::vector<double>(6, 0.05);
#endif
#if PGMOS_ENABLE_VELOCITY
    const std::vector<double> noise_velocity_sensor(3, 0.05);
    noise_configurations_.insert(noise_configurations_.end(), noise_velocity_sensor.begin(), noise_velocity_sensor.end());
#endif
#if PGMOS_ENABLE_VELOCITY_OVER_HEIGHT
    const std::vector<double> noise_velocity_over_height_sensor(3, 0.05);
    noise_configurations_.insert(noise_configurations_.end(), noise_velocity_over_height_sensor.begin(), noise_velocity_over_height_sensor.end());
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
    unsigned int offset = 0;
    double max_abs_sensor_value = 0.0;

    const Vector3D &position = {state[0], state[1], state[2]};
    const Vector3D &velocity = {state[3], state[4], state[5]};

    const double norm_height_pow2 = VectorDotProduct(height, height);
    const double norm_height = sqrt(norm_height_pow2);
    const double coef_norm_height = sqrt(3) / norm_height;

#if PGMOS_ENABLE_OPTICAL_FLOW
    const double velocity_dot_height = VectorDotProduct(velocity, height);
    const double scaling = velocity_dot_height / norm_height_pow2;

    const Vector3D &velocity_vertical = {scaling * height[0], scaling * height[1], scaling * height[2]};
    const Vector3D velocity_horizontal = VectorSub(velocity, velocity_vertical);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = velocity_vertical[i] * coef_norm_height;
        sensor_data[3+i] = velocity_horizontal[i] * coef_norm_height;
    }

#if SSPS_WITH_NOISE
    for (unsigned int i = 0; i < 6; ++i) {
        sensor_data[i] += sensor_data[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
    }
#endif

    max_abs_sensor_value = 1e-3;
    for (unsigned int i = 0; i < 6; ++i) {
        sensor_data[i] = Normalize(sensor_data[i], max_abs_sensor_value);
    }

#endif

#if PGMOS_ENABLE_VELOCITY
    offset= PGMOS_ENABLE_OPTICAL_FLOW * 6;

    sensor_data[offset] = velocity[0];
    sensor_data[offset + 1] = velocity[1];
    sensor_data[offset + 2] = velocity[2];

#if SSPS_WITH_NOISE
    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[offset + i] += sensor_data[offset + i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(offset + i));
    }
#endif

    max_abs_sensor_value = 0.5;
    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[offset + i] = Normalize(sensor_data[offset + i], max_abs_sensor_value);
    }

#endif

#if PGMOS_ENABLE_VELOCITY_OVER_HEIGHT
    offset = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3;

    sensor_data[offset] = velocity[0] * coef_norm_height;
    sensor_data[offset + 1] = velocity[1] * coef_norm_height;
    sensor_data[offset + 2] = velocity[2] * coef_norm_height;

#if SSPS_WITH_NOISE
    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[offset + i] += sensor_data[offset + i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(offset + i));
    }
#endif

    max_abs_sensor_value = 1e-3;
    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[offset + i] = Normalize(sensor_data[offset + i], max_abs_sensor_value);
    }

#endif

#if PGMOS_ENABLE_DIRECTION_SENSOR
    offset = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3 + PGMOS_ENABLE_VELOCITY_OVER_HEIGHT * 3;
    Vector3D direction = VectorSub(surface_point_, position);

#if SSPS_WITH_NOISE
    for (unsigned int i = 0; i < 3; ++i) {
        direction[i] += direction[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(PGMOS_ENABLE_OPTICAL_FLOW * 6 + i));
    }
#endif

    direction = VectorNormalize(direction);
    sensor_data[offset] = direction[0];
    sensor_data[offset + 1] = direction[1];
    sensor_data[offset + 2] = direction[2];

#endif

#if PGMOS_ENABLE_ACCELEROMETER
    offset = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3 + PGMOS_ENABLE_VELOCITY_OVER_HEIGHT * 3 + PGMOS_ENABLE_DIRECTION_SENSOR * 3;

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

    max_abs_sensor_value = 0.025;
    for (unsigned int i = 0; i < 3; ++i) {
        double sensor_value = perturbations_acceleration[i]
                + gravity_acceleration[i]
                - coriolis_acceleration[i]
                - euler_acceleration[i]
                - centrifugal_acceleration[i];

#if SSPS_WITH_NOISE
        sensor_value+= sensor_value * sample_factory_.SampleNormal(0.0, noise_configurations_.at(offset + i));
#endif

        sensor_data[offset + i] = Normalize(sensor_value, max_abs_sensor_value);
    }

#endif

    return sensor_data;
}

double SensorSimulatorPartialState::Normalize(const double &sensor_value, const double &max_abs_sensor_value) {
    double normalized_sensor_value = sensor_value;
    if (normalized_sensor_value > max_abs_sensor_value) {
        normalized_sensor_value = max_abs_sensor_value;
    } else if (normalized_sensor_value < -max_abs_sensor_value) {
        normalized_sensor_value = -max_abs_sensor_value;
    }
    normalized_sensor_value = normalized_sensor_value * 0.5 / max_abs_sensor_value + 0.5;
    return normalized_sensor_value;
}

