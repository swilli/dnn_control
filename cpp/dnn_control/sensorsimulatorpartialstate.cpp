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
    const boost::tuple<Vector3D, double, double> sampled_point = sample_factory.SamplePointOnEllipsoidSurface(asteroid.SemiAxis());
    surface_point_ = boost::get<0>(sampled_point);
    const std::vector<double> noise_direction_sensor(3, 0.05);
    noise_configurations_.insert(noise_configurations_.end(), noise_direction_sensor.begin(), noise_direction_sensor.end());
#endif
#if PGMOS_ENABLE_ACCELEROMETER
    const std::vector<double> noise_accelerometer(3, 0.05);
    noise_configurations_.insert(noise_configurations_.end(), noise_accelerometer.begin(), noise_accelerometer.end());
#endif

}

std::vector<double> SensorSimulatorPartialState::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    std::vector<double> sensor_data(dimensions_, 0.0);
    unsigned int offset = 0;
    double max_abs_sensor_value = 0.0;
    const double up_scale = 1000000.0;

    const Vector3D &position = {state[0], state[1], state[2]};
    const Vector3D &velocity = {state[3], state[4], state[5]};

    const double coef_norm_height = 1.0 / VectorNorm(height);

#if PGMOS_ENABLE_OPTICAL_FLOW
    const Vector3D &normalized_height = {height[0] * coef_norm_height, height[1] * coef_norm_height, height[2] * coef_norm_height};
    const double magn_velocity_parallel = VectorDotProduct(velocity, normalized_height);
    const Vector3D &velocity_parallel = {magn_velocity_parallel * normalized_height[0], magn_velocity_parallel * normalized_height[1], magn_velocity_parallel * normalized_height[2]};
    const Vector3D velocity_perpendicular = VectorSub(velocity, velocity_parallel);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = up_scale * velocity_parallel[i] * coef_norm_height;
        sensor_data[3+i] = up_scale * velocity_perpendicular[i] * coef_norm_height;
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

    sensor_data[offset] = velocity[0] * up_scale * coef_norm_height;
    sensor_data[offset + 1] = velocity[1] * up_scale * coef_norm_height;
    sensor_data[offset + 2] = velocity[2] * up_scale * coef_norm_height;

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

    max_abs_sensor_value = 1.0;
    sensor_data[offset] = Normalize(direction[0], max_abs_sensor_value);
    sensor_data[offset + 1] = Normalize(direction[1], max_abs_sensor_value);
    sensor_data[offset + 2] = Normalize(direction[2], max_abs_sensor_value);

#endif

#if PGMOS_ENABLE_ACCELEROMETER
    offset = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3 + PGMOS_ENABLE_VELOCITY_OVER_HEIGHT * 3 + PGMOS_ENABLE_DIRECTION_SENSOR * 3;

    const Vector3D gravity_acceleration = asteroid_.GravityAccelerationAtPosition(position);

    const boost::tuple<Vector3D, Vector3D> result_angular = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    const Vector3D &angular_velocity = boost::get<0>(result_angular);
    const Vector3D &angular_acceleration = boost::get<1>(result_angular);

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

        sensor_value *= up_scale
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

#if SSPS_NORMALIZE_SENSOR_VALUES
    if (normalized_sensor_value > max_abs_sensor_value) {
        normalized_sensor_value = max_abs_sensor_value;
    } else if (normalized_sensor_value < -max_abs_sensor_value) {
        normalized_sensor_value = -max_abs_sensor_value;
    }
    normalized_sensor_value = normalized_sensor_value * 0.5 / max_abs_sensor_value + 0.5;

#endif

    return normalized_sensor_value;
}

