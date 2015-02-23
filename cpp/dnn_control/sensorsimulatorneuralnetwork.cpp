#include "sensorsimulatorneuralnetwork.h"
#include "configuration.h"


#if PGMOSNN_ENABLE_ACCELEROMETER
const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 7;
#else
const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 4;
#endif

SensorSimulatorNeuralNetwork::SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

#if PGMOSNN_ENABLE_ACCELEROMETER
    sensor_maximum_absolute_ranges_ = {1e-4, 1e-4, 1e-4, 0.025, 0.025, 0.025, 0.025};
#else
    sensor_maximum_absolute_ranges_ = {1e-4, 1e-4, 1e-4, 1e-4}; //1e-4, 1e-4, 1e-4};
#endif

    if (sensor_maximum_absolute_ranges_.size() != dimensions_) {
        throw RangeMalConfigurationException();
    }

    noise_configurations_ = std::vector<double>(dimensions_, 0.05);
}

SensorSimulatorNeuralNetwork::~SensorSimulatorNeuralNetwork() {

}

SensorData SensorSimulatorNeuralNetwork::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    const Vector3D &velocity = {state[3], state[4], state[5]};

    sensor_data[0] = velocity[0];
    sensor_data[1] = velocity[1];
    sensor_data[2] = velocity[2];
    sensor_data[3] = state[6] / 500.0;

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

#if PGMOSNN_ENABLE_ACCELEROMETER
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

    /*
 for (unsigned int i = 0; i < dimensions_; ++i) {
        double &sensor_value = sensor_data[i];
        const double &max_abs_sensor_value = sensor_maximum_absolute_ranges_.at(i);
        if (sensor_value > max_abs_sensor_value) {
            sensor_value = max_abs_sensor_value;
        } else if (sensor_value < -max_abs_sensor_value) {
            sensor_value = -max_abs_sensor_value;
        }
        sensor_value = sensor_value * 0.5 / max_abs_sensor_value + 0.5;

#if SSNN_WITH_NOISE
        sensor_value += sensor_value * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
#endif

    }
    */

    return sensor_data;
}


