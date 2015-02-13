#include "sensorsimulatorneuralnetwork.h"
#include "configuration.h"

#if SSNN_SENSOR_DATA_TYPE == SSNN_SD_TYPE_TARGET
const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 6;
#elif SSNN_SENSOR_DATA_TYPE == SSNN_SD_TYPE_HOVER
const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 5;
#endif

SensorSimulatorNeuralNetwork::SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid, const Vector3D &target_position)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

    noise_configurations_ = std::vector<double>(dimensions_, 0.05);
    target_position_ = target_position;
}

SensorSimulatorNeuralNetwork::~SensorSimulatorNeuralNetwork() {

}

SensorData SensorSimulatorNeuralNetwork::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

#if SSNN_SENSOR_DATA_TYPE == SSNN_SD_TYPE_TARGET
    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = target_position_[i] - state[i];
        sensor_data[3+i] = -state[3+i];

#if SSNN_WITH_NOISE
        sensor_data[i] += sensor_data[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
        sensor_data[3+i] += sensor_data[3+i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
#endif

    }

#elif SSNN_SENSOR_DATA_TYPE == SSNN_SD_TYPE_HOVER
    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};

    const double norm_height_pow2 = VectorDotProduct(height, height);
    const double norm_height = sqrt(norm_height_pow2);

    const double velocity_dot_height = VectorDotProduct(velocity, height);
    const double scaling = velocity_dot_height / norm_height_pow2;
    const Vector3D velocity_vertical = {scaling * height[0], scaling * height[1], scaling * height[2]};
    const Vector3D velocity_horizontal = VectorSub(velocity, velocity_vertical);

    const double time_to_contact = (scaling >= 0.0 ? 1.0 : -1.0) * VectorNorm(velocity_vertical) / norm_height;
    const double optical_flow = VectorNorm(velocity_horizontal) / norm_height;

    sensor_data[0] = time_to_contact;
    sensor_data[1] = optical_flow;

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
        sensor_data[2+i] = perturbations_acceleration[i]
                + gravity_acceleration[i]
                - coriolis_acceleration[i]
                - euler_acceleration[i]
                - centrifugal_acceleration[i];
    }

#endif

    return sensor_data;
}


