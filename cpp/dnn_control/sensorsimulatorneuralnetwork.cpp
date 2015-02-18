#include "sensorsimulatorneuralnetwork.h"
#include "configuration.h"

const unsigned int SensorSimulatorNeuralNetwork::kDimensions = 3;

SensorSimulatorNeuralNetwork::SensorSimulatorNeuralNetwork(SampleFactory &sample_factory, const Asteroid &asteroid)
    : SensorSimulator(kDimensions, sample_factory, asteroid) {

    noise_configurations_ = std::vector<double>(dimensions_, 0.05);
}

SensorSimulatorNeuralNetwork::~SensorSimulatorNeuralNetwork() {

}

SensorData SensorSimulatorNeuralNetwork::Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    SensorData sensor_data(dimensions_, 0.0);

    const Vector3D &velocity = {state[3], state[4], state[5]};

    sensor_data[0] = state[3];
    sensor_data[1] = state[4];
    sensor_data[2] = state[5];

    return sensor_data;

    const double norm_height_pow2 = VectorDotProduct(height, height);
    const double norm_height = sqrt(norm_height_pow2);
    const double coef_norm_height = 1.0 / norm_height;

    const double velocity_dot_height = VectorDotProduct(velocity, height);
    const double scaling = velocity_dot_height / norm_height_pow2;

    const Vector3D &velocity_vertical = {scaling * height[0], scaling * height[1], scaling * height[2]};
    const Vector3D velocity_horizontal = VectorSub(velocity, velocity_vertical);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_data[i] = velocity_vertical[i] * coef_norm_height;
        sensor_data[3+i] = velocity_horizontal[i] * coef_norm_height;
    }

#if SSNN_WITH_NOISE
        sensor_data[i] += sensor_data[i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
        sensor_data[3+i] += sensor_data[3+i] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(3+i));
    }
#endif

    return sensor_data;
}


