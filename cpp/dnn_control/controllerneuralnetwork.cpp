#include "controllerneuralnetwork.h"

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden) : Controller(CONTROLLER_DIMENSION, maximum_thrust), neural_network_(CONTROLLER_DIMENSION, num_hidden, 3) {

}

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights) : Controller(CONTROLLER_DIMENSION, maximum_thrust), neural_network_(CONTROLLER_DIMENSION, num_hidden, 3) {
    neural_network_.SetWeights(weights);
}

ControllerNeuralNetwork::~ControllerNeuralNetwork() {

}

Vector3D ControllerNeuralNetwork::GetThrustForSensorData(const SensorData &sensor_data) {
    const std::vector<double> normalized_thrust = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        thrust[i] = (normalized_thrust[i] - 0.5) * (maximum_thrust_ * 2.0);
    }
    return thrust;
}

void ControllerNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    neural_network_.SetWeights(weights);
}
