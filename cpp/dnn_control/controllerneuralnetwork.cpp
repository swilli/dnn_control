#include "controllerneuralnetwork.h"
#include "configuration.h"

#if PGMOS_ENABLE_ODOMETRY
const unsigned int ControllerNeuralNetwork::kDimensions = 6;
#else
#if PGMOS_ENABLE_ACCELEROMETER
const unsigned int ControllerNeuralNetwork::kDimensions = 7;
#else
const unsigned int ControllerNeuralNetwork::kDimensions = 3;
#endif
#endif

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, true), std::make_pair(num_hidden, true), std::make_pair(3, false)}, NeuralNetwork::ActivationFunctionType::Sigmoid) {
    number_of_parameters_ = neural_network_.Size();
}

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, true), std::make_pair(num_hidden, true), std::make_pair(3, false)}, NeuralNetwork::ActivationFunctionType::Sigmoid) {
    number_of_parameters_ = neural_network_.Size();
    SetWeights(weights);
}

ControllerNeuralNetwork::~ControllerNeuralNetwork() {

}

void ControllerNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    if (weights.size() == number_of_parameters_) {
        neural_network_.SetWeights(weights);
    } else {
        throw SizeMismatchException();
    }
}

Vector3D ControllerNeuralNetwork::GetThrustForSensorData(const SensorData &sensor_data) {
    const std::vector<double> unscaled_thrust = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        thrust[i] = (2.0 * unscaled_thrust[i] - 1.0) * maximum_thrust_;
    }
    return thrust;
}
