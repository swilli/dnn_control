#include "controllerneuralnetwork.h"
#include "constants.h"
#include "configuration.h"

#if PGMOS_ENABLE_ODOMETRY
const unsigned int ControllerNeuralNetwork::kDimensions = 6;
#else
#if PGMOS_ENABLE_ACCELEROMETER
const unsigned int ControllerNeuralNetwork::kDimensions = 7;
#else
const unsigned int ControllerNeuralNetwork::kDimensions = 4;
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
    const std::vector<double> normalized_u_v_t = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    const double u = 2.0 * kPi * normalized_u_v_t[0];
    const double v = acos(2.0 * normalized_u_v_t[1] - 1.0);
    const double t = normalized_u_v_t[2];
    thrust[0] = cos(u) * sin(v) * t;
    thrust[1] = sin(u) * sin(v) * t;
    thrust[2] = cos(v) * t;
    return thrust;
}
